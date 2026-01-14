#include "kelo_tulip/PlatformDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>

// --- GLOBALS (Retained from Original Driver) ---
kelo::PlatformDriver* driver;
std::vector<kelo::WheelConfig> wheelConfigs;
std::vector<kelo::WheelData> wheelData;

// --- PUBLISHERS ---
ros::Publisher anglePublisher;   // Current pivot angle phi for MPC
ros::Publisher imuPublisher;     // Raw IMU from the wheel
ros::Publisher batteryPublisher; // Bus voltage
ros::Publisher statusPublisher;  // EtherCAT/Hardware status

// --- STATE VARIABLES ---
double cmd_vk = 0.0; // Linear velocity target (m/s)
double cmd_wk = 0.0; // Steering rate target (rad/s)
bool useJoy = false; 

// --- HARDWARE PARAMETERS (Extracted from Original PlatformDriver.cpp) ---
const double d_w = 0.0775; // Distance between hub wheels
const double r_w = 0.0524; // Wheel radius (0.1048 diameter / 2)

// --- CALLBACKS ---

void cmdVelKeloCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!useJoy) {
        cmd_vk = msg->linear.x;   // Interpreted as vk
        cmd_wk = msg->angular.z;  // Interpreted as wk (phi_dot)
    }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    // Button 5 (R1/RB) is the deadman switch
    if (joy->buttons[5]) {
        useJoy = true;
        // Map Left Stick: Y-axis to linear speed, X-axis to steering rate
        cmd_vk = joy->axes[1] * 1.0; 
        cmd_wk = joy->axes[0] * 1.0;
    } else {
        if (useJoy) { cmd_vk = 0.0; cmd_wk = 0.0; }
        useJoy = false;
    }
}

// --- CORE UPDATE LOOP (Timer-driven at 50Hz) ---

void updateHardware(const ros::TimerEvent&) {
    if (wheelConfigs.empty()) return;

    // 1. DATA ACCESS
    // slave index 0 refers to the first wheel in your config
    int slave_id = wheelConfigs[0].ethercatNumber;
    txpdo1_t* swData = driver->getProcessData(slave_id);

    // 2. ABSOLUTE ANGLE FEEDBACK (For MPC State)
    // raw_encoder - a (mount offset). Normalize to [-pi, pi]
    double current_phi = swData->encoder_pivot - wheelConfigs[0].a;
    current_phi = atan2(sin(current_phi), cos(current_phi));

    std_msgs::Float64 angleMsg;
    angleMsg.data = current_phi;
    anglePublisher.publish(angleMsg);

    // 3. KINEMATIC MAPPING (Unicycle -> Differential)
    // Calculating motor angular velocities (rad/s)
    double vL_rads = (cmd_vk + (cmd_wk * d_w / 2.0)) / r_w;
    double vR_rads = (cmd_vk - (cmd_wk * d_w / 2.0)) / r_w;

    // 4. CONSTRUCT ETHERCAT PACKET (Direct Bypass of VelocityPlatformController)
    rxpdo1_t rxdata;
    // Safety: Increment timestamp based on sensor feedback to keep watchdog alive
    rxdata.timestamp = swData->sensor_ts + 100 * 1000; 

    // Mode: Enable both motors and set to Velocity Mode
    rxdata.command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_VELOCITY;

    // Hardware Current Limits: 20 Amperes (The 'currentDrive' value from original code)
    rxdata.limit1_p = 20.0; 
    rxdata.limit1_n = -20.0;
    rxdata.limit2_p = 20.0; 
    rxdata.limit2_n = -20.0;

    // Setpoints: L and R motor speeds. 
    // Note: setpoint2 is inverted to account for the mirrored mounting of the dual motors.
    rxdata.setpoint1 = vL_rads;
    rxdata.setpoint2 = -vR_rads;

    // 5. SEND TO HARDWARE
    // Injects data directly into the EtherCAT memory map managed by PlatformDriver
    driver->setProcessData(slave_id, &rxdata);

    // 6. DIAGNOSTIC PUBLISHING
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now();
    imu.angular_velocity.z = swData->gyro_z;
    imu.linear_acceleration.x = swData->accel_x;
    imu.linear_acceleration.y = swData->accel_y;
    imuPublisher.publish(imu);

    std_msgs::Float32 batt;
    batt.data = swData->voltage_bus;
    batteryPublisher.publish(batt);

    std_msgs::Int32 status;
    status.data = driver->getDriverStatus();
    statusPublisher.publish(status);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kelo_bicycle_driver");
    ros::NodeHandle nh("~");

    // Load Configuration (Mirrors readWheelConfig from original)
    int nWheels;
    nh.getParam("num_wheels", nWheels);
    wheelConfigs.resize(nWheels);
    wheelData.resize(nWheels);
    for (int i=0; i<nWheels; i++) {
        std::string p = "wheel" + std::to_string(i);
        nh.getParam(p + "/ethercat_number", wheelConfigs[i].ethercatNumber);
        nh.getParam(p + "/a", wheelConfigs[i].a); // This is your 'pi' mount offset
    }

    // Initialize EtherCAT Master
    std::string device;
    nh.param<std::string>("device", device, "eth0");
    std::vector<kelo::EtherCATModule*> modules;
    driver = new kelo::PlatformDriver(device, modules, &wheelConfigs, &wheelData, 0, nWheels);

    if (!driver->initEthercat()) {
        ROS_ERROR("Failed to initialize EtherCAT Master.");
        return -1;
    }

    // ROS Interface
    ros::Subscriber joySub = nh.subscribe("/joy", 10, joyCallback);
    ros::Subscriber cmdSub = nh.subscribe("/cmd_vel", 10, cmdVelKeloCallback);
    
    anglePublisher = nh.advertise<std_msgs::Float64>("/kelo_angle", 10);
    imuPublisher = nh.advertise<sensor_msgs::Imu>("imu", 10);
    batteryPublisher = nh.advertise<std_msgs::Float32>("battery", 10);
    statusPublisher = nh.advertise<std_msgs::Int32>("status", 10);

    // Start 50Hz update loop
    ros::Timer timer = nh.createTimer(ros::Duration(0.02), updateHardware);

    ros::spin();

    driver->closeEthercat();
    return 0;
}