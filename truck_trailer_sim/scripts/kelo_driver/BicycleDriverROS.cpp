#include "kelo_tulip/BicycleDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

kelo::PlatformDriver* driver;
std::vector<kelo::WheelConfig> wheelConfigs;
ros::Publisher anglePublisher;

// Actuator Targets
double target_vk = 0.0; // Linear speed (m/s)
double target_wk = 0.0; // Steering rate (rad/s)
bool useJoy = false;

// Hardware Specs (Validated from Original Files)
const double d_w = 0.0775; // Wheel base inside the hub
const double r_w = 0.0524; // Wheel radius

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!useJoy) {
        target_vk = msg->linear.x;
        target_wk = msg->angular.z;
    }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    if (joy->buttons[5]) { // R1 deadman switch
        useJoy = true;
        target_vk = joy->axes[1] * 1.0; 
        target_wk = joy->axes[0] * 1.5; // Slightly faster steering for joy
    } else {
        if (useJoy) { target_vk = 0.0; target_wk = 0.0; }
        useJoy = false;
    }
}

void updateHardware(const ros::TimerEvent&) {
    if (wheelConfigs.empty()) return;

    // 1. GET SENSORS
    txpdo1_t* sensors = driver->getRawSensorData(0);

    // 2. BICYCLE KINEMATICS (vk, wk -> Motor rad/s)
    double vL_rads = (target_vk + (target_wk * d_w / 2.0)) / r_w;
    double vR_rads = (target_vk - (target_wk * d_w / 2.0)) / r_w;

    // 3. CONSTRUCT RAW COMMAND
    rxpdo1_t command;
    command.timestamp = sensors->sensor_ts; // Absolute sync
    command.command1 = 7;                   // Enable Motors + Velocity Mode
    command.limit1_p = 20.0; command.limit1_n = -20.0; // 20A limit
    command.limit2_p = 20.0; command.limit2_n = -20.0;
    
    command.setpoint1 = (float)vL_rads;
    command.setpoint2 = (float)-vR_rads; // Mirroring inversion

    // 4. WRITE DIRECT TO ETHERCAT
    driver->sendRawCommand(0, &command);

    // 5. FEEDBACK FOR MPC (State phi)
    double current_phi = sensors->encoder_pivot - wheelConfigs[0].a;
    current_phi = atan2(sin(current_phi), cos(current_phi));
    
    std_msgs::Float64 msg;
    msg.data = current_phi;
    anglePublisher.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kelo_bicycle_driver");
    ros::NodeHandle nh("~");

    // Load wheel config for 'a' (mount offset) and ethercat number
    int nWheels; nh.getParam("num_wheels", nWheels);
    wheelConfigs.resize(nWheels);
    for (int i=0; i<nWheels; i++) {
        std::string p = "wheel" + std::to_string(i);
        nh.getParam(p + "/ethercat_number", wheelConfigs[i].ethercatNumber);
        nh.getParam(p + "/a", wheelConfigs[i].a);
    }

    std::string device; nh.param<std::string>("device", device, "eth0");
    driver = new kelo::PlatformDriver(device, &wheelConfigs, nWheels);

    if (!driver->initEthercat()) {
        ROS_ERROR("Hardware Init Failed. Check Ethernet Connection.");
        return -1;
    }

    ros::Subscriber joySub = nh.subscribe("/joy", 10, joyCallback);
    ros::Subscriber cmdSub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);
    anglePublisher = nh.advertise<std_msgs::Float64>("kelo_angle", 10);

    // 50Hz Loop
    ros::Timer timer = nh.createTimer(ros::Duration(0.02), updateHardware);
    
    ros::spin();
    return 0;
}