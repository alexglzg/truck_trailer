#include "kelo_tulip/BicycleDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

kelo::BicycleDriver* driver;
std::vector<kelo::WheelConfig> configs;
ros::Publisher anglePub;
double vk = 0.0, wk = 0.0;
bool useJoy = false;

// Kelo Dimensions
const double d_w = 0.0775; 
const double r_w = 0.0524;

void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!useJoy) { vk = msg->linear.x; wk = msg->angular.z; }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    if (joy->buttons[5]) { // R1/RB Deadman switch
        useJoy = true;
        vk = joy->axes[1] * 1.0; 
        wk = joy->axes[0] * 1.5;
    } else {
        if (useJoy) { vk = 0.0; wk = 0.0; }
        useJoy = false;
    }
}

void updateHardware(const ros::TimerEvent&) {
    if (configs.empty()) return;

    txpdo1_t* feedback = driver->getRawSensorData(0);
    if (!feedback) return;

    // --- CRITICAL: THE HARDWARE ARMING SEQUENCE ---
    // 1. Sync timestamp to hardware sensor clock + offset
    // This stops the hardware safety watchdog from tripping.
    uint64_t next_ts = feedback->sensor_ts + 1000;

    // 2. Map Unicycle to Differential
    double vL = (vk + (wk * d_w / 2.0)) / r_w;
    double vR = (vk - (wk * d_w / 2.0)) / r_w;

    // 3. Construct the 'Locking' Command
    rxpdo1_t command;
    command.timestamp = next_ts;
    
    // command1 = 7 (Bit 0: Enable M1, Bit 1: Enable M2, Bit 2: Velocity Mode)
    command.command1 = 7; 
    
    command.limit1_p = 20.0; command.limit1_n = -20.0;
    command.limit2_p = 20.0; command.limit2_n = -20.0;
    command.setpoint1 = (float)vL;
    command.setpoint2 = (float)-vR;

    // 4. Send directly to EtherCAT output buffer
    driver->sendRawCommand(0, &command);

    // 5. Feedback
    double phi = feedback->encoder_pivot - configs[0].a;
    std_msgs::Float64 m; m.data = atan2(sin(phi), cos(phi));
    anglePub.publish(m);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bicycle_driver_node");
    ros::NodeHandle nh("~");

    int n; nh.getParam("num_wheels", n);
    configs.resize(n);
    for (int i=0; i<n; i++) {
        std::string p = "wheel" + std::to_string(i);
        nh.getParam(p + "/ethercat_number", configs[i].ethercatNumber);
        nh.getParam(p + "/a", configs[i].a);
    }

    std::string dev; nh.param<std::string>("device", dev, "eth0");
    driver = new kelo::BicycleDriver(dev, &configs, n);

    if (!driver->initEthercat()) {
        ROS_ERROR("EtherCAT Init Failed. Slaves might not be Operational.");
        return -1;
    }

    anglePub = nh.advertise<std_msgs::Float64>("kelo_angle", 10);
    ros::Subscriber s1 = nh.subscribe("/cmd_vel", 10, velCallback);
    ros::Subscriber s2 = nh.subscribe("/joy", 10, joyCallback);
    ros::Timer t = nh.createTimer(ros::Duration(0.02), updateHardware);

    ros::spin();
    return 0;
}