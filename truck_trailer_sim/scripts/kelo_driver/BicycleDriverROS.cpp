#include "kelo_tulip/BicycleDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

kelo::BicycleDriver* driver;
std::vector<kelo::WheelConfig> configs;
ros::Publisher anglePub;

// Kelo Hub Hardware Specs
const double d_w = 0.0775; 
const double r_w = 0.0524;

void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Kinematic mapping: Unicycle -> Differential Actuators
    double v = msg->linear.x;
    double w = msg->angular.z;

    double vL = (v + (w * d_w / 2.0)) / r_w;
    double vR = (v - (w * d_w / 2.0)) / r_w;

    // Direct update to the high-speed thread variables
    driver->target_vL = (float)vL;
    driver->target_vR = (float)vR;
}

void timerCallback(const ros::TimerEvent&) {
    if (configs.empty()) return;

    txpdo1_t* sensors = driver->getRawSensorData(0);
    if (!sensors) return;

    // Publish pivot angle (phi) for MPC
    double phi = sensors->encoder_pivot - configs[0].a;
    std_msgs::Float64 m;
    m.data = atan2(sin(phi), cos(phi));
    anglePub.publish(m);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bicycle_driver_node");
    ros::NodeHandle nh("~");

    int n; 
    if(!nh.getParam("num_wheels", n)) n = 1;
    
    configs.resize(n);
    for (int i=0; i<n; i++) {
        std::string p = "wheel" + std::to_string(i);
        nh.getParam(p + "/ethercat_number", configs[i].ethercatNumber);
        nh.getParam(p + "/a", configs[i].a);
    }

    std::string dev; nh.param<std::string>("device", dev, "eth0");
    driver = new kelo::BicycleDriver(dev, &configs, n);

    if (!driver->initEthercat()) {
        ROS_ERROR("Hardware error: Could not initialize EtherCAT bus.");
        return -1;
    }

    anglePub = nh.advertise<std_msgs::Float64>("/kelo_angle", 10);
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, velCallback);
    ros::Timer t = nh.createTimer(ros::Duration(0.02), timerCallback);

    ROS_INFO("Bypass Driver Active. Motors should be locked.");

    ros::spin();
    return 0;
}