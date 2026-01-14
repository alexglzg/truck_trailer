#include "kelo_tulip/BicycleDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

kelo::BicycleDriver* driver;
std::vector<kelo::WheelConfig> configs;
ros::Publisher anglePub;

// Kelo Hub Hardware Dimensions
const double d_w = 0.0775; 
const double r_w = 0.0524;

void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double v = msg->linear.x;
    double w = msg->angular.z;
    // Map Unicycle (v,w) to Hub Differential (vL, vR)
    driver->target_vL = (float)((v + (w * d_w / 2.0)) / r_w);
    driver->target_vR = (float)((v - (w * d_w / 2.0)) / r_w);
}

void timerCallback(const ros::TimerEvent&) {
    if (configs.empty()) return;
    txpdo1_t* feedback = driver->getRawSensorData(0);
    if (!feedback) return;
    
    // Publish pivot angle (phi)
    double phi = feedback->encoder_pivot - configs[0].a;
    std_msgs::Float64 m; m.data = atan2(sin(phi), cos(phi));
    anglePub.publish(m);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bicycle_driver_node");
    ros::NodeHandle nh("~");

    int n; nh.param<int>("num_wheels", n, 1);
    configs.resize(n);
    for (int i=0; i<n; i++) {
        std::string p = "wheel" + std::to_string(i);
        nh.getParam(p + "/ethercat_number", configs[i].ethercatNumber);
        nh.getParam(p + "/a", configs[i].a);
    }

    std::string dev; nh.param<std::string>("device", dev, "eth0");
    driver = new kelo::BicycleDriver(dev, &configs, n);

    if (!driver->initEthercat()) {
        ROS_ERROR("Kelo Hub Handshake Failed. Motor configuration rejected.");
        return -1;
    }

    anglePub = nh.advertise<std_msgs::Float64>("kelo_angle", 10);
    ros::Subscriber s = nh.subscribe("/cmd_vel", 10, velCallback);
    ros::Timer t = nh.createTimer(ros::Duration(0.02), timerCallback);

    ROS_INFO("Bicycle Driver Operational. Motors are electrically locked.");
    ros::spin();
    return 0;
}