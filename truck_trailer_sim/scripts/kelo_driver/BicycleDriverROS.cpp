#include "kelo_tulip/BicycleDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

// Use kelo namespace for the objects
kelo::BicycleDriver* driver;
std::vector<kelo::WheelConfig> configs;
ros::Publisher anglePub;
double vk = 0.0, wk = 0.0;

// Bicycle Constants
const double d_w = 0.0775; 
const double r_w = 0.0524;

void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    vk = msg->linear.x;
    wk = msg->angular.z;
}

void timerCallback(const ros::TimerEvent&) {
    if (configs.empty()) return;

    txpdo1_t* feedback = driver->getRawSensorData(0);
    if (!feedback) return;

    // Kinematics
    double vL = (vk + (wk * d_w / 2.0)) / r_w;
    double vR = (vk - (wk * d_w / 2.0)) / r_w;

    // Command
    rxpdo1_t command;
    command.timestamp = feedback->sensor_ts;
    command.command1 = 7; // Enable + Velocity
    command.limit1_p = 20.0; command.limit1_n = -20.0;
    command.limit2_p = 20.0; command.limit2_n = -20.0;
    command.setpoint1 = (float)vL;
    command.setpoint2 = (float)-vR;

    driver->sendRawCommand(0, &command);

    // Feedback phi
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
        ROS_ERROR("Failed to start EtherCAT");
        return -1;
    }

    anglePub = nh.advertise<std_msgs::Float64>("/kelo_angle", 10);
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, velCallback);
    ros::Timer t = nh.createTimer(ros::Duration(0.02), timerCallback);

    ros::spin();
    return 0;
}