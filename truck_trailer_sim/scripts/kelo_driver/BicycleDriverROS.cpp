#include "kelo_tulip/BicycleDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

kelo::BicycleDriver* driver;
std::vector<kelo::WheelConfig> configs;

ros::Publisher anglePub, imuPub, battPub, statusPub;

double vk = 0.0, wk = 0.0;
bool useJoy = false;

// Kelo Hub Dimensions
const double d_w = 0.0775; 
const double r_w = 0.0524;

void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!useJoy) { vk = msg->linear.x; wk = msg->angular.z; }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    if (joy->buttons[5]) { // RB/R1 Deadman
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

    // 1. ACTUATOR MATH (Bicycle/Unicycle Bypass)
    double vL = (vk + (wk * d_w / 2.0)) / r_w;
    double vR = (vk - (wk * d_w / 2.0)) / r_w;

    // 2. CONSTRUCT RAW PACKET
    rxpdo1_t cmd;
    cmd.timestamp = feedback->sensor_ts; // Absolute Sync
    cmd.command1 = 7;                   // Enable + Velocity Mode
    cmd.limit1_p = 20.0; cmd.limit1_n = -20.0;
    cmd.limit2_p = 20.0; cmd.limit2_n = -20.0;
    cmd.setpoint1 = (float)vL;
    cmd.setpoint2 = (float)-vR;

    driver->sendRawCommand(0, &cmd);

    // 3. PUBLISHERS
    // Angle phi
    std_msgs::Float64 m; 
    double phi = feedback->encoder_pivot - configs[0].a;
    m.data = atan2(sin(phi), cos(phi));
    anglePub.publish(m);

    // IMU
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now();
    imu.angular_velocity.z = feedback->gyro_z;
    imu.linear_acceleration.x = feedback->accel_x;
    imuPub.publish(imu);

    // Battery
    std_msgs::Float32 b; b.data = feedback->voltage_bus;
    battPub.publish(b);
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
        ROS_ERROR("Failed to start EtherCAT Master.");
        return -1;
    }

    anglePub = nh.advertise<std_msgs::Float64>("/kelo_angle", 10);
    imuPub = nh.advertise<sensor_msgs::Imu>("imu", 10);
    battPub = nh.advertise<std_msgs::Float32>("battery", 10);
    
    ros::Subscriber s1 = nh.subscribe("/cmd_vel", 10, velCallback);
    ros::Subscriber s2 = nh.subscribe("/joy", 10, joyCallback);
    ros::Timer t = nh.createTimer(ros::Duration(0.02), updateHardware);

    ros::spin();
    return 0;
}