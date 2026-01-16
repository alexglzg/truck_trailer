#include "kelo_tulip/AngleDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>

kelo::AngleDriver* driver;
std::vector<kelo::WheelConfig> wheelConfigs;
std::vector<kelo::WheelData> wheelData;
ros::Publisher anglePublisher;
ros::Publisher batteryPublisher;
int nWheels = 0;

double joyVlinMax = 0.5;
double joyPhiMax = 1.57; // 45 deg
bool useJoy = false;

void applyTargets(double v, double phi) {
    // 180 flip + CCW convention
    driver->target_linear_v = (float)(-v);
    driver->target_phi = (float)(-phi);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    if (joy->buttons[5]) { // Deadman RB/R1
        useJoy = true;
        applyTargets(joy->axes[1] * joyVlinMax, joy->axes[0] * -joyPhiMax);
        driver->setCanChangeActive();
    } else {
        if (useJoy) applyTargets(0.0, 0.0);
        useJoy = false;
    }
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!useJoy) {
        applyTargets(msg->linear.x, msg->angular.z);
    }
}

void publishStatus(const ros::TimerEvent&) {
    if (nWheels == 0) return;
    txpdo1_t* swData = driver->getProcessData(wheelConfigs[0].ethercatNumber);
    if (swData) {
        std_msgs::Float32 angleMsg;
        double theta = swData->encoder_pivot - wheelConfigs[0].a;
        angleMsg.data = atan2(sin(theta), cos(theta));
        anglePublisher.publish(angleMsg);

        std_msgs::Float32 battMsg;
        battMsg.data = swData->voltage_bus;
        batteryPublisher.publish(battMsg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "angle_driver");
    ros::NodeHandle nh("~");

    if (!nh.getParam("num_wheels", nWheels)) nWheels = 1;
    wheelConfigs.resize(nWheels);
    wheelData.resize(nWheels, {true, false, false});

    for (int i = 0; i < nWheels; i++) {
        std::stringstream ss; ss << "wheel" << i;
        nh.getParam(ss.str() + "/ethercat_number", wheelConfigs[i].ethercatNumber);
        nh.getParam(ss.str() + "/a", wheelConfigs[i].a);
    }

    std::string device; nh.param<std::string>("device", device, "eth0:2");
    driver = new kelo::AngleDriver(device, &wheelConfigs, &wheelData, nWheels);

    if (!driver->initEthercat()) {
        ROS_ERROR("Failed to initialize EtherCAT Angle Driver.");
        return -1;
    }

    driver->setCanChangeActive();

    anglePublisher = nh.advertise<std_msgs::Float32>("/kelo_angle", 10);
    batteryPublisher = nh.advertise<std_msgs::Float32>("/battery", 10);
    ros::Subscriber cmdVelSub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);
    ros::Subscriber joySub = nh.subscribe("/joy", 10, joyCallback);
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), publishStatus);

    ROS_INFO("Angle Driver Active. Use Joy Deadman or cmd_vel.");
    ros::spin();
    return 0;
}