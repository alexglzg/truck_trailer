#include "kelo_tulip/BicycleDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>

kelo::BicycleDriver* driver;
std::vector<kelo::WheelConfig> wheelConfigs;
std::vector<kelo::WheelData> wheelData;
ros::Publisher anglePublisher;
ros::Publisher batteryPublisher;
int nWheels = 0;

const double d_w = 0.0775; 
const double r_w = 0.0524;

double joyVlinMax = 1.0;
double joyVaMax = 1.0;
bool useJoy = false;

void applyBicycleKinematics(double v, double w) {
    // FIX 1: Negative 'w' for Counter-Clockwise Steering (ROS Standard)
    // FIX 2: Negative 'v' for Forward/Backward Orientation Fix
    double v_corrected = -v; 
    double w_corrected = -w;

    driver->target_vL = (float)((v_corrected + (w_corrected * d_w / 2.0)) / r_w);
    driver->target_vR = (float)((v_corrected - (w_corrected * d_w / 2.0)) / r_w);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    if (joy->buttons[5]) { // Deadman switch
        useJoy = true;
        applyBicycleKinematics(joy->axes[1] * joyVlinMax, joy->axes[0] * joyVaMax);
        driver->setCanChangeActive();
    } else {
        if (useJoy) {
            driver->target_vL = 0;
            driver->target_vR = 0;
        }
        useJoy = false;
    }
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!useJoy) {
        applyBicycleKinematics(msg->linear.x, msg->angular.z);
    }
}

void publishStatus(const ros::TimerEvent&) {
    if (nWheels == 0) return;

    txpdo1_t* swData = driver->getProcessData(wheelConfigs[0].ethercatNumber);
    if (swData) {
        // Pivot Angle Publisher
        std_msgs::Float32 angleMsg;
        double theta = swData->encoder_pivot - wheelConfigs[0].a;
        angleMsg.data = atan2(sin(theta), cos(theta));
        anglePublisher.publish(angleMsg);

        // Battery Publisher
        std_msgs::Float32 battMsg;
        battMsg.data = swData->voltage_bus; // In Volts
        batteryPublisher.publish(battMsg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bicycle_driver");
    ros::NodeHandle nh("~");

    nh.getParam("num_wheels", nWheels);
    wheelConfigs.resize(nWheels);
    wheelData.resize(nWheels, {true, false, false});

    for (int i = 0; i < nWheels; i++) {
        std::string group = "wheel" + std::to_string(i);
        nh.getParam(group + "/ethercat_number", wheelConfigs[i].ethercatNumber);
        nh.getParam(group + "/a", wheelConfigs[i].a);
    }

    std::string device; nh.getParam("device", device);
    driver = new kelo::BicycleDriver(device, &wheelConfigs, &wheelData, nWheels);

    if (!driver->initEthercat()) {
        ROS_ERROR("Hardware error: Check EtherCAT priority and Slave IDs.");
        return -1;
    }

    nh.getParam("vlin_max", joyVlinMax);
    nh.getParam("va_max", joyVaMax);

    driver->setCanChangeActive();

    anglePublisher = nh.advertise<std_msgs::Float32>("/kelo_angle", 10);
    batteryPublisher = nh.advertise<std_msgs::Float32>("/battery", 10);
    ros::Subscriber cmdVelSub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);
    ros::Subscriber joySub = nh.subscribe("/joy", 10, joyCallback);
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), publishStatus);

    ROS_INFO("Bicycle Driver active.");
    ros::spin();
    return 0;
}