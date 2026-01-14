#ifndef BICYCLE_DRIVER_H
#define BICYCLE_DRIVER_H

#include <string>
#include <vector>
#include <boost/thread.hpp>

// Original KELO includes
extern "C" {
#include "kelo_tulip/soem/ethercat.h"
#include "kelo_tulip/EtherCATModule.h"
}
#include "kelo_tulip/WheelConfig.h"

namespace kelo {

// Minimal WheelData to satisfy the constructor logic if needed
struct BicycleWheelData {
    bool error;
    bool errorTimestamp;
};

class BicycleDriver {
public:
    BicycleDriver(std::string device, std::vector<kelo::WheelConfig>* configs, int nWheels);
    virtual ~BicycleDriver();

    bool initEthercat();
    void closeEthercat();

    // Raw Actuator Interface
    txpdo1_t* getRawSensorData(int wheel_idx);
    void sendRawCommand(int wheel_idx, rxpdo1_t* command);

private:
    void ethercatHandler(); 

    std::string device;
    std::vector<kelo::WheelConfig>* wheelConfigs;
    int nWheels;
    
    volatile bool stopThread;
    boost::thread* ethercatThread;

    // SOEM Low-level context objects
    ecx_contextt ecx_context;
    ecx_portt ecx_port;
    ec_slavet ecx_slave[EC_MAXSLAVE];
    int ecx_slavecount;
    ec_groupt ec_group[EC_MAXGROUP];
    char IOmap[4096];
};

} // namespace kelo
#endif