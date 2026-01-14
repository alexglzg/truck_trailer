#ifndef KELO_PLATFORM_DRIVER_H
#define KELO_PLATFORM_DRIVER_H

#include <string>
#include <vector>
#include <boost/thread.hpp>
#include "kelo_tulip/soem/ethercat.h"
#include "kelo_tulip/EtherCATModule.h"

namespace kelo {

class PlatformDriver {
public:
    PlatformDriver(std::string device, std::vector<WheelConfig>* configs, int nWheels);
    virtual ~PlatformDriver();

    bool initEthercat();
    void closeEthercat();

    // The raw Actuator Interface
    txpdo1_t* getRawSensorData(int wheel_index);
    void sendRawCommand(int wheel_index, rxpdo1_t* command);

private:
    void ethercatHandler(); // 1ms Raw I/O Loop

    std::string device;
    std::vector<WheelConfig>* wheelConfigs;
    int nWheels;
    
    bool stopThread;
    boost::thread* ethercatThread;

    // SOEM Low-Level Context
    ecx_contextt ecx_context;
    ecx_portt ecx_port;
    ec_slavet ecx_slave[EC_MAXSLAVE];
    int ecx_slavecount;
    char IOmap[4096];
    OSAL_THREAD_HANDLE thread_handle;
};

} // namespace kelo
#endif