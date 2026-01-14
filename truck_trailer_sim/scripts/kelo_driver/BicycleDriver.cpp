#include "kelo_tulip/BicycleDriver.h"
#include <iostream>

namespace kelo {

PlatformDriver::PlatformDriver(std::string device, std::vector<WheelConfig>* configs, int nWheels)
    : device(device), wheelConfigs(configs), nWheels(nWheels), stopThread(false) {
    
    ecx_context.port = &ecx_port;
    ecx_context.slavelist = &ecx_slave[0];
    ecx_context.slavecount = &ecx_slavecount;
    ecx_context.maxslave = EC_MAXSLAVE;
    ecx_context.esibuf = NULL; 
    ecx_context.esimap = NULL;
}

PlatformDriver::~PlatformDriver() {
    closeEthercat();
}

bool PlatformDriver::initEthercat() {
    if (!ecx_init(&ecx_context, const_cast<char*>(device.c_str()))) {
        return false;
    }

    if (ecx_config_init(&ecx_context, TRUE) <= 0) {
        return false;
    }

    ecx_config_map_group(&ecx_context, IOmap, 0);
    ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    // Request Operational State
    ecx_slave[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ecx_context, 0);
    ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

    if (ecx_slave[0].state != EC_STATE_OPERATIONAL) {
        return false;
    }

    // Start the 1ms I/O thread
    ethercatThread = new boost::thread(boost::bind(&PlatformDriver::ethercatHandler, this));
    return true;
}

void PlatformDriver::ethercatHandler() {
    while (!stopThread) {
        ecx_receive_processdata(&ecx_context, 1000);
        ecx_send_processdata(&ecx_context);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
}

txpdo1_t* PlatformDriver::getRawSensorData(int wheel_index) {
    int slave = (*wheelConfigs)[wheel_index].ethercatNumber;
    return (txpdo1_t*) ecx_slave[slave].inputs;
}

void PlatformDriver::sendRawCommand(int wheel_index, rxpdo1_t* command) {
    int slave = (*wheelConfigs)[wheel_index].ethercatNumber;
    rxpdo1_t* ecData = (rxpdo1_t*) ecx_slave[slave].outputs;
    *ecData = *command;
}

void PlatformDriver::closeEthercat() {
    stopThread = true;
    if (ethercatThread) {
        ethercatThread->join();
    }
    ecx_slave[0].state = EC_STATE_INIT;
    ecx_writestate(&ecx_context, 0);
    ecx_close(&ecx_context);
}

} // namespace kelo