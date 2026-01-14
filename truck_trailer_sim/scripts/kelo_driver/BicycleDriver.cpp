#include "kelo_tulip/BicycleDriver.h"
#include <iostream>

namespace kelo {

BicycleDriver::BicycleDriver(std::string device, std::vector<kelo::WheelConfig>* configs, int nWheels)
    : device(device), wheelConfigs(configs), nWheels(nWheels), stopThread(false), 
      target_vL(0.0f), target_vR(0.0f), ethercatThread(NULL) {
    
    // Initialize SOEM pointers (Absolute must for stability)
    ecx_context.port = &ecx_port;
    ecx_context.slavelist = &ecx_slave[0];
    ecx_context.slavecount = &ecx_slavecount;
    ecx_context.maxslave = EC_MAXSLAVE;
    ecx_context.grouplist = &ec_group[0];
    ecx_context.maxgroup = EC_MAXGROUP;
    ecx_context.esibuf = &esibuf[0];
    ecx_context.esimap = &esimap[0];
    ecx_context.elist = &ec_elist;
    ecx_context.idxstack = &ec_idxstack;
    ecx_context.ecaterror = &EcatError;
    ecx_context.DCtime = &ec_DCtime;
    ecx_context.SMcommtype = &ec_SMcommtype;
    ecx_context.PDOassign = &ec_PDOassign;
    ecx_context.PDOdesc = &ec_PDOdesc;
    ecx_context.eepSM = &ec_SM;
    ecx_context.eepFMMU = &ec_FMMU;
    EcatError = FALSE;
}

bool BicycleDriver::initEthercat() {
    if (!ecx_init(&ecx_context, const_cast<char*>(device.c_str()))) return false;
    if (ecx_config_init(&ecx_context, TRUE) <= 0) return false;
    
    ecx_config_map_group(&ecx_context, IOmap, 0);
    
    // Request Operational State
    ecx_slave[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ecx_context, 0);
    ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

    if (ecx_slave[0].state != EC_STATE_OPERATIONAL) return false;

    ethercatThread = new boost::thread(boost::bind(&BicycleDriver::ethercatHandler, this));
    return true;
}

void BicycleDriver::ethercatHandler() {
    while (!stopThread) {
        ecx_receive_processdata(&ecx_context, 1000);

        for (int i = 0; i < nWheels; i++) {
            int slave = (*wheelConfigs)[i].ethercatNumber;
            txpdo1_t* tx_data = (txpdo1_t*) ecx_slave[slave].inputs;
            rxpdo1_t* rx_data = (rxpdo1_t*) ecx_slave[slave].outputs;

            // 1. Precise Timing handshake (Match original driver exactly)
            rx_data->timestamp = tx_data->sensor_ts + 100000; 

            // 2. Command: Bit 0 (Enable1), Bit 1 (Enable2), Bit 2 (Velocity Mode)
            // We use 7. If resistance is still weak, the hardware might need an 
            // additional 'Status' bit from rx_data->command2.
            rx_data->command1 = 7; 
            rx_data->command2 = 0;

            // 3. Torque/Current Limits - Setting these to 20.0f provides the "Lock"
            rx_data->limit1_p = 20.0f; rx_data->limit1_n = -20.0f;
            rx_data->limit2_p = 20.0f; rx_data->limit2_n = -20.0f;

            // 4. THE SIGN TEST
            // If the motors are fighting (weak resistance), one needs to be flipped.
            // In Kelo Hubs, the mechanical mirroring requires one setpoint to be inverted.
            rx_data->setpoint1 = target_vL;
            rx_data->setpoint2 = -target_vR; 
        }

        ecx_send_processdata(&ecx_context);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
}

txpdo1_t* BicycleDriver::getRawSensorData(int wheel_idx) {
    int slave = (*wheelConfigs)[wheel_idx].ethercatNumber;
    return (txpdo1_t*) ecx_slave[slave].inputs;
}

void BicycleDriver::closeEthercat() {
    stopThread = true;
    if (ethercatThread) {
        ethercatThread->join();
        delete ethercatThread;
    }
    ecx_close(&ecx_context);
}

BicycleDriver::~BicycleDriver() { closeEthercat(); }

}