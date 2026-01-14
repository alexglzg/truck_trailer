#include "kelo_tulip/BicycleDriver.h"
#include <iostream>

namespace kelo {

BicycleDriver::BicycleDriver(std::string device, std::vector<kelo::WheelConfig>* configs, int nWheels)
    : device(device), wheelConfigs(configs), nWheels(nWheels), stopThread(false), 
      target_vL(0.0f), target_vR(0.0f), ethercatThread(NULL) {
    
    // FULL POINTER SETUP
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
}

bool BicycleDriver::initEthercat() {
    if (!ecx_init(&ecx_context, const_cast<char*>(device.c_str()))) return false;
    
    // Find Slaves and Move to PRE-OP
    if (ecx_config_init(&ecx_context, TRUE) <= 0) return false;

    // --- ANALOGOUS TO ORIGINAL: SLAVE CONFIGURATION ---
    for (int i = 1; i <= ecx_slavecount; i++) {
        // Set Sync Managers for Process Data
        ecx_slave[i].SM[2].StartAddr = 0x1600;
        ecx_slave[i].SM[3].StartAddr = 0x1a00;
    }

    ecx_config_map_group(&ecx_context, IOmap, 0);
    ecx_configdc(&ecx_context);

    // MOVE TO OPERATIONAL
    ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
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
            txpdo1_t* feedback = (txpdo1_t*) ecx_slave[slave].inputs;
            rxpdo1_t* command = (rxpdo1_t*) ecx_slave[slave].outputs;

            // 1. THE WATCHDOG HEARTBEAT
            // The +100*1000 offset is standard for the Kelo's firmware 
            // to prevent "Timestamp Error" and unlock full current.
            command->timestamp = feedback->sensor_ts + 100000; 

            // 2. THE STIFF COMMAND
            // Original: COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_VELOCITY
            command->command1 = 7; 
            command->command2 = 0;

            // 3. CURRENT LIMITS (The "Lock")
            // This is what creates the "Electrical Lock" resistance.
            command->limit1_p = 20.0f; command->limit1_n = -20.0f;
            command->limit2_p = 20.0f; command->limit2_n = -20.0f;

            // 4. ACTUATOR SETPOINTS
            // Mechanical inversion: Motor 1 (Positive), Motor 2 (Negative)
            command->setpoint1 = target_vL;
            command->setpoint2 = -target_vR; 
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
    if (ethercatThread) ethercatThread->join();
    ecx_slave[0].state = EC_STATE_INIT;
    ecx_writestate(&ecx_context, 0);
    ecx_close(&ecx_context);
}

BicycleDriver::~BicycleDriver() { closeEthercat(); }

}