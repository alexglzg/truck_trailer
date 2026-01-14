#include "kelo_tulip/BicycleDriver.h"
#include <iostream>

namespace kelo {

BicycleDriver::BicycleDriver(std::string device, std::vector<kelo::WheelConfig>* configs, int nWheels)
    : device(device), wheelConfigs(configs), nWheels(nWheels), stopThread(false), 
      target_vL(0.0f), target_vR(0.0f), ethercatThread(NULL) {
    
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
    
    // 1. Reset any existing errors on the slaves
    if (ecx_config_init(&ecx_context, TRUE) <= 0) return false;
    
    for (int i = 1; i <= ecx_slavecount; i++) {
        // Clear Error Bit
        ecx_slave[i].state = EC_STATE_INIT | EC_STATE_ACK;
        ecx_writestate(&ecx_context, i);

        // Map Sync Managers to Kelo-specific addresses
        ecx_slave[i].SM[2].StartAddr = 0x1600;
        ecx_slave[i].SM[2].SMflags = 0x00010024; // Output SM Flags
        ecx_slave[i].SM[3].StartAddr = 0x1a00;
        ecx_slave[i].SM[3].SMflags = 0x00010020; // Input SM Flags
    }

    ecx_config_map_group(&ecx_context, IOmap, 0);
    ecx_configdc(&ecx_context);

    // 2. Cyclic State Transition to SAFE_OP
    ecx_slave[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ecx_context, 0);
    int chk = 100;
    do {
        ecx_send_processdata(&ecx_context);
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
        ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, 500);
    } while (chk-- && (ecx_slave[0].state != EC_STATE_SAFE_OP));

    if (ecx_slave[0].state != EC_STATE_SAFE_OP) {
        std::cerr << "Hardware rejected SAFE_OP. Code: " << (int)ecx_slave[0].state << std::endl;
        return false;
    }

    // 3. Cyclic State Transition to OPERATIONAL
    ecx_slave[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ecx_context, 0);
    chk = 100;
    do {
        ecx_send_processdata(&ecx_context);
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
        ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, 500);
    } while (chk-- && (ecx_slave[0].state != EC_STATE_OPERATIONAL));

    if (ecx_slave[0].state != EC_STATE_OPERATIONAL) {
        std::cerr << "Hardware rejected OP. Code: " << (int)ecx_slave[0].state << std::endl;
        return false;
    }

    std::cout << "EtherCAT Handshake Complete. Motors Armed." << std::endl;
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

            // Handshake synchronization
            command->timestamp = feedback->sensor_ts + 100000; 
            command->command1 = 7; // ENABLE1 | ENABLE2 | MODE_VELOCITY
            command->command2 = 0;
            command->limit1_p = 25.0f; command->limit1_n = -25.0f;
            command->limit2_p = 25.0f; command->limit2_n = -25.0f;

            // Direct Velocity Mapping
            command->setpoint1 = target_vL;
            command->setpoint2 = -target_vR; // Mechanical sign flip
        }

        ecx_send_processdata(&ecx_context);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
}

txpdo1_t* BicycleDriver::getRawSensorData(int wheel_idx) {
    if (wheel_idx >= nWheels) return NULL;
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