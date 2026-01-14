#include "kelo_tulip/BicycleDriver.h"
#include <iostream>

namespace kelo {

BicycleDriver::BicycleDriver(std::string device, std::vector<kelo::WheelConfig>* configs, int nWheels)
    : device(device), wheelConfigs(configs), nWheels(nWheels), stopThread(false), 
      target_vL(0.0f), target_vR(0.0f), ethercatThread(NULL) {
    
    // Standard SOEM pointer initialization
    ecx_context.port = &ecx_port;
    ecx_context.slavelist = &ecx_slave[0];
    ecx_context.slavecount = &ecx_slavecount;
    ecx_context.maxslave = EC_MAXSLAVE;
    ecx_context.grouplist = &ec_group[0];
    ecx_context.maxgroup = EC_MAXGROUP;
    ecx_context.esibuf = &esibuf[0];
    ecx_context.esimap = &esimap[0];
    ecx_context.esislave = 0;
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

    // 1. SET MAILBOX / SYNC MANAGERS
    for (int i = 1; i <= ecx_slavecount; i++) {
        ecx_slave[i].SM[2].StartAddr = 0x1600;
        ecx_slave[i].SM[3].StartAddr = 0x1a00;
        
        // --- SDO CONFIGURATION BLOCK (The "Handshake") ---
        // This tells the motors to enter high-torque velocity mode (Mode 3)
        // and sets the internal limits.
        int8 mode = 3; // Profile Velocity Mode
        ecx_SDOwrite(&ecx_context, i, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);
        
        float max_current = 20.0f;
        ecx_SDOwrite(&ecx_context, i, 0x6073, 0x00, FALSE, sizeof(max_current), &max_current, EC_TIMEOUTRXM);
    }

    ecx_config_map_group(&ecx_context, IOmap, 0);
    ecx_configdc(&ecx_context);

    // 2. TRANSITION TO SAFE_OP (Check for failures here)
    ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    if (ecx_slave[0].state != EC_STATE_SAFE_OP && ecx_slave[0].state != EC_STATE_OPERATIONAL) {
        std::cerr << "Failed to reach SAFE_OP. Error: " << ec_elist2string() << std::endl;
        return false;
    }

    // 3. TRANSITION TO OPERATIONAL (OP)
    ecx_slave[0].state = EC_STATE_OPERATIONAL;
    ecx_send_processdata(&ecx_context);
    ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
    ecx_writestate(&ecx_context, 0);

    // Poll for OP state (Must send process data during transition)
    int chk = 100;
    do {
        ecx_send_processdata(&ecx_context);
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
        ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ecx_slave[0].state != EC_STATE_OPERATIONAL));

    if (ecx_slave[0].state != EC_STATE_OPERATIONAL) {
        std::cerr << "OP state timeout. Slave 1 state: " << ecx_slave[1].state << std::endl;
        return false;
    }

    std::cout << "Kelo Hub Operational & Locked." << std::endl;
    ethercatThread = new boost::thread(boost::bind(&BicycleDriver::ethercatHandler, this));
    return true;
}

void BicycleDriver::ethercatHandler() {
    while (!stopThread) {
        ecx_receive_processdata(&ecx_context, 1000);

        for (int i = 0; i < nWheels; i++) {
            int slave = (*wheelConfigs)[i].ethercatNumber;
            txpdo1_t* tx = (txpdo1_t*) ecx_slave[slave].inputs;
            rxpdo1_t* rx = (rxpdo1_t*) ecx_slave[slave].outputs;

            // Strict timestamp offset (Original driver uses 100*1000)
            rx->timestamp = tx->sensor_ts + 100000; 

            // Enable Sequence (7 = Enable1 + Enable2 + VelocityMode)
            rx->command1 = 7; 
            rx->command2 = 0;
            rx->limit1_p = 20.0f; rx->limit1_n = -20.0f;
            rx->limit2_p = 20.0f; rx->limit2_n = -20.0f;

            rx->setpoint1 = target_vL;
            rx->setpoint2 = -target_vR; // Mirrored sign
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
    if (ethercatThread) {
        ethercatThread->join();
        delete ethercatThread;
    }
    ecx_slave[0].state = EC_STATE_INIT;
    ecx_writestate(&ecx_context, 0);
    ecx_close(&ecx_context);
}

BicycleDriver::~BicycleDriver() { closeEthercat(); }

} // namespace kelo