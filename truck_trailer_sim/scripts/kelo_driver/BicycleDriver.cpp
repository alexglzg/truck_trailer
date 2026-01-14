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

bool BicycleDriver::setupSDOs(int slave) {
    int retval = 0;
    int8_t mode = 3; // Profile Velocity Mode
    float max_curr = 25.0f;
    
    // Write Mode of Operation
    retval += ecx_SDOwrite(&ecx_context, slave, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);
    // Write Max Current (Torque Limit)
    retval += ecx_SDOwrite(&ecx_context, slave, 0x6073, 0x00, FALSE, sizeof(max_curr), &max_curr, EC_TIMEOUTRXM);
    
    return (retval > 0);
}

bool BicycleDriver::initEthercat() {
    if (!ecx_init(&ecx_context, const_cast<char*>(device.c_str()))) return false;
    if (ecx_config_init(&ecx_context, TRUE) <= 0) return false;

    for (int i = 1; i <= ecx_slavecount; i++) {
        // Clear errors and force INIT
        ecx_slave[i].state = EC_STATE_INIT | EC_STATE_ACK;
        ecx_writestate(&ecx_context, i);

        // Configure Mailboxes (Sync Managers)
        ecx_slave[i].SM[2].StartAddr = 0x1600;
        ecx_slave[i].SM[2].SMflags = 0x00010024;
        ecx_slave[i].SM[3].StartAddr = 0x1a00;
        ecx_slave[i].SM[3].SMflags = 0x00010020;

        // Apply SDO configuration while in PRE-OP
        setupSDOs(i);
    }

    ecx_config_map_group(&ecx_context, IOmap, 0);
    ecx_configdc(&ecx_context);

    // Transition Loop: PRE-OP -> SAFE-OP -> OPERATIONAL
    ecx_slave[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ecx_context, 0);
    
    int chk = 200;
    do {
        ecx_send_processdata(&ecx_context);
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
        ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, 500);
    } while (chk-- && (ecx_slave[0].state != EC_STATE_SAFE_OP));

    if (ecx_slave[0].state != EC_STATE_SAFE_OP) return false;

    ecx_slave[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ecx_context, 0);
    chk = 200;
    do {
        ecx_send_processdata(&ecx_context);
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
        ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, 500);
    } while (chk-- && (ecx_slave[0].state != EC_STATE_OPERATIONAL));

    if (ecx_slave[0].state != EC_STATE_OPERATIONAL) return false;

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

            // Handshake synchronization (Watchdog)
            rx->timestamp = tx->sensor_ts + 100000; 
            rx->command1 = 7; // Enable Motor 1 & 2 + Velocity Mode
            rx->command2 = 0;
            rx->limit1_p = 25.0f; rx->limit1_n = -25.0f;
            rx->limit2_p = 25.0f; rx->limit2_n = -25.0f;

            rx->setpoint1 = target_vL;
            rx->setpoint2 = -target_vR; // Mechanical Mirroring Inversion
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