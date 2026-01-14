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
    // 1. Physical Layer Init
    if (!ecx_init(&ecx_context, const_cast<char*>(device.c_str()))) {
        std::cerr << "SOEM Init Failed on " << device << std::endl;
        return false;
    }

    // 2. Discover Slaves
    if (ecx_config_init(&ecx_context, TRUE) <= 0) {
        std::cerr << "No Slaves Found." << std::endl;
        return false;
    }

    // 3. Configure Sync Managers (The Mailboxes)
    // This maps the memory addresses so the hardware knows where to look for commands
    for (int i = 1; i <= ecx_slavecount; i++) {
        ecx_slave[i].SM[2].StartAddr = 0x1600;
        ecx_slave[i].SM[3].StartAddr = 0x1a00;
    }

    // 4. Map the I/O memory
    ecx_config_map_group(&ecx_context, IOmap, 0);

    // 5. Distributed Clock Setup (Kelo requires this for sync)
    ecx_configdc(&ecx_context);

    // 6. Transition to SAFE-OP first (Mandatory for most Kelo firmware)
    std::cout << "Transitioning to SAFE-OP..." << std::endl;
    ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    // 7. Transition to OPERATIONAL
    std::cout << "Requesting OPERATIONAL state..." << std::endl;
    ecx_slave[0].state = EC_STATE_OPERATIONAL;
    ecx_send_processdata(&ecx_context);
    ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
    ecx_writestate(&ecx_context, 0);

    // Check if it reached OP
    int chk = 40;
    do {
        ecx_send_processdata(&ecx_context);
        ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
        ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ecx_slave[0].state != EC_STATE_OPERATIONAL));

    if (ecx_slave[0].state != EC_STATE_OPERATIONAL) {
        std::cerr << "Slaves failed to reach OP state. Current state: " << ecx_slave[0].state << std::endl;
        // Print SOEM errors if any
        while(ecx_iserror(&ecx_context)) {
            std::cerr << "SOEM Error: " << ec_elist2string(&ecx_context) << std::endl;
        }
        return false;
    }

    std::cout << "EtherCAT Slaves Operational. Starting handler thread..." << std::endl;
    ethercatThread = new boost::thread(boost::bind(&BicycleDriver::ethercatHandler, this));
    return true;
}

void BicycleDriver::ethercatHandler() {
    while (!stopThread) {
        // We capture the Working Counter (WKC) to monitor connection health
        int wkc = ecx_receive_processdata(&ecx_context, 1000);
        
        for (int i = 0; i < nWheels; i++) {
            int slave = (*wheelConfigs)[i].ethercatNumber;
            txpdo1_t* feedback = (txpdo1_t*) ecx_slave[slave].inputs;
            rxpdo1_t* command = (rxpdo1_t*) ecx_slave[slave].outputs;

            // Handshake logic
            command->timestamp = feedback->sensor_ts + 100000; 
            command->command1 = 7; // Enable + Velocity
            command->command2 = 0;
            command->limit1_p = 20.0f; command->limit1_n = -20.0f;
            command->limit2_p = 20.0f; command->limit2_n = -20.0f;

            // Target Rad/s
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