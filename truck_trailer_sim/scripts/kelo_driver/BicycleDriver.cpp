#include "kelo_tulip/BicycleDriver.h"

namespace kelo {

BicycleDriver::BicycleDriver(std::string device, std::vector<kelo::WheelConfig>* configs, int nWheels)
    : device(device), wheelConfigs(configs), nWheels(nWheels), stopThread(false), 
      target_vL(0.0f), target_vR(0.0f) {
    
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
    if (ecx_config_init(&ecx_context, TRUE) <= 0) return false;
    ecx_config_map_group(&ecx_context, IOmap, 0);
    
    ecx_slave[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ecx_context, 0);
    ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

    ethercatThread = new boost::thread(boost::bind(&BicycleDriver::ethercatHandler, this));
    return true;
}

void BicycleDriver::ethercatHandler() {
    while (!stopThread) {
        ecx_receive_processdata(&ecx_context, 1000);

        // --- THE INJECTION ---
        for (int i = 0; i < nWheels; i++) {
            int slave = (*wheelConfigs)[i].ethercatNumber;
            txpdo1_t* rx = (txpdo1_t*) ecx_slave[slave].inputs;
            rxpdo1_t* tx = (rxpdo1_t*) ecx_slave[slave].outputs;

            // 1. Sync timestamp perfectly with what we just received
            tx->timestamp = rx->sensor_ts + 1000; 
            
            // 2. Force Enable + Velocity Mode (The "Lock")
            tx->command1 = 7; 
            tx->limit1_p = 20.0; tx->limit1_n = -20.0;
            tx->limit2_p = 20.0; tx->limit2_n = -20.0;

            // 3. Apply the targets set by the ROS Node
            tx->setpoint1 = target_vL;
            tx->setpoint2 = -target_vR;
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
    ecx_close(&ecx_context);
}

BicycleDriver::~BicycleDriver() { closeEthercat(); }

}