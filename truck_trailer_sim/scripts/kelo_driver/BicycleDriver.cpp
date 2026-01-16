#include "kelo_tulip/BicycleDriver.h"
#include <iostream>
#include <algorithm>

namespace kelo {

BicycleDriver::BicycleDriver(std::string device, std::vector<WheelConfig>* configs, 
                             std::vector<WheelData>* data, int nWheels)
    : wheelConfigs(configs), wheelData(data), nWheels(nWheels), device(device) {
    
    state = DRIVER_STATE_INIT;
    canChangeActive = false;
    ethercatInitialized = false;
    stopThread = false;
    target_vL = 0.0f; target_vR = 0.0f;

    // Initialize ramped values to zero
    ramped_vL = 0.0f;
    ramped_vR = 0.0f;

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
}

bool BicycleDriver::initEthercat() {
    if (!ecx_init(&ecx_context, const_cast<char*>(device.c_str()))) return false;
    
    int wkc = ecx_config_init(&ecx_context, TRUE);
    if (!wkc) return false;

    ecx_config_map_group(&ecx_context, IOmap, 0);

    for (unsigned int i = 0; i < nWheels; i++) {
        int slave = (*wheelConfigs)[i].ethercatNumber;
        if (ecx_slave[slave].eep_id != 24137745 && ecx_slave[slave].eep_id != 0 && 
            ecx_slave[slave].eep_id != 0x17010091 && ecx_slave[slave].eep_id != 0x02001001) {
            std::cout << "Wrong ID on slave " << slave << std::endl;
            return false;
        }
    }

    ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    if (ecx_slave[0].state != EC_STATE_SAFE_OP) return false;

    rxpdo1_t zeroData = {0};
    zeroData.timestamp = 1;
    for (unsigned int i = 0; i < nWheels; i++) {
        rxpdo1_t* ecData = (rxpdo1_t*) ecx_slave[(*wheelConfigs)[i].ethercatNumber].outputs;
        *ecData = zeroData;
    }
    ecx_send_processdata(&ecx_context);

    ecx_slave[0].state = EC_STATE_OPERATIONAL;
    ecx_send_processdata(&ecx_context);
    ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
    ecx_writestate(&ecx_context, 0);

    ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    if (ecx_slave[0].state != EC_STATE_OPERATIONAL) return false;

    ethercatThread = new boost::thread(boost::bind(&BicycleDriver::ethercatHandler, this));
    return true;
}

void BicycleDriver::ethercatHandler() {
    state = DRIVER_STATE_READY;
    while (!stopThread) {
        ecx_receive_processdata(&ecx_context, 1000);
        
        if (canChangeActive && state == DRIVER_STATE_READY) state = DRIVER_STATE_ACTIVE;

        if (state == DRIVER_STATE_ACTIVE) doControl();
        else doStop();

        ecx_send_processdata(&ecx_context);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
}

void BicycleDriver::doControl() {
    // FIX: Access sensor_ts field directly to avoid pointer-to-int cast error
    int slave0 = (*wheelConfigs)[0].ethercatNumber;
    txpdo1_t* feedback0 = (txpdo1_t*)ecx_slave[slave0].inputs;
    uint32 current_ts = feedback0->sensor_ts;

    float diffL = target_vL - ramped_vL;
    float diffR = target_vR - ramped_vR;

    if (diffL > max_v_step) diffL = max_v_step;
    if (diffL < -max_v_step) diffL = -max_v_step;
    if (diffR > max_v_step) diffR = max_v_step;
    if (diffR < -max_v_step) diffR = -max_v_step;

    ramped_vL += diffL;
    ramped_vR += diffR;

    for (unsigned int i = 0; i < nWheels; i++) {
        int slave = (*wheelConfigs)[i].ethercatNumber;
        txpdo1_t* in = (txpdo1_t*) ecx_slave[slave].inputs;
        rxpdo1_t* out = (rxpdo1_t*) ecx_slave[slave].outputs;

        out->timestamp = in->sensor_ts + 100 * 1000;
        out->command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_VELOCITY;
        out->limit1_p = 20.0f; out->limit1_n = -20.0f;
        out->limit2_p = 20.0f; out->limit2_n = -20.0f;
        // out->setpoint1 = target_vL;
        // out->setpoint2 = -target_vR;
        out->setpoint1 = ramped_vL;
        out->setpoint2 = -ramped_vR;
    }
}

void BicycleDriver::doStop() {
    ramped_vL = 0.0f;
    ramped_vR = 0.0f;

    for (unsigned int i = 0; i < nWheels; i++) {
        rxpdo1_t* out = (rxpdo1_t*) ecx_slave[(*wheelConfigs)[i].ethercatNumber].outputs;
        out->command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_VELOCITY;
        out->setpoint1 = 0; out->setpoint2 = 0;
    }
}

void BicycleDriver::setCanChangeActive() { canChangeActive = true; }

txpdo1_t* BicycleDriver::getProcessData(int slave) { return (txpdo1_t*) ecx_slave[slave].inputs; }

void BicycleDriver::closeEthercat() {
    stopThread = true;
    if (ethercatThread) ethercatThread->join();
    ecx_slave[0].state = EC_STATE_INIT;
    ecx_writestate(&ecx_context, 0);
    ecx_close(&ecx_context);
}

BicycleDriver::~BicycleDriver() { closeEthercat(); }

}