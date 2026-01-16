#include "kelo_tulip/AngleDriver.h"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace kelo {

AngleDriver::AngleDriver(std::string device, std::vector<WheelConfig>* configs, 
                         std::vector<WheelData>* data, int nWheels)
    : wheelConfigs(configs), wheelData(data), nWheels(nWheels), device(device) {
    
    state = DRIVER_STATE_INIT;
    canChangeActive = false;
    ethercatInitialized = false;
    stopThread = false;
    target_linear_v = 0.0f; 
    target_phi = 0.0f;
    ramped_vL = 0.0f;
    ramped_vR = 0.0f;
    ramped_phi = 0.0f;
    last_error = 0.0f;

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

bool AngleDriver::initEthercat() {
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

    ethercatThread = new boost::thread(boost::bind(&AngleDriver::ethercatHandler, this));
    return true;
}

void AngleDriver::ethercatHandler() {
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

void AngleDriver::doControl() {
    int slave0 = (*wheelConfigs)[0].ethercatNumber;
    txpdo1_t* feedback0 = (txpdo1_t*)ecx_slave[slave0].inputs;

    // 1. Angle Target Ramping
    float phi_ramp_step = 0.002f; 
    if (target_phi > ramped_phi + phi_ramp_step) ramped_phi += phi_ramp_step;
    else if (target_phi < ramped_phi - phi_ramp_step) ramped_phi -= phi_ramp_step;
    else ramped_phi = target_phi;

    // 2. Pivot Position Control (PD Loop)
    float current_phi = atan2(sin(feedback0->encoder_pivot - (*wheelConfigs)[0].a), 
                              cos(feedback0->encoder_pivot - (*wheelConfigs)[0].a));
    
    float error = ramped_phi - current_phi;
    error = atan2(sin(error), cos(error)); 

    float d_error = error - last_error;
    last_error = error;

    // w_output is the differential speed needed
    float w_output = (Kp * error) + (Kd * d_error);

    // --- THE CRITICAL FIX ---
    // We flip the signs here to change from Positive Feedback to Negative Feedback.
    // If it was (+) for vL and (-) for vR, we change it to:
    float vL_target = (target_linear_v - (w_output * d_width / 2.0f)) / r_wheel;
    float vR_target = (target_linear_v + (w_output * d_width / 2.0f)) / r_wheel;
    // ------------------------

    // 4. Velocity Ramping
    float diffL = vL_target - ramped_vL;
    float diffR = vR_target - ramped_vR;

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
        
        out->setpoint1 = ramped_vL;
        out->setpoint2 = -ramped_vR;
    }
}

void AngleDriver::doStop() {
    ramped_vL = 0.0f;
    ramped_vR = 0.0f;
    for (unsigned int i = 0; i < nWheels; i++) {
        rxpdo1_t* out = (rxpdo1_t*) ecx_slave[(*wheelConfigs)[i].ethercatNumber].outputs;
        out->command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_VELOCITY;
        out->setpoint1 = 0; out->setpoint2 = 0;
    }
}

void AngleDriver::setCanChangeActive() { canChangeActive = true; }
txpdo1_t* AngleDriver::getProcessData(int slave) { return (txpdo1_t*) ecx_slave[slave].inputs; }

void AngleDriver::closeEthercat() {
    stopThread = true;
    if (ethercatThread) ethercatThread->join();
    ecx_slave[0].state = EC_STATE_INIT;
    ecx_writestate(&ecx_context, 0);
    ecx_close(&ecx_context);
}

AngleDriver::~AngleDriver() { closeEthercat(); }

} // namespace kelo