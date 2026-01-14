#ifndef BICYCLE_DRIVER_H
#define BICYCLE_DRIVER_H

#include <string>
#include <vector>
#include <boost/thread.hpp>

extern "C" {
#include "kelo_tulip/soem/ethercat.h"
#include "kelo_tulip/EtherCATModule.h"
#include "kelo_tulip/KeloDriveAPI.h"
}
#include "kelo_tulip/WheelConfig.h"

namespace kelo {

class BicycleDriver {
public:
    BicycleDriver(std::string device, std::vector<kelo::WheelConfig>* configs, int nWheels);
    virtual ~BicycleDriver();

    bool initEthercat();
    void closeEthercat();

    // Direct Actuator Targets
    volatile float target_vL;
    volatile float target_vR;

    txpdo1_t* getRawSensorData(int wheel_idx);

private:
    void ethercatHandler(); 
    
    std::string device;
    std::vector<kelo::WheelConfig>* wheelConfigs;
    int nWheels;
    
    volatile bool stopThread;
    boost::thread* ethercatThread;

    // --- FULL SOEM STACK ---
    ecx_contextt ecx_context;
    ecx_portt ecx_port;
    ec_slavet ecx_slave[EC_MAXSLAVE];
    int ecx_slavecount;
    ec_groupt ec_group[EC_MAXGROUP];
    uint8 esibuf[EC_MAXEEPBUF];
    uint32 esimap[EC_MAXEEPBITMAP];
    ec_eringt ec_elist;
    ec_idxstackT ec_idxstack;
    ec_SMcommtypet ec_SMcommtype;
    ec_PDOassignt ec_PDOassign;
    ec_PDOdesct ec_PDOdesc;
    ec_eepromSMt ec_SM;
    ec_eepromFMMUt ec_FMMU;
    boolean EcatError;
    int64 ec_DCtime;
    char IOmap[4096];
};

} 
#endif