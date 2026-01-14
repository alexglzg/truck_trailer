#ifndef BICYCLE_DRIVER_H
#define BICYCLE_DRIVER_H

#include <string>
#include <vector>
#include <fstream>
#include <boost/thread.hpp>

extern "C" {
#include "kelo_tulip/soem/ethercattype.h"
#include "kelo_tulip/EtherCATModule.h"
#include "kelo_tulip/KeloDriveAPI.h"
#include "nicdrv.h"
#include "kelo_tulip/soem/ethercatbase.h"
#include "kelo_tulip/soem/ethercatmain.h"
#include "kelo_tulip/soem/ethercatconfig.h"
#include "kelo_tulip/soem/ethercatcoe.h"
#include "kelo_tulip/soem/ethercatdc.h"
#include "kelo_tulip/soem/ethercatprint.h"
}

#include "kelo_tulip/Utils.h"
#include "kelo_tulip/WheelConfig.h"

namespace kelo {

class BicycleDriver {
public:
    BicycleDriver(std::string device, std::vector<kelo::WheelConfig>* configs, int nWheels);
    virtual ~BicycleDriver();

    bool initEthercat();
    void closeEthercat();

    txpdo1_t* getRawSensorData(int wheel_idx);
    void sendRawCommand(int wheel_idx, rxpdo1_t* command);

private:
    void ethercatHandler(); 

    std::string device;
    std::vector<kelo::WheelConfig>* wheelConfigs;
    int nWheels;
    
    volatile bool stopThread;
    boost::thread* ethercatThread;

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
    ecx_portt ecx_port;
    ecx_redportt ecx_redport;
    ecx_contextt ecx_context;
    char IOmap[4096];
};

} // namespace kelo
#endif