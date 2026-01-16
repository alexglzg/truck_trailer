#ifndef ANGLE_DRIVER_H_
#define ANGLE_DRIVER_H_

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
#include <boost/thread.hpp>
#include <string>
#include <vector>

namespace kelo {

struct WheelData {
    bool enable;
    bool error;
    bool errorTimestamp;
};

enum DriverState {
    DRIVER_STATE_UNDEFINED = 0x00,
    DRIVER_STATE_ACTIVE = 0x01,
    DRIVER_STATE_READY = 0x02,
    DRIVER_STATE_INIT = 0x04,
    DRIVER_STATE_ERROR = 0x10,
    DRIVER_STATE_TIMEOUT = 0x20
};

class AngleDriver {
public:
    AngleDriver(std::string device, std::vector<WheelConfig>* wheelConfigs, 
                  std::vector<WheelData>* wheelData, int nWheels);
    virtual ~AngleDriver();

    bool initEthercat();
    void closeEthercat();
    void setCanChangeActive();

    // volatile float target_vL;
    // volatile float target_vR;

    volatile float target_linear_v;
    volatile float target_phi;

    txpdo1_t* getProcessData(int slave);

protected:
    void ethercatHandler();
    void doControl();
    void doStop();

    // Original Kelo Constants for Pivot Control
    const float Kp = 15.0f;     // Proportional Gain
    const float Kd = 0.5f;      // Derivative Gain
    const float s_offset = 0.01f; // 1cm Caster Offset
    const float d_width = 0.0775f; 
    const float r_wheel = 0.0524f;

    // Internal state for ramping and PD
    float ramped_phi;
    float last_error;

    float ramped_vL;
    float ramped_vR;
    const float max_v_step = 0.025f; // Max change in velocity per 1ms frame

    volatile DriverState state;
    bool canChangeActive;

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
    std::string device;
    bool ethercatInitialized;
    boost::thread* ethercatThread;
    volatile bool stopThread;

    int nWheels;
    std::vector<WheelConfig>* wheelConfigs;
    std::vector<WheelData>* wheelData;

private:
    AngleDriver(const AngleDriver&);
};

} // namespace kelo

#endif