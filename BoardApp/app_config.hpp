
#ifndef RASTERDRIVER_APP_CONFIG_HPP
#define RASTERDRIVER_APP_CONFIG_HPP

#include "embedded_hw_utils/StepperMotor/accel_motor.hpp"

#define mStep                           16
#define mSTEPS(v)                       int((v) * mStep)

//#define TIME_GRID_BTN_LONG_PRESS      1000
#define TOTAL_RANGE_STEPS               mSTEPS(562)     //total steps before abnormal error stop (switch is missing)
#define STEPS_BEFORE_DECCEL             mSTEPS(456)     //steps before deceleration in max long rage move (from edge to edge)
#define EXPO_RANGE_STEPS                mSTEPS(18.75)   //steps in expo move before changing direction
#define EXPO_OFFSET_STEPS               mSTEPS(-9.375)  //steps to run inside park zone (to center expo move)
#define SWITCH_PRESS_STEPS              mSTEPS(3)       //steps to run inside park zone on end of move (ensure not to lose switch while bouncing etc.)
#define RUN_OUT_STEPS                   mSTEPS(15)      //steps running out of the parking zone and returning for correct parking

#define INITIAL_SPEED                   mSTEPS(31.25)   //start speed at every move
#define CONFIG1_MAX_SPEED               mSTEPS(120)     //max speed in acceleration moves
#define CONFIG2_MAX_SPEED               mSTEPS(150)
#define CONFIG1_RAMP_TIME               48'000          //time in uSec for acceleration phase (from start to max speed)
#define CONFIG2_RAMP_TIME               38'000
#define CONFIG1_ACCELERATION            mSTEPS(2.5)     //acceleration only for kParabolic
#define CONFIG2_ACCELERATION            mSTEPS(2.0)     //acceleration only for kParabolic

#define SERVICE_MOVE_MAX_SPEED          mSTEPS(150)
#define INIT_MOVE_MAX_SPEED             mSTEPS(90)

#define ACCEL_TYPE                      \
                                      MotorSpecial::AccelType::kParabolic
//                                      MotorSpecial::AccelType::kLinear
//                                      MotorSpecial::AccelType::kConstantPower
//                                      MotorSpecial::AccelType::kSigmoid

#define IN_MOTION_uSec_DELAY            1'000           //time gap in uSec before accel phase end (to set out IN_MOTION sig)
#define IN_MOTION_mSec_DELAY            (IN_MOTION_uSec_DELAY / 1000)

//static void FreezeDeviceDelay(uint32_t delay){
//    uint16_t msDelay = delay * 10 > UINT16_MAX ? UINT16_MAX : delay * 10;
//    __disable_irq ();
//    TIM6->CNT = 0;
//    while(TIM6->CNT < msDelay){}
//    __enable_irq ();
//}

struct AppCfg{
    MotorSpecial::AccelCfg accelCfg;
    bool shake_scan_enabled {false};
    bool direction_inverted {false};
};

static AppCfg getBaseConfig(){
    static StepperMotor::StepperCfg s_cfg{
            PIN<PinWriteable>{STEP_GPIO_Port, STEP_Pin},
            PIN<PinWriteable>{DIR_GPIO_Port, DIR_Pin},
            PIN<PinWriteable>{ENABLE_GPIO_Port, ENABLE_Pin},
            &htim4,
            TIM_CHANNEL_2,
            TOTAL_RANGE_STEPS
    };
    static MotorSpecial::AccelCfg a_cfg{
            CONFIG1_RAMP_TIME,
            ACCEL_TYPE,
            CONFIG1_ACCELERATION,
            CONFIG1_MAX_SPEED,
            INITIAL_SPEED,
            s_cfg
    };
    static AppCfg appCfg {a_cfg};
    return appCfg;
}

static AppCfg& getDIPConfig(){
    static auto cfg = getBaseConfig();
    if(HAL_GPIO_ReadPin(CONFIG_1_GPIO_Port, CONFIG_1_Pin)){
        cfg.accelCfg.Vmax = CONFIG1_MAX_SPEED;
        cfg.accelCfg.A = CONFIG1_ACCELERATION;
        cfg.accelCfg.ramp_time = CONFIG1_RAMP_TIME;
    }else{
        cfg.accelCfg.Vmax = CONFIG2_MAX_SPEED;
        cfg.accelCfg.A = CONFIG2_ACCELERATION;
        cfg.accelCfg.ramp_time = CONFIG2_RAMP_TIME;
    }
    cfg.accelCfg.Vmin = INITIAL_SPEED;
    cfg.shake_scan_enabled = HAL_GPIO_ReadPin(CONFIG_2_GPIO_Port, CONFIG_2_Pin);

    HAL_GPIO_ReadPin(CONFIG_3_GPIO_Port, CONFIG_3_Pin) ? cfg.accelCfg.accel_type = MotorSpecial::AccelType::kParabolic
                                                            : cfg.accelCfg.accel_type = MotorSpecial::AccelType::kConstantPower;

    return cfg;
}

static void StartTaskTimIT(uint16_t delay){
    uint16_t msDelay = delay > UINT16_MAX ? UINT16_MAX : delay;
    __HAL_TIM_SET_AUTORELOAD(&htim6, msDelay);
    HAL_TIM_Base_Start_IT(&htim6);
}

namespace RB::types{

    enum OUTPUT_TYPE{
        INDICATION_0 = 1,
        INDICATION_1 = 0,
        IN_MOTION = 2,
    };

    enum INPUT_TYPE{
        EXP_REQ = 0,
        GRID_HOME_DETECT = 1,
        GRID_INFIELD_DETECT = 2,
    };

    enum BOARD_STATUS_ERROR{
        NO_ERROR,
        LIMIT_SWITCH_ERROR,
        STANDBY_MOVEMENT_ERROR,
        EXP_REQ_ERROR
    };

    enum BOARD_STATUS{
        DEVICE_INIT_STATE,
        DEVICE_SERVICE_MOVING,
        DEVICE_GRID_IN_FIELD,
        DEVICE_GRID_HOME,
        DEVICE_SCANNING,
        DEVICE_SHAKE_SCANNING,
        DEVICE_ERROR,
        DEVICE_POSITION,
        DEVICE_MOVE_IN_FIELD,
        DEVICE_MOVE_HOME
    };

    enum TIM_TASKS{
        NO_TASKS,
        FREEZE_SWITCH_TASK,
        IN_MOTION_SIG_DELAY_TASK,
        SHAKE_EXPO_DELAY_TASK
    };
}

#endif //RASTERDRIVER_APP_CONFIG_HPP