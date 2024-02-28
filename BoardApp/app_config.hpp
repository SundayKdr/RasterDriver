
#ifndef RASTERDRIVER_APP_CONFIG_HPP
#define RASTERDRIVER_APP_CONFIG_HPP

#include "StepperMotor/stepper_motor_base.hpp"

#define mStep                           16
#define mSTEPS(v)                       int((v) * mStep)

//#define TIME_GRID_BTN_LONG_PRESS      1000
#define TOTAL_RANGE_STEPS               mSTEPS(562)
#define STEPS_BEFORE_DECCEL             mSTEPS(456)
#define EXPO_RANGE_STEPS                mSTEPS(18.75)
#define EXPO_OFFSET_STEPS               mSTEPS(-9.375)
#define SWITCH_PRESS_STEPS              mSTEPS(3)
#define RUN_OUT_STEPS                   mSTEPS(15)

#define CONFIG1_SPEED                   mSTEPS(120)
#define CONFIG2_SPEED                   mSTEPS(150)
#define CONFIG1_ACCELERATION            mSTEPS(2.5)
#define CONFIG2_ACCELERATION            mSTEPS(2.0)

#define SERVICE_MOVE_ACCELERATION       mSTEPS(0.25)
#define INITIAL_SPEED                   mSTEPS(31.25)
#define SERVICE_MOVE_MAX_SPEED          mSTEPS(150)
#define INIT_MOVE_MAX_SPEED             mSTEPS(90)

#define IN_MOTION_DELAY                 150

//static void FreezeDeviceDelay(uint32_t delay){
//    uint16_t msDelay = delay * 10 > UINT16_MAX ? UINT16_MAX : delay * 10;
//    __disable_irq ();
//    TIM6->CNT = 0;
//    while(TIM6->CNT < msDelay){}
//    __enable_irq ();
//}

static StepperMotor::StepperCfg& getBaseConfig(){
    static StepperMotor::StepperCfg cfg{
            PIN<PinWriteable>{STEP_GPIO_Port, STEP_Pin},
            PIN<PinWriteable>{DIR_GPIO_Port, DIR_Pin},
            PIN<PinWriteable>{ENABLE_GPIO_Port, ENABLE_Pin},
            PIN<PinWriteable>{CURRENT_WIND_GPIO_Port, CURRENT_WIND_Pin},
            &htim4,
            TIM_CHANNEL_2,
            TOTAL_RANGE_STEPS
    };
    return cfg;
}

static StepperMotor::StepperCfg& getDIPConfig(){
    auto& cfg = getBaseConfig();
    if(HAL_GPIO_ReadPin(CONFIG_1_GPIO_Port, CONFIG_1_Pin)){
        cfg.Vmax = CONFIG1_SPEED;
        cfg.A = CONFIG1_ACCELERATION;
    }else{
        cfg.Vmax = CONFIG2_SPEED;
        cfg.A = CONFIG2_ACCELERATION;
    }
    cfg.Vmin = INITIAL_SPEED;
    cfg.shake_scan_enabled_ = HAL_GPIO_ReadPin(CONFIG_2_GPIO_Port, CONFIG_2_Pin);
    cfg.directionInverted = HAL_GPIO_ReadPin(CONFIG_3_GPIO_Port, CONFIG_3_Pin);
    return cfg;
}

static void StartTaskTimIT(uint16_t delay){
    uint16_t msDelay = delay * 10 > UINT16_MAX ? UINT16_MAX : delay * 10;
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