
#ifndef RASTERDRIVER_APP_CONFIG_HPP
#define RASTERDRIVER_APP_CONFIG_HPP

#include "StepperMotor/StepperMotorBase.hpp"

#define TIME_GRID_BTN_LONG_PRESS        1000
#define TOTAL_DISTANCE_N_OF_STEPS       9000
#define STEPS_BEFORE_DECCEL             7300
#define EXPO_DISTANCE_STEPS             600
#define EXPO_REACH_STEPS                100
#define STEPS_AFTER_SWITCH              45

#define CONFIG1_SPEED                   4535
#define CONFIG2_SPEED                   7129
#define CONFIG1_ACCELERATION            32
#define CONFIG2_ACCELERATION            52

#define SERVICE_MOVE_ACCELERATION       4.0
#define SERVICE_MOVE_START_SPEED        500
#define INIT_MOVE_MAX_SPEED             2000
#define SERVICE_MOVE_MAX_SPEED          3000

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
            TOTAL_DISTANCE_N_OF_STEPS
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
    cfg.Vmin = SERVICE_MOVE_START_SPEED;
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
        INDICATION_0 = 0,
        INDICATION_1 = 1,
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
    };

    enum TIM_TASKS{
        NO_TASKS,
        FREEZE_SWITCH_TASK,
        IN_MOTION_SIG_DELAY_TASK
    };
}

#endif //RASTERDRIVER_APP_CONFIG_HPP