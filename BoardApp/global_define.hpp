
#ifndef RASTERDRIVER_GLOBAL_DEFINE_HPP
#define RASTERDRIVER_GLOBAL_DEFINE_HPP

#define TIME_GRID_BTN_LONG_PRESS        1000
#define TOTAL_DISTANCE_N_OF_STEPS       12000
#define EXPO_DISTANCE_N_OF_STEPS        3000
#define EXPO_REACH_DISTANCE_N_OF_STEPS  500
#define LOAD_UNLOAD_SPEED               1985
#define START_SPEED                     1569

#define CONFIG1_SPEED                   4535
#define CONFIG2_SPEED                   3969
#define CONFIG3_SPEED                   5003
#define CONFIG4_SPEED                   7129
#define CONFIG1_ACCELERATION            40
#define CONFIG2_ACCELERATION            32
#define CONFIG3_ACCELERATION            43
#define CONFIG4_ACCELERATION            52

//static void FreezeDeviceDelay(uint32_t delay){
//    uint16_t msDelay = delay * 10 > UINT16_MAX ? UINT16_MAX : delay * 10;
//    __disable_irq ();
//    TIM6->CNT = 0;
//    while(TIM6->CNT < msDelay){}
//    __enable_irq ();
//}

StepperMotor::StepperCfg DIPSwitches_configureDriver(){
    StepperMotor::StepperCfg cfg{};
    cfg.Vmin = LOAD_UNLOAD_SPEED;
    cfg.htim = &htim4;
    cfg.timChannel = TIM_CHANNEL_2;
    cfg.reach_steps = EXPO_REACH_DISTANCE_N_OF_STEPS;
    cfg.criticalNofSteps = TOTAL_DISTANCE_N_OF_STEPS;
    cfg.expo_distance_steps = EXPO_DISTANCE_N_OF_STEPS;
    if(HAL_GPIO_ReadPin(CONFIG_1_GPIO_Port, CONFIG_1_Pin)){
        if(HAL_GPIO_ReadPin(CONFIG_2_GPIO_Port, CONFIG_2_Pin)){
            cfg.Vmax = CONFIG1_SPEED;
            cfg.A = CONFIG1_ACCELERATION;
        }else{
            cfg.Vmax = CONFIG2_SPEED;
            cfg.A = CONFIG2_ACCELERATION;
        }
    }else{
        if(HAL_GPIO_ReadPin(CONFIG_2_GPIO_Port, CONFIG_2_Pin)){
            cfg.Vmax = CONFIG3_SPEED;
            cfg.A = CONFIG3_ACCELERATION;
        }else{
            cfg.Vmax = CONFIG4_SPEED;
            cfg.A = CONFIG4_ACCELERATION;
        }
    }

    if(!HAL_GPIO_ReadPin(CONFIG_3_GPIO_Port, CONFIG_3_Pin)){
        cfg.directionInverted = true;
    }
    return cfg;
}

static void StartFreezeTimIT(uint32_t delay){
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

    enum BTN_TYPE{
        GRID_BUTTON,
    };
}

#endif //RASTERDRIVER_GLOBAL_DEFINE_HPP