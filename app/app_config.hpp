#pragma once

#include "embedded_hw_utils/motors/stepper_motor/accel_motor.hpp"
#include "embedded_hw_utils/motors/stepper_motor/steps_converter.hpp"

//#define TIME_GRID_BTN_LONG_PRESS          1000
#define $MotorTimPsc                        (160)
#define $DriverMicroStep                    float(16)

#define TOTAL_RANGE_STEPS                   $mSTEPS(562)     //total steps before abnormal error stop (switch is missing)
#define STEPS_BEFORE_DECCEL                 $mSTEPS(475)     //steps before deceleration in max long rage move (from edge to edge)
#define EXPO_RANGE_STEPS                    $mSTEPS(18.75)   //steps in expo move before changing direction
#define EXPO_OFFSET_STEPS                   $mSTEPS(9.375)   //steps to run inside park zone (to center expo move)
#define SWITCH_PRESS_STEPS                  $mSTEPS(3)       //steps to run inside park zone on end of move (ensure not to lose switch while bouncing etc.)
#define RUN_OUT_STEPS                       $mSTEPS(15)      //steps running out of the parking zone and returning for correct parking

#define INITIAL_SPEED                       $mSTEPS(31.25)   //start speed at every move
#define CONFIG1_MAX_SPEED                   $mSTEPS(130)     //max speed in acceleration moves
#define CONFIG2_MAX_SPEED                   $mSTEPS(150)
#define CONFIG1_RAMP_TIME                   $rampT_(4)      //optimal acceleration phase width multiplier
#define CONFIG2_RAMP_TIME                   $rampT_(4)      //optimal acceleration phase width multiplier
#define CONFIG1_ACCELERATION                $mSTEPS(2.5)     //acceleration only for kParabolic
#define CONFIG2_ACCELERATION                $mSTEPS(3.5)     //acceleration only for kParabolic

#define SERVICE_MOVE_MAX_SPEED              $mSTEPS(150)
#define INIT_MOVE_MAX_SPEED                 $mSTEPS(90)

#define ACCEL_TYPE                      \
                                        MotorSpecial::AccelType::kParabolic
//                                      MotorSpecial::AccelType::kLinear
//                                      MotorSpecial::AccelType::kConstantPower
//                                      MotorSpecial::AccelType::kSigmoid

#define IN_MOTION_uSec_DELAY            1000   //time gap in uSec before accel phase end (to set out in_motion sig)
#define IN_MOTION_mSec_DELAY            (IN_MOTION_uSec_DELAY / 1000)

struct AppCfg{
    MotorSpecial::AccelCfg accelCfg;
    bool oscillation_enabled {false};
    bool direction_inverted {false};
};

auto getBaseConfig(){
    static StepperMotor::StepperCfg s_cfg{
            pin_board::PIN<pin_board::Writeable>{STEP_GPIO_Port, STEP_Pin},
            pin_board::PIN<pin_board::Writeable>{DIR_GPIO_Port, DIR_Pin},
            pin_board::PIN<pin_board::Writeable>{ENABLE_GPIO_Port, ENABLE_Pin},
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

auto getDIPConfig(){
    auto cfg = getBaseConfig();
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
    cfg.oscillation_enabled = HAL_GPIO_ReadPin(CONFIG_2_GPIO_Port, CONFIG_2_Pin);

    cfg.accelCfg.accel_type = HAL_GPIO_ReadPin(CONFIG_3_GPIO_Port, CONFIG_3_Pin) ? MotorSpecial::AccelType::kParabolic
                                                                            : MotorSpecial::AccelType::kConstantPower;
    return cfg;
}

void StartTaskTimIT(uint16_t delay){
    __HAL_TIM_SET_AUTORELOAD(&htim6, delay);
    HAL_TIM_Base_Start_IT(&htim6);
}

namespace utils{
    template<typename T>
    constexpr auto get_idx(T e){
        return static_cast<std::size_t>(e);
    }
}

namespace RBTypes{
    enum class Output : std::size_t{
        indication_0 = 1,
        indication_1 = 0,
        in_motion = 2,
    };

    enum class Input : std::size_t{
        exp_req = 0,
        grid_home = 1,
        grid_in_field = 2,
    };

    enum class Error : std::size_t{
        no_error,
        limit_switch_error,
        initial_movement_error,
        exp_req_error
    };

    enum class State : std::size_t{
        init_state,
        service_moving,
        grid_in_field,
        grid_home,
        scanning,
        oscillation,
        error,
        moving_in_field,
        moving_home
    };

    enum class TimTask : std::size_t{
        no_task,
        freeze_switch,
        in_motion_delay,
    };
    enum class MoveSpeed : std::size_t{
        slow,
        fast
    };
}