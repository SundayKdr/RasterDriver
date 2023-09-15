//
// Created by 79162 on 25.09.2021.
//

#ifndef TOMO_A4BOARD_STEPPERMOTOR_HPP
#define TOMO_A4BOARD_STEPPERMOTOR_HPP

#include <cstdlib>
#include "IO/PIN.hpp"
#include "Global.hpp"

using namespace RB::types;

struct StepperCfg
{
    float A = CONFIG1_ACCELERATION;
    float Vmin = START_SPEED;
    float Vmax = CONFIG1_SPEED;
    const int criticalNofSteps = TOTAL_DISTANCE_N_OF_STEPS;
    bool directionInverted = false;
    TIM_HandleTypeDef *htim = &htim4;
    uint32_t timChannel = TIM_CHANNEL_2;
};

class MotorController{
public:
    enum MOTOR_PIN{
        CURRENT_WIND,
        STEP_PIN,
        DIR_PIN,
        ENABLE_PIN,
        RESET_PIN,
    };
    using MOTOR_IOS = PIN<MOTOR_PIN, PinWriteable>;

    enum class Mode
    {
        IDLE,
        ACCEL,
        CONST,
        DECCEL,
        in_ERROR
    };
    enum class Direction{
        BACKWARDS = 0,
        FORWARD = 1
    };

    MotorController() = default;

    static MotorController& getRef(){
        static MotorController motorController;
        return motorController;
    }

    void load_driver(StepperCfg &&cfg){
        A = cfg.A;
        Vmin = cfg.Vmin;
        Vmax = cfg.Vmax;
        criticalNofSteps = cfg.criticalNofSteps;
        htim = cfg.htim;
        timChannel = cfg.timChannel;
        directionInverted = cfg.directionInverted;
        timerDividend = SystemCoreClock/(htim->Instance->PSC);
    }

    void motor_refresh(){
        if(mode == Mode::in_ERROR) return;
        if(accelerationMode){
            reCalcSpeed();
            if(currentStep >= EXPO_DISTANCE_N_OF_STEPS)
                changeDirection();
            regValueCalc();
        }
        else{
            currentStep++;
            if(currentStep >= criticalNofSteps){
                if(noReturn) stopMotor();
                else changeDirection();
            }
        }
        if(direction_changed > 1){
            stopMotor();
            mode = Mode::in_ERROR;
        }
    }

    inline void get_position(Direction dir, bool noRet = false){
        accelerationMode = false;
        V = LOAD_UNLOAD_SPEED;
        setDirection(dir);
        if(noRet) noReturn = true;
        startMotor();
    }

    inline void exposition(){
        accelerationMode = true;
        V = Vmin;
        setDirection(Direction::FORWARD);
        startMotor();
    }

    inline void stopMotor(){
        if(motorMoving){
            HAL_TIM_PWM_Stop_IT(htim, timChannel);
            enable.setValue(HIGH);
            motorMoving = false;
            mode = Mode::IDLE;
            event = EVENT_STOP;
            noReturn = false;
        }
    }

    inline void changeDirection(){
        if(!accelerationMode) direction_changed += 1;
        setDirection(static_cast<bool>(currentDirection) ? Direction::BACKWARDS : Direction::FORWARD);
        currentStep = 0;
        accel_step = 0;
        mode = Mode::ACCEL;
    }

    inline bool isMotorMoving() const {
        return motorMoving;
    }

    inline Mode getMode() const {
        return mode;
    }

    inline MOTOR_EVENT getEvent() const {
        return event;
    }

    inline Direction getCurrentDirection() const {
        return currentDirection;
    }

    inline bool noReturnMode() const {
        return noReturn;
    }

private:
    MOTOR_IOS step = MOTOR_IOS(STEP_PIN, STEP_GPIO_Port, STEP_Pin);
    MOTOR_IOS direction = MOTOR_IOS(DIR_PIN, DIR_GPIO_Port, DIR_Pin);
    MOTOR_IOS enable = MOTOR_IOS(ENABLE_PIN, ENABLE_GPIO_Port, ENABLE_Pin);
    MOTOR_IOS current = MOTOR_IOS(CURRENT_WIND, CURRENT_WIND_GPIO_Port, CURRENT_WIND_Pin);
    TIM_HandleTypeDef *htim;
    uint32_t timChannel;
    uint32_t timerDividend;

    int currentStep = 0;
    int accel_step = 0;

    float A = 40.0f;
    float V = 0.0f;
    float Vmin = START_SPEED;
    float Vmax = CONFIG1_SPEED;
    int criticalNofSteps = TOTAL_DISTANCE_N_OF_STEPS;

    bool directionInverted = false;
    Direction currentDirection = Direction::FORWARD;
    Mode mode = Mode::IDLE;
    MOTOR_EVENT event = EVENT_STOP;
    bool motorMoving = false;
    bool accelerationMode = false;
    bool noReturn = false;
    uint8_t direction_changed = 0;

    inline void startMotor(){
        if(!motorMoving){
            accel_step = 0;
            currentStep = 0;
            mode = Mode::ACCEL;
            direction_changed = 0;
            motorMoving = true;
            enable.setValue(LOW);
            regValueCalc();
            HAL_TIM_PWM_Start_IT(htim, timChannel);
        }
    }
    inline void regValueCalc(){
        if(V > 0){
            uint32_t buf = timerDividend / uint32_t(V);
            if(buf > 0 && buf < UINT16_MAX){
                __HAL_TIM_SET_AUTORELOAD(htim, buf);
                __HAL_TIM_SET_COMPARE(htim, timChannel,buf/2);
            }
        }
    }

    inline void setDirection(Direction newDirection){
        currentDirection = newDirection;
        if(directionInverted) direction.setValue(LOGIC_LEVEL((
                static_cast<bool>(currentDirection) ? Direction::BACKWARDS : Direction::FORWARD)));
        else direction.setValue(LOGIC_LEVEL(currentDirection));
    }

    inline void reCalcSpeed(){
        if (mode == Mode::IDLE) return;
        switch (mode)
        {
            case Mode::ACCEL:
            {
                if (V >= Vmax)
                {
                    V = Vmax;
                    event = EVENT_CSS;
                    mode = Mode::CONST;
                }else
                    V += A;

                if (accel_step >= EXPO_DISTANCE_N_OF_STEPS / 2)
                {
                    mode = Mode::DECCEL;
                    break;
                }
                accel_step++;
            }
            break;

            case Mode::CONST:
            {
                if (currentStep + accel_step >= EXPO_DISTANCE_N_OF_STEPS) {
                    mode = Mode::DECCEL;
                }
            }
            break;

            case Mode::DECCEL:
            {
                if (accel_step <= 0)
                {
                    stopMotor();
                    mode = Mode::IDLE;
                    event = EVENT_STOP;
                    break;
                }else{
                    V -= A;
                    if (V < Vmin) V = Vmin;
                    accel_step--;
                }
            }
            break;

            default:
                break;
        }

        if (mode == Mode::ACCEL || mode == Mode::CONST || mode == Mode::DECCEL)
            currentStep++;
    }
};

#endif //TOMO_A4BOARD_STEPPERMOTOR_HPP