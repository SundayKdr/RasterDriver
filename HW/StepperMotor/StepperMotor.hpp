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
        A_ = cfg.A;
        kVmin_ = cfg.Vmin;
        kVmax_ = cfg.Vmax;
        kCriticalNofSteps_ = cfg.criticalNofSteps;
        htim_ = cfg.htim;
        timChannel_ = cfg.timChannel;
        directionInverted_ = cfg.directionInverted;
        timerDividend_ = SystemCoreClock/(htim_->Instance->PSC);
    }

    void motor_refresh(){
        if(mode_ == Mode::in_ERROR) return;
        if(accelerationMode_){
            ReCalcSpeed();
            if(currentStep_ >= kExpoDistanceSteps_)
                ChangeDirection();
            RegValueCalc_();
        }
        else{
            currentStep_++;
            if(currentStep_ >= kCriticalNofSteps_){
                if(noReturn_) StopMotor();
                else ChangeDirection();
            }
        }
        if(direction_changed_ > 1){
            StopMotor();
            mode_ = Mode::in_ERROR;
        }
    }

    void get_position(Direction dir, bool noRet = false){
        accelerationMode_ = false;
        V_ = kServiceSpeed_;
        SetDirection(dir);
        if(noRet) noReturn_ = true;
        StartMotor_();
    }

    void Exposition(){
        accelerationMode_ = true;
        V_ = kVmin_;
        SetDirection(Direction::FORWARD);
        StartMotor_();
        StepsCorrectionHack();
    }

    void EndSideStepsCorr(){
        kCriticalNofSteps_ -= kReachSteps_;
        ChangeDirection();
    }

    void StopMotor(){
        if(motorMoving_){
            HAL_TIM_PWM_Stop_IT(htim_, timChannel_);
            enable_.setValue(HIGH);
            motorMoving_ = false;
            mode_ = Mode::IDLE;
            event_ = EVENT_STOP;
            noReturn_ = false;
        }
    }

    void ChangeDirection(){
        if(!accelerationMode_) direction_changed_ += 1;
        SetDirection(static_cast<bool>(currentDirection_) ? Direction::BACKWARDS : Direction::FORWARD);
        currentStep_ = 0;
        accel_step_ = 0;
        mode_ = Mode::ACCEL;
    }

    [[nodiscard]] bool IsMotorMoving() const {
        return motorMoving_;
    }

    [[nodiscard]] Mode GetMode() const {
        return mode_;
    }

    [[nodiscard]] MOTOR_EVENT GetEvent() const {
        return event_;
    }

    [[nodiscard]] Direction GetCurrentDirection() const {
        return currentDirection_;
    }

    [[nodiscard]] bool NoReturnMode() const {
        return noReturn_;
    }

private:
    MOTOR_IOS step_ = MOTOR_IOS(STEP_PIN, STEP_GPIO_Port, STEP_Pin);
    MOTOR_IOS direction_ = MOTOR_IOS(DIR_PIN, DIR_GPIO_Port, DIR_Pin);
    MOTOR_IOS enable_ = MOTOR_IOS(ENABLE_PIN, ENABLE_GPIO_Port, ENABLE_Pin);
    MOTOR_IOS current_ = MOTOR_IOS(CURRENT_WIND, CURRENT_WIND_GPIO_Port, CURRENT_WIND_Pin);
    TIM_HandleTypeDef *htim_;
    uint32_t timChannel_;
    uint32_t timerDividend_;

    int currentStep_ = 0;
    int accel_step_ = 0;

    float A_ = 40.0f;
    float V_ = 0.0f;
    float kVmin_ = START_SPEED;
    float kVmax_ = CONFIG1_SPEED;
    int kCriticalNofSteps_ = TOTAL_DISTANCE_N_OF_STEPS;
    int kReachSteps_ = EXPO_REACH_DISTANCE_N_OF_STEPS;
    int kExpoDistanceSteps_ = EXPO_DISTANCE_N_OF_STEPS;
    float kServiceSpeed_ = (float)LOAD_UNLOAD_SPEED;

    bool directionInverted_ = false;
    Direction currentDirection_ = Direction::FORWARD;
    Mode mode_ = Mode::IDLE;
    MOTOR_EVENT event_ = EVENT_STOP;
    bool motorMoving_ = false;
    bool accelerationMode_ = false;
    bool noReturn_ = false;
    uint8_t direction_changed_ = 0;

    void StartMotor_(){
        if(!motorMoving_){
            accel_step_ = 0;
            currentStep_ = 0;
            mode_ = Mode::ACCEL;
            direction_changed_ = 0;
            motorMoving_ = true;
            enable_.setValue(LOW);
            RegValueCalc_();
            HAL_TIM_PWM_Start_IT(htim_, timChannel_);
        }
    }
    void RegValueCalc_(){
        if(V_ > 0){
            uint32_t buf = timerDividend_ / uint32_t(V_);
            if(buf > 0 && buf < UINT16_MAX){
                __HAL_TIM_SET_AUTORELOAD(htim_, buf);
                __HAL_TIM_SET_COMPARE(htim_, timChannel_,buf/2);
            }
        }
    }

    void SetDirection(Direction newDirection){
        currentDirection_ = newDirection;
        if(directionInverted_) direction_.setValue(LOGIC_LEVEL((
                static_cast<bool>(currentDirection_) ? Direction::BACKWARDS : Direction::FORWARD)));
        else direction_.setValue(LOGIC_LEVEL(currentDirection_));
    }

    void StepsCorrectionHack(){
        currentStep_ -= kReachSteps_;
    }

    void ReCalcSpeed(){
        if (mode_ == Mode::IDLE) return;
        switch (mode_)
        {
            case Mode::ACCEL:
            {
                if (V_ >= kVmax_)
                {
                    V_ = kVmax_;
                    event_ = EVENT_CSS;
                    mode_ = Mode::CONST;
                }else
                    V_ += A_;

                if (accel_step_ >= kExpoDistanceSteps_ / 2)
                {
                    mode_ = Mode::DECCEL;
                    break;
                }
                accel_step_++;
            }
            break;

            case Mode::CONST:
            {
                if (currentStep_ + accel_step_ >= kExpoDistanceSteps_) {
                    mode_ = Mode::DECCEL;
                }
            }
            break;

            case Mode::DECCEL:
            {
                if (accel_step_ <= 0)
                {
                    StopMotor();
                    mode_ = Mode::IDLE;
                    event_ = EVENT_STOP;
                    break;
                }else{
                    V_ -= A_;
                    if (V_ < kVmin_) V_ = kVmin_;
                    accel_step_--;
                }
            }
            break;

            default:
                break;
        }

        if (mode_ == Mode::ACCEL || mode_ == Mode::CONST || mode_ == Mode::DECCEL)
            currentStep_++;
    }
};

#endif //TOMO_A4BOARD_STEPPERMOTOR_HPP