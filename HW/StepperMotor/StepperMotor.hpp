
#ifndef TOMO_A4BOARD_STEPPERMOTOR_HPP
#define TOMO_A4BOARD_STEPPERMOTOR_HPP

#include <cstdlib>
#include "IO/PIN.hpp"

using namespace pin_impl;

namespace StepperMotor{

struct StepperCfg
{
    float A;
    float Vmin;
    float Vmax;
    int criticalNofSteps;
    int reach_steps;
    int expo_distance_steps;
    bool directionInverted;
    TIM_HandleTypeDef* htim;
    uint32_t timChannel {0x00000000U};
};

enum MOTOR_EVENT {
    EVENT_NULL = 0,
    EVENT_STOP,
    EVENT_CSS,  //	constant speed reached
    EVENT_CSE   //  constant speed end
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

    using MOTOR_IOS = PIN<PinWriteable>;

    enum Mode
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

    const MotorController& operator=(const MotorController &) = delete;
    MotorController& operator=(MotorController &) = delete;
    MotorController(MotorController&) = delete;
    MotorController(MotorController&&)= delete;

    static MotorController& GetRef(){
        static MotorController motorController;
        return motorController;
    }

    static void LoadDriver(StepperCfg &&cfg){
        auto&& ref = GetRef();
        ref.A_ = cfg.A;
        ref.Vmin_ = cfg.Vmin;
        ref.Vmax_ = cfg.Vmax;
        ref.service_speed_ = cfg.Vmin;
        ref.expo_distance_steps_ = cfg.expo_distance_steps;
        ref.kCriticalNofSteps_ = cfg.criticalNofSteps;
        ref.reach_steps_ = cfg.reach_steps;
        ref.htim_ = cfg.htim;
        ref.timChannel_ = cfg.timChannel;
        ref.directionInverted_ = cfg.directionInverted;
        ref.timerDividend_ = SystemCoreClock/(ref.htim_->Instance->PSC);
    }

    void load_driver(StepperCfg &&cfg){
        A_ = cfg.A;
        Vmin_ = cfg.Vmin;
        Vmax_ = cfg.Vmax;
        service_speed_ = cfg.Vmin;
        expo_distance_steps_ = cfg.expo_distance_steps;
        kCriticalNofSteps_ = cfg.criticalNofSteps;
        reach_steps_ = cfg.reach_steps;
        htim_ = cfg.htim;
        timChannel_ = cfg.timChannel;
        directionInverted_ = cfg.directionInverted;
        timerDividend_ = htim_ ? SystemCoreClock/(htim_->Instance->PSC) : 1;
    }

    void MotorRefresh(){
        if(mode_ == IDLE || mode_ == Mode::in_ERROR)
            return;
        if(accelerationMode_){
            if(currentStep_ >= expo_distance_steps_)
                ChangeDirection();
            else
                CalcSpeed();
            CalcRegValue();
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

    void GetPosition(Direction dir, bool noRet = false){
        if(motorMoving_)
            StopMotor();
        accelerationMode_ = false;
        V_ = service_speed_;
        SetDirection(dir);
        if(noRet) noReturn_ = true;
        StartMotor();
    }

    void Exposition(Direction dir = Direction::BACKWARDS){
        accelerationMode_ = true;
        V_ = Vmin_;
        SetDirection(dir);
        StartMotor();
    }

    void EndSideStepsCorr(){
        kCriticalNofSteps_ -= reach_steps_;
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
        if(!accelerationMode_)
            direction_changed_ += 1;
        SetDirection(static_cast<bool>(currentDirection_) ? Direction::BACKWARDS : Direction::FORWARD);
        V_ = Vmin_;
        mode_ = Mode::ACCEL;
        CalcRegValue();
        currentStep_ = 0;
        accel_step_ = 0;
    }

    void StepsCorrectionHack(){
        currentStep_ -= reach_steps_;
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
    MotorController() = default;

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
    float Vmin_ = 0.0f;
    float Vmax_ = 40.0f;
    int kCriticalNofSteps_ = 0;
    int reach_steps_ = 0;
    int expo_distance_steps_ = 0;
    float service_speed_ = (float)0;

    bool directionInverted_ = false;
    Direction currentDirection_ = Direction::FORWARD;
    Mode mode_ = Mode::IDLE;
    MOTOR_EVENT event_ = EVENT_STOP;
    bool motorMoving_ = false;
    bool accelerationMode_ = false;
    bool noReturn_ = false;
    uint8_t direction_changed_ = 0;

    void StartMotor(){
        if(!motorMoving_){
            accel_step_ = 0;
            currentStep_ = 0;
            mode_ = Mode::ACCEL;
            direction_changed_ = 0;
            motorMoving_ = true;
            enable_.setValue(LOW);
            CalcRegValue();
            HAL_TIM_PWM_Start_IT(htim_, timChannel_);
        }
    }
    void CalcRegValue(){
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
        if(directionInverted_) direction_.setValue(
                static_cast<bool>(currentDirection_) ? LOGIC_LEVEL(Direction::BACKWARDS) : LOGIC_LEVEL(Direction::FORWARD));
        else direction_.setValue(LOGIC_LEVEL(currentDirection_));
    }

    void CalcSpeed(){
        if (mode_ == Mode::IDLE) return;
        switch (mode_)
        {
            case Mode::ACCEL:
            {
                if (V_ >= Vmax_)
                {
                    V_ = Vmax_;
                    event_ = EVENT_CSS;
                    mode_ = Mode::CONST;
                }else
                    V_ += A_;

                if (accel_step_ >= expo_distance_steps_ / 2)
                {
                    mode_ = Mode::DECCEL;
                    break;
                }
                accel_step_++;
            }
                break;

            case Mode::CONST:
            {
                if (currentStep_ + accel_step_ >= expo_distance_steps_) {
                    mode_ = Mode::DECCEL;
                }
            }
                break;

            case Mode::DECCEL:
            {
                if (accel_step_ > 0)
                {
                    V_ -= A_;
                    if (V_ < Vmin_) V_ = Vmin_;
                    accel_step_--;
                }else{
                    StopMotor();
                    mode_ = Mode::IDLE;
                    event_ = EVENT_STOP;
                    break;
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

} //namespace StepperMotor

#endif //TOMO_A4BOARD_STEPPERMOTOR_HPP