#pragma once

#include <cstdlib>
#include <algorithm>

#include "IO/PIN.hpp"

#include "stm32g4xx_hal_tim.h"

using namespace pin_impl;

namespace StepperMotor{

    struct StepperCfg
    {
        PIN<PinWriteable> step_pin;
        PIN<PinWriteable> direction_pin;
        PIN<PinWriteable> enable_pin;
        PIN<PinWriteable> current_pin;
        TIM_HandleTypeDef* htim {};
        uint32_t timChannel {0x00000000U};
        uint32_t criticalNofSteps {};
        bool directionInverted {};
        bool shake_scan_enabled_ {};
        float A {};
        float Vmax {};
        float Vmin {};
    };

    enum MOTOR_EVENT {
        EVENT_NULL = 0,
        EVENT_STOP,
        EVENT_CSS,  //	constant speed reached
        EVENT_CSE   //  constant speed end
    };

    enum MOTOR_PIN{
        CURRENT_WIND,
        STEP_PIN,
        DIR_PIN,
        ENABLE_PIN,
        RESET_PIN,
    };

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

    class StepperMotorBase{
    public:
        using MOTOR_IOS = PIN<PinWriteable>;

        const StepperMotorBase& operator=(const StepperMotorBase &) = delete;
        StepperMotorBase& operator=(StepperMotorBase &) = delete;
        StepperMotorBase(StepperMotorBase&) = delete;
        StepperMotorBase(StepperMotorBase&&)= delete;

        void MotorRefresh(){
            if(mode_ == IDLE || mode_ == Mode::in_ERROR)
                return;
            AppCorrection();
            CalcSpeed_();
            CalcRegValue_();
        }

        virtual void AppCorrection() = 0;

        void StartMotor(float acceleration,
                        float start_speed,
                        float max_speed,
                        Direction dir = currentDirection_,
                        uint32_t steps = kCriticalNofSteps_)
        {
            if(motorMoving_)
                StopMotor();
            A_ = acceleration;
            Vmin_ = start_speed;
            Vmax_ = max_speed;
            V_ = start_speed;
            SetDirection_(dir);
            StartMotor_(steps);
        }

        void StopMotor(){
            if(motorMoving_){
                HAL_TIM_PWM_Stop_IT(htim_, timChannel_);
                enable_pin_.setValue(HIGH);
                motorMoving_ = false;
                mode_ = Mode::IDLE;
                event_ = EVENT_STOP;
            }
        }

        void ChangeDirection(){
            SetDirection_(static_cast<bool>(currentDirection_) ? Direction::BACKWARDS : Direction::FORWARD);
            V_ = Vmin_;
            mode_ = Mode::ACCEL;
            CalcRegValue_();
            currentStep_ = 0;
            accel_step_ = 0;
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
    protected:
        StepperMotorBase() = delete;
        explicit StepperMotorBase(StepperMotor::StepperCfg& cfg)
            :step_pin_(cfg.step_pin),
             direction_pin_(cfg.direction_pin),
             enable_pin_(cfg.enable_pin),
             current_pin_(cfg.current_pin),
             timChannel_(cfg.timChannel),
             htim_(cfg.htim),
             A_(cfg.A),
             Vmax_(cfg.Vmax),
             Vmin_(cfg.Vmin),
             directionInverted_(cfg.directionInverted)
        {
            kCriticalNofSteps_ = cfg.criticalNofSteps;
            timerDividend_ = SystemCoreClock / (cfg.htim->Instance->PSC);
        };

        MOTOR_IOS step_pin_;
        MOTOR_IOS direction_pin_;
        MOTOR_IOS enable_pin_;
        MOTOR_IOS current_pin_;

        TIM_HandleTypeDef *htim_;
        uint32_t timChannel_;
        uint32_t timerDividend_;

        inline static uint32_t kCriticalNofSteps_ {0};
        uint32_t steps_to_go_ {0};

        int currentStep_ {0};
        int accel_step_ {0};

        float A_ {0.0f};
        float V_ {0.0f};
        float Vmin_ {0.0f};
        float Vmax_ {0.0f};

        inline static Direction currentDirection_ {Direction::FORWARD};
        Mode mode_ {Mode::IDLE};
        MOTOR_EVENT event_ {EVENT_STOP};

        bool directionInverted_ {false};
        bool motorMoving_ {false};

        void StartMotor_(uint32_t steps){
            if(!motorMoving_){
                accel_step_ = 0;
                currentStep_ = 0;
                mode_ = Mode::ACCEL;
                motorMoving_ = true;
                steps_to_go_ = steps;
                enable_pin_.setValue(LOW);
                CalcRegValue_();
                HAL_TIM_PWM_Start_IT(htim_, timChannel_);
            }
        }

        void CalcRegValue_(){
            if(V_ > 0){
                uint32_t buf = timerDividend_ / uint32_t(V_);
                if(buf > 0 && buf < UINT16_MAX){
                    __HAL_TIM_SET_AUTORELOAD(htim_, buf);
                    __HAL_TIM_SET_COMPARE(htim_, timChannel_,buf/2);
                }
            }
        }

        void SetDirection_(Direction newDirection){
            currentDirection_ = newDirection;
            if(directionInverted_) direction_pin_.setValue(
                        static_cast<bool>(currentDirection_) ?
                            LOGIC_LEVEL(Direction::BACKWARDS) : LOGIC_LEVEL(Direction::FORWARD));
            else direction_pin_.setValue(LOGIC_LEVEL(currentDirection_));
        }

        void CalcSpeed_(){
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

                    if (accel_step_ >= steps_to_go_ / 2)
                    {
                        mode_ = Mode::DECCEL;
                        break;
                    }
                    accel_step_++;
                }
                break;

                case Mode::CONST:
                {
                    if(currentStep_ + accel_step_ >= steps_to_go_)
                        mode_ = Mode::DECCEL;
                }
                break;

                case Mode::DECCEL:
                {
                    if(accel_step_ > 0){
                        V_ -= A_;
                        if (V_ < Vmin_) V_ = Vmin_;
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

} //namespace StepperMotor