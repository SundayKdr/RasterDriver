#pragma once

#include "app_config.hpp"
#include "StepperMotor/stepper_motor_base.hpp"

#include <cmath>

using namespace StepperMotor;

class MotorController : public StepperMotor::StepperMotorBase{
public:
    const MotorController& operator=(const MotorController &) = delete;
    MotorController& operator=(MotorController &) = delete;
    MotorController(MotorController&) = delete;
    MotorController(MotorController&&)= delete;

    enum class MoveMode{
        kExpo,
        kService_accel,
        kService_slow,
        kSwitch_press,
        kDecel_and_stop
    };

    void UpdateConfig(StepperMotor::StepperCfg& cfg){
        directionInverted_ = cfg.directionInverted;
        A_ = cfg.A;
        accel_type_ = cfg.accel_type;
        if(cfg.ramp_time != T_ || cfg.Vmax != config_Vmax_){
            config_Vmax_ = cfg.Vmax;
            T_ = cfg.ramp_time;
            ReCalcKFactors();
        }
    }

    static MotorController& GetRef(){
        static MotorController motorController(getDIPConfig());
        return motorController;
    }
    
    void MoveToPos(StepperMotor::Direction dir, uint32_t steps){
        current_move_mode_ = MoveMode::kService_slow;
        MakeMotorTask(INITIAL_SPEED, INIT_MOVE_MAX_SPEED,
                      dir, steps);
    }

    void MoveToEndPointSlow(StepperMotor::Direction dir){
        current_move_mode_ = MoveMode::kService_slow;
        MakeMotorTask(INITIAL_SPEED, INIT_MOVE_MAX_SPEED,
                      dir);
    }

    void MoveToEndPointFast(StepperMotor::Direction dir){
        current_move_mode_ = MoveMode::kService_accel;
        MakeMotorTask(INITIAL_SPEED, SERVICE_MOVE_MAX_SPEED,
                      dir, steps_to_stop_);
    }

    void MakeStepsAfterSwitch(){
        current_move_mode_ = MoveMode::kSwitch_press;
        MakeMotorTask(INITIAL_SPEED, INITIAL_SPEED,
                      currentDirection_, SWITCH_PRESS_STEPS);
    }

    void Exposition(StepperMotor::Direction dir = StepperMotor::Direction::BACKWARDS){
        current_move_mode_ = MoveMode::kExpo;
        MakeMotorTask(INITIAL_SPEED, config_Vmax_, dir, expo_distance_steps_);
        StepsCorrectionHack();
    }

    void ChangeDirAbnormalExpo(){
        if(currentDirection_ == StepperMotor::Direction::BACKWARDS)
            ChangeDirection();
        StepsCorrectionHack();
    }

    void DecelAndStop(){
        current_move_mode_ = MoveMode::kDecel_and_stop;
        mode_ = StepperMotor::DECCEL;
    }

    void EndSideStepsCorr(){
        steps_to_go_ -= reach_steps_;
        ChangeDirection();
    }

    void StepsCorrectionHack(){
        currentStep_ -= reach_steps_;
    }

    uint32_t GetAccelTimeGap(){
        return T_ - uSec_accel_;
    }

private:
    MotorController() = delete;
    MotorController(StepperMotor::StepperCfg& cfg)
        : StepperMotor::StepperMotorBase(cfg)
    {
        ReCalcKFactors();
    }

    struct{
        float k1 {1};
        float k2 {1};
        float y_offset {1};
        int x_offset {1};
        const uint8_t k3  {2};       //move sigmoid center (k3=2) from y_axis (0<k3<2) to y_axis (2<k3<8)

        float core_f(int x) const{
            return x / (k2 + std::abs(x));
        }
        void KCalc(uint32_t tTotal, float Vmax, float Vmin){
            x_offset = tTotal / k3;
            k2 = x_offset * (1 / ( 1 - (Vmin / (Vmax/2))) - 1);
            k1 = 1 / (core_f(tTotal - x_offset) + 1);
            k2 = x_offset * (1 / ( 1 - (Vmin / (Vmax * k1))) - 1);
            k1 = 1 / (core_f(tTotal - x_offset) + 1);
            y_offset = k1 * Vmax;
        }
        uint32_t VCalc(uint32_t uSec){
            return y_offset * core_f(uSec - x_offset) + y_offset;
        }
    }sigmoid_;

    float k_ {1};

    uint32_t T_ {CONFIG1_RAMP_TIME};
    uint32_t A_ {1};
    uint32_t config_Vmax_ {SERVICE_MOVE_MAX_SPEED};

    int reach_steps_ {EXPO_OFFSET_STEPS};
    int expo_distance_steps_ {EXPO_RANGE_STEPS};
    int steps_to_stop_ {STEPS_BEFORE_DECCEL};

    MoveMode current_move_mode_;
    AccelType accel_type_ = ACCEL_TYPE;

    void ReCalcKFactors(){
        switch (accel_type_) {
            case kLinear:
                k_ = static_cast<float>(config_Vmax_ - Vmin_) / T_;
                break;
            case kConstantPower:
                k_ = (config_Vmax_ - Vmin_) / sqrtf(T_);
                break;
            case kParabolic:
                break;
            case kSigmoid:
                sigmoid_.KCalc(T_, static_cast<float>(config_Vmax_), static_cast<float>(Vmin_));
                break;
        }
    }

    void AccelerationImpl(){
        switch (accel_type_){
            case kLinear:
                V_ = static_cast<uint32_t>(k_ * uSec_accel_) + Vmin_;
                break;
            case kConstantPower:
                V_ = static_cast<uint32_t>(k_ * sqrtf(uSec_accel_)) + Vmin_;
                break;
            case kSigmoid:
                V_ = sigmoid_.VCalc(uSec_accel_);
                break;
            case kParabolic:
                ParabolicAcceleration();
                break;
        }
    }

    void ParabolicAcceleration(){
        switch (mode_){
            case Mode::ACCEL:
                V_ += A_;
                break;

            case Mode::DECCEL:
                V_ -= A_;
                break;
            default:
                break;
        }
    }

    void AppCorrection() override{
        switch (current_move_mode_){
            case MoveMode::kExpo:
                if(currentStep_ >= steps_to_go_)
                    ChangeDirection();
                break;
            case MoveMode::kService_slow:
            case MoveMode::kSwitch_press:
                if(currentStep_ >= steps_to_go_)
                    StopMotor();
                break;
            case MoveMode::kService_accel:
                break;
            case MoveMode::kDecel_and_stop:
                if(V_ == Vmin_)
                    StopMotor();
                break;
        }
    }
};
