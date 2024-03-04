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
        config_acceleration_ = cfg.A;
        config_Vmax_ = cfg.Vmax;
        uSec_ramp_time_ = cfg.ramp_time;
        accel_type_ = cfg.accel_type;
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
        return uSec_ramp_time_ - uSec_accel_;
    }

private:
    MotorController() = delete;
    MotorController(StepperMotor::StepperCfg& cfg)
        : StepperMotor::StepperMotorBase(cfg)
    {}

    uint32_t uSec_ramp_time_{0};

    float k_ {0};
    uint32_t config_acceleration_ {SERVICE_MOVE_ACCELERATION};
    uint32_t config_Vmax_ {SERVICE_MOVE_MAX_SPEED};

    int reach_steps_ {EXPO_OFFSET_STEPS};
    int expo_distance_steps_ {EXPO_RANGE_STEPS};
    int steps_to_stop_ {STEPS_BEFORE_DECCEL};

    MoveMode current_move_mode_;
    AccelType accel_type_ = ACCEL_TYPE;

    void CalcBeforeStartImpl() override{
        k_ = static_cast<float>(Vmax_ - Vmin_) / uSec_ramp_time_;
    }

    void AccelerationImpl() override{
        if(accel_type_ == AccelType::kLinear)
            KAcceleration(static_cast<float >(uSec_accel_));

        else if(accel_type_ == AccelType::kParabolic)
            ParabolicAcceleration();

        else if(accel_type_ == AccelType::kConstantPower){
//            auto x = static_cast<float>(std::sqrt(uSec_accel_));
            auto x = sqrtf(uSec_accel_);
            KAcceleration(x);
        }
    }

    void KAcceleration(float x){
        switch (mode_){
            case Mode::ACCEL:
                V_ = static_cast<uint32_t>(k_ * x) + Vmin_;
                break;
            case Mode::DECCEL:
                V_ = static_cast<uint32_t>((-k_) * x) + Vmax_;
                break;
            default:
                break;
        }
    }

    void ParabolicAcceleration(){
        switch (mode_){
            case Mode::ACCEL:
                V_ += config_acceleration_;
                break;

            case Mode::DECCEL:
                V_ -= config_acceleration_;
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
