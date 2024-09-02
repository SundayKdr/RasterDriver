#pragma once

#include "app_config.hpp"
#include "embedded_hw_utils/motors/stepper_motor/accel_motor.hpp"

#include <cmath>

using namespace MotorSpecial;

class MotorController : public AccelMotor{
public:
    MotorController() = delete;
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

    void UpdateConfig(AppCfg cfg){
        SetDirInversion(cfg.direction_inverted);
        AccelMotor::UpdateConfig(cfg.accelCfg);
    }

    static MotorController& global(){
        static MotorController motorController(getDIPConfig());
        return motorController;
    }
    
    void MoveToPos(StepperMotor::Direction dir, uint32_t steps){
        current_state_ = MoveMode::kService_slow;
        MakeMotorTask(INITIAL_SPEED, INIT_MOVE_MAX_SPEED,
                      dir, steps);
    }

    void MoveToEndPointSlow(StepperMotor::Direction dir){
        current_state_ = MoveMode::kService_slow;
        MakeMotorTask(INITIAL_SPEED, INIT_MOVE_MAX_SPEED,
                      dir, GetTotalRangeSteps());
    }

    void MoveToEndPointFast(StepperMotor::Direction dir){
        current_state_ = MoveMode::kService_accel;
        MakeMotorTask(INITIAL_SPEED, SERVICE_MOVE_MAX_SPEED,
                      dir, STEPS_BEFORE_DECCEL);
    }

    void MakeStepsAfterSwitch(){
        current_state_ = MoveMode::kSwitch_press;
        MakeMotorTask(INITIAL_SPEED, INITIAL_SPEED,
                      CurrentDirection(), SWITCH_PRESS_STEPS);
    }

    void Exposition(StepperMotor::Direction dir = StepperMotor::Direction::BACKWARDS){
        current_state_ = MoveMode::kExpo;
        MakeMotorTask(INITIAL_SPEED, config_Vmax_, dir, expo_distance_steps_);
    }

    void ChangeDirAbnormalExpo(){
        if(CurrentDirection() == StepperMotor::Direction::BACKWARDS)
            ChangeDirection();
        StepsCorrectionHack();
    }

    void SlowDownAndStop(){
        current_state_ = MoveMode::kDecel_and_stop;
        SetMode(StepperMotor::DECCEL);
    }

    void EndSideStepsCorr(){
        CorrectStepsToGo(-reach_steps_);
        ChangeDirection();
    }

    void StepsCorrectionHack(){
        CorrectCurrentStep(reach_steps_);
    }

private:
    MotorController(AppCfg cfg)
        :AccelMotor(cfg.accelCfg)
    {
        UpdateConfig(cfg);
    }

    int reach_steps_ {EXPO_OFFSET_STEPS};
    int expo_distance_steps_ {EXPO_RANGE_STEPS};

    MoveMode current_state_;

    void AppCorrection() override{
        switch (current_state_){
            case MoveMode::kExpo:
                if(CurrentStep() >= StepsToGo())
                    ChangeDirection();
                break;
            case MoveMode::kService_slow:
            case MoveMode::kSwitch_press:
                if(CurrentStep() >= StepsToGo())
                    StopMotor();
                break;
            case MoveMode::kService_accel:
                break;
            case MoveMode::kDecel_and_stop:
                if(V_ == CurrentMinSpeed())
                    StopMotor();
                break;
        }
    }
};
