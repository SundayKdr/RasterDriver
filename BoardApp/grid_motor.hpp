#pragma once

#include "app_config.hpp"
#include "embedded_hw_utils/StepperMotor/accel_motor.hpp"

#include <cmath>

using namespace MotorSpecial;

class MotorController : public AccelMotor{
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

    void UpdateConfig(AppCfg& cfg){
        directionInverted_ = cfg.direction_inverted;
        AccelMotor::UpdateConfig(cfg.accelCfg);
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
                      dir, STEPS_BEFORE_DECCEL);
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

private:
    MotorController() = delete;
    MotorController(AppCfg& cfg)
        :AccelMotor(cfg.accelCfg)
    {
        UpdateConfig(cfg);
    }

    int reach_steps_ {EXPO_OFFSET_STEPS};
    int expo_distance_steps_ {EXPO_RANGE_STEPS};

    MoveMode current_move_mode_;

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
