#pragma once

#include "app_config.hpp"
#include "StepperMotor/StepperMotorBase.hpp"

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
        kSwitch_press
    };

    void UpdateConfig(StepperMotor::StepperCfg& cfg){
        directionInverted_ = cfg.directionInverted;
        config_acceleration_ = cfg.A;
        config_Vmax_ = cfg.Vmax;
    }

    static MotorController& GetRef(){
        static MotorController motorController(getDIPConfig());
        return motorController;
    }

    void GetPositionSlow(StepperMotor::Direction dir){
        current_move_mode_ = MoveMode::kService_slow;
        MakeMotorTask(SERVICE_MOVE_ACCELERATION, SERVICE_MOVE_START_SPEED, INIT_MOVE_MAX_SPEED,
                      dir);
    }

    void GetPosition(StepperMotor::Direction dir){
        current_move_mode_ = MoveMode::kService_accel;
        MakeMotorTask(SERVICE_MOVE_ACCELERATION, SERVICE_MOVE_START_SPEED, SERVICE_MOVE_MAX_SPEED,
                      dir, steps_to_stop_);
    }

    void MakeStepsAfterSwitch(int steps = STEPS_AFTER_SWITCH){
        current_move_mode_ = MoveMode::kSwitch_press;
        MakeMotorTask(SERVICE_MOVE_ACCELERATION, 300, 300,
                      currentDirection_, steps);
    }

    void Exposition(StepperMotor::Direction dir = StepperMotor::Direction::BACKWARDS){
        current_move_mode_ = MoveMode::kExpo;
        MakeMotorTask(config_acceleration_, SERVICE_MOVE_START_SPEED, config_Vmax_,
                   dir, expo_distance_steps_);
        StepsCorrectionHack();
    }

    void ChangeDirAbnormalExpo(){
        if(currentDirection_ == StepperMotor::Direction::BACKWARDS)
            ChangeDirection();
        StepsCorrectionHack();
    }

    void EndSideStepsCorr(){
        steps_to_go_ -= reach_steps_;
        ChangeDirection();
    }

    void StepsCorrectionHack(){
        currentStep_ -= reach_steps_;
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
        }
    }

private:
    MotorController() = delete;
    MotorController(StepperMotor::StepperCfg& cfg)
        : StepperMotor::StepperMotorBase(cfg)
    {}

    float config_acceleration_ {SERVICE_MOVE_ACCELERATION};
    float config_Vmax_ {SERVICE_MOVE_MAX_SPEED};

    int reach_steps_ {EXPO_REACH_STEPS};
    int expo_distance_steps_ {EXPO_DISTANCE_STEPS};
    int steps_to_stop_ {STEPS_BEFORE_DECCEL};

    MoveMode current_move_mode_;
};
