#pragma once

#include <array>
#include <optional>
#include <functional>

#include "grid_motor.hpp"

#include "embedded_hw_utils/IO/pin.hpp"
#include "embedded_hw_utils/IO/button.hpp"
#include "app_config.hpp"

using namespace RB::types;
using namespace StepperMotor;
using namespace PIN_BOARD;

class MainController {
    using InputPinType = PIN<PinReadable>;
    using OutputPinType = PIN<PinWriteable>;
    using Dir = StepperMotor::Direction;
    using MotorStatus = StepperMotor::Mode;
public:
    const MainController& operator=(const MainController &) = delete;
    MainController& operator=(MainController &) = delete;
    MainController() = delete;
    MainController(MainController&) = delete;
    MainController(MainController&&)= delete;

    static MainController& GetRef(){
        static auto self = MainController(MotorController::GetRef());
        return self;
    }

    void UpdateConfig(){
        auto& config = getDIPConfig();
        kShakingScanEnabled_ = config.shake_scan_enabled;
        motor_controller_.UpdateConfig(config);
    }

    void InvertPins(){
        input_pin_container_[EXP_REQ].setInverted();
        input_pin_container_[GRID_HOME_DETECT].setInverted();
        input_pin_container_[GRID_INFIELD_DETECT].setInverted();
    }

    void BoardInit(){
        UpdateConfig();
        InvertPins();
        if (test_btn.getState()) TestMove();
        else InitialMove();
    }

    void TestMove(){
        motor_controller_.MoveToPos(Dir::BACKWARDS, mSTEPS(7));
        pending_move_ = [&]{
            motor_controller_.MoveToPos(Dir::FORWARD, mSTEPS(7));
        };
    }

    void InitialMove(){
        if(isSignalHigh(GRID_HOME_DETECT)){
            ChangeDeviceState(DEVICE_SERVICE_MOVING);
            motor_controller_.MoveToPos(Dir::FORWARD, RUN_OUT_STEPS);
            pending_move_ = [&]{ RasterMoveHome(true); };
        }else{
            ChangeDeviceState(DEVICE_MOVE_HOME);
            motor_controller_.MoveToEndPointSlow(Dir::BACKWARDS);
        }
    }

    bool isInState(BOARD_STATUS status){
       return currentState_ == status;
   }

    bool isSignalHigh(INPUT_TYPE pin){
        return static_cast<bool>(input_pin_container_[pin].getState());
    }

    void ChangeDeviceState(BOARD_STATUS new_status){
       currentState_ = new_status;
    }

    void StopMotor(){
        motor_controller_.StopMotor();
    }

    void SlowStopMotor(){
        ChangeDeviceState(RB::types::DEVICE_SERVICE_MOVING);
        motor_controller_.DecelAndStop();
    }

    void CorrectExpoSteps(){
        motor_controller_.EndSideStepsCorr();
    }

    void SetOutputSignal(OUTPUT_TYPE sigType, LOGIC_LEVEL level){
        output_pin_container_[sigType].setValue(level);
    }

    void BtnEventHandle(Button& btn){
            if(isInState(DEVICE_GRID_IN_FIELD))
                RasterMoveHome();
            else if(isInState(DEVICE_GRID_HOME))
                RasterMoveInField();
    }

    void ErrorsCheck(){
        if(motor_controller_.GetMode() == MotorStatus::in_ERROR)
            currentError_ = LIMIT_SWITCH_ERROR;
        if(currentError_ != NO_ERROR)
            ErrorHandler_(currentError_);
    }

    void LimitSwitchesCheck(){
        if(switch_ignore_flag_)
            return;
        HomeSwitchCheck();
        InFieldSwitchCheck();
    }

    void HomeSwitchCheck(){
        if(isSignalHigh(GRID_HOME_DETECT)){
            SetOutputSignal(INDICATION_0, HIGH);
            switch (currentState_){
                case DEVICE_MOVE_HOME:
                    StopMotor();
                    MoveCloserToSwitch();
                    ChangeDeviceState(DEVICE_GRID_HOME);
                    break;
                case DEVICE_SHAKE_SCANNING:
                    CorrectExpoSteps();
                    break;
                case DEVICE_GRID_IN_FIELD:
                case DEVICE_GRID_HOME:
                    StopMotor();
                    break;
                default:
                    break;
            }
        }else
            SetOutputSignal(INDICATION_0, LOW);
    }

    void InFieldSwitchCheck(){
        if(isSignalHigh(GRID_INFIELD_DETECT)){
            switch (currentState_) {
                case DEVICE_MOVE_IN_FIELD:
                    StopMotor();
//                    MoveCloserToSwitch();
                    ChangeDeviceState(DEVICE_GRID_IN_FIELD);
                    break;
                default:
                    break;
            }
        }
    }

    void MoveCloserToSwitch(){
        FreezeSwitchCheck(50);
        motor_controller_.MakeStepsAfterSwitch();
    }

    void ExpStateCheck(){
        if(isSignalHigh(EXP_REQ) || isInState(DEVICE_SHAKE_SCANNING) || isInState(DEVICE_SCANNING))
            ExpositionProcedure();
    }

    void CheckPendingMove(){
        if(pending_move_ && (!motor_controller_.IsMotorMoving())){
            auto callable = pending_move_.value();
            pending_move_.reset();
            callable();
        }
    }

    void BoardUpdate(){
        ErrorsCheck();
        LimitSwitchesCheck();
        ExpStateCheck();
        CheckPendingMove();
    }

    void RasterMoveInField(bool slow = false){
        if(isSignalHigh(GRID_INFIELD_DETECT)){
            ChangeDeviceState(DEVICE_GRID_IN_FIELD);
            StopMotor();
            return;
        }
        ChangeDeviceState(DEVICE_MOVE_IN_FIELD);
        slow ? motor_controller_.MoveToEndPointSlow(Dir::FORWARD) :
        motor_controller_.MoveToEndPointFast(Dir::FORWARD);
    }

    void RasterMoveHome(bool slow = false){
        if(isSignalHigh(GRID_HOME_DETECT)){
            ChangeDeviceState(DEVICE_GRID_HOME);
            StopMotor();
            return;
        }
        ChangeDeviceState(DEVICE_MOVE_HOME);
        slow ? motor_controller_.MoveToEndPointSlow(Dir::BACKWARDS) :
        motor_controller_.MoveToEndPointFast(Dir::BACKWARDS);
    }

    void FreezeSwitchCheck(uint16_t delay = 300){
        if(isInState(DEVICE_SERVICE_MOVING))
            return;
        current_tim_task_ = FREEZE_SWITCH_TASK;
        switch_ignore_flag_ = true;
        StartTaskTimIT(delay);
    }

    void RasterMoveInFieldAfterExpo(){
        SlowStopMotor();
        pending_move_ = [&]{
            motor_controller_.MoveToPos(Dir::BACKWARDS, RUN_OUT_STEPS);
            pending_move_ = [&]{ RasterMoveInField(true); };
        };
    }

    void RasterMoveHomeAfterExpo(){
        SlowStopMotor();
        pending_move_ = [&]{
            motor_controller_.MoveToPos(Dir::FORWARD, RUN_OUT_STEPS);
            pending_move_ = [&]{ RasterMoveHome(true); };
        };
    }

    void ExpRequestedOnHoneGrid(){
        ChangeDeviceState(DEVICE_ERROR);
        currentError_ = EXP_REQ_ERROR;
    }

    void SetInMotionSigWithDelay(){
        current_tim_task_ = IN_MOTION_SIG_DELAY_TASK;
        StartTaskTimIT(IN_MOTION_mSec_DELAY);
    }

    void SetInMotionSig(LOGIC_LEVEL level){
        SetOutputSignal(IN_MOTION, level);
    }

    void StartShakeExposition(){
        lastPosition_ = currentState_;
        ChangeDeviceState(DEVICE_SHAKE_SCANNING);
        motor_controller_.Exposition();
    }

    bool IsInMotionSigReady(){
        auto dir = motor_controller_.GetCurrentDirection() == StepperMotor::Direction::FORWARD;
        auto in_mode = motor_controller_.GetMode() == StepperMotor::ACCEL;
        auto in_time = motor_controller_.GetAccelTimeGap() <= IN_MOTION_uSec_DELAY;
        return in_mode && in_time && dir;
    }

    void ExpositionProcedure(){
        if(isSignalHigh(EXP_REQ)){
            switch (currentState_) {

                case DEVICE_GRID_IN_FIELD:
                    lastPosition_ = currentState_;
                    if(!kShakingScanEnabled_){
                        ChangeDeviceState(DEVICE_SCANNING);
                        SetInMotionSigWithDelay();
                    }else
                        StartShakeExposition();
                    break;

                case DEVICE_GRID_HOME:
                    if(!kRasterHomeExpReqIsOk_)
                        ExpRequestedOnHoneGrid();
                    else{
                        lastPosition_ = currentState_;
                        ChangeDeviceState(DEVICE_SCANNING);
                        SetInMotionSigWithDelay();
                    }
                    break;

                case DEVICE_SHAKE_SCANNING:
                    if(IsInMotionSigReady())
                        SetInMotionSig(HIGH);
                    break;
                default:
                    break;
            }
        }else
            FinishExpoProcedure();
    }

    void FinishExpoProcedure(){
        SetInMotionSig(LOW);
        switch (lastPosition_) {
            case DEVICE_GRID_HOME:
                if(kShakingScanEnabled_)
                    RasterMoveHome(true);
                else
                    ChangeDeviceState(DEVICE_GRID_HOME);
                break;
            case DEVICE_GRID_IN_FIELD:
                if(kShakingScanEnabled_)
                    RasterMoveInFieldAfterExpo();
                else
                    ChangeDeviceState(DEVICE_GRID_IN_FIELD);
                break;
            default:
                break;
        }
    }

    void TimTaskHandler(){
        switch (current_tim_task_) {
            case FREEZE_SWITCH_TASK:
                UnFreezeSwitches();
                break;
            case IN_MOTION_SIG_DELAY_TASK:
                SetInMotionSig(HIGH);
                break;
            default:
                break;
        }
        current_tim_task_ = NO_TASKS;
    }

    void UnFreezeSwitches(){
        switch_ignore_flag_ = false;
    }

private:
    explicit MainController(MotorController &incomeMotorController)
            :motor_controller_(incomeMotorController)
    {}

    static constexpr int kIN_PIN_CNT = 3;
    std::array<InputPinType, kIN_PIN_CNT> input_pin_container_{
            InputPinType(EXP_REQ_IN_GPIO_Port, EXP_REQ_IN_Pin),
            InputPinType(GRID_INFIELD_DETECT_GPIO_Port, GRID_INFIELD_DETECT_Pin),
            InputPinType(GRID_HOME_DETECT_GPIO_Port, GRID_HOME_DETECT_Pin),
    };
    static constexpr int kOUT_PIN_CNT = 3;
    std::array<OutputPinType, kOUT_PIN_CNT> output_pin_container_{
            OutputPinType(INDICATION_0_OUT_GPIO_Port, INDICATION_0_OUT_Pin),
            OutputPinType(INDICATION_1_OUT_GPIO_Port, INDICATION_1_OUT_Pin),
            OutputPinType(IN_MOTION_OUT_GPIO_Port, IN_MOTION_OUT_Pin)
    };
    InputPinType test_btn {NOTUSED_PUSHBUTTON_GPIO_Port, NOTUSED_PUSHBUTTON_Pin};

    MotorController& motor_controller_;
    TIM_TASKS current_tim_task_ {NO_TASKS};
    BOARD_STATUS_ERROR currentError_ {NO_ERROR};
    BOARD_STATUS currentState_ {DEVICE_INIT_STATE};
    BOARD_STATUS lastPosition_ {DEVICE_GRID_IN_FIELD};

    bool kShakingScanEnabled_ {false};
    bool switch_ignore_flag_ {false};
    const bool kRasterHomeExpReqIsOk_ {true};

    std::optional<std::function<void()>> pending_move_;

    void ErrorHandler_(BOARD_STATUS_ERROR error){
        StopMotor();
        SetOutputSignal(INDICATION_1, HIGH);
        switch (error) {
            case STANDBY_MOVEMENT_ERROR:
            case LIMIT_SWITCH_ERROR:
                break;
            case EXP_REQ_ERROR:
                return;
            default:
                break;
        }
        Error_Handler();
    }
};