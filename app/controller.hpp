#pragma once

#include <array>
#include <optional>
#include <functional>

#include "grid_motor.hpp"

#include "embedded_hw_utils/IO/pin.hpp"
#include "embedded_hw_utils/IO/button.hpp"
#include "app_config.hpp"
#include "input_signal.hpp"

using namespace RBTypes;
using namespace StepperMotor;
using namespace pin_board;

class MainController {
    using InputPin = PIN<Readable>;
    using OutputPin = PIN<Writeable>;
    using Dir = StepperMotor::Direction;
    using MotorStatus = StepperMotor::Mode;
public:
    const MainController& operator=(const MainController &) = delete;
    MainController& operator=(MainController &) = delete;
    MainController() = delete;
    MainController(MainController&) = delete;
    MainController(MainController&&)= delete;

    static MainController& global(){
        static auto self = MainController(MotorController::global());
        return self;
    }

    void UpdateConfig(){
        auto config = getDIPConfig();
        oscillation_enabled_ = config.oscillation_enabled;
        motor_controller_.UpdateConfig(config);
    }

    void InvertPins(){
        input_pins_[utils::get_idx(Input::exp_req)].setInverted();
        input_pins_[utils::get_idx(Input::grid_home)].setInverted();
        input_pins_[utils::get_idx(Input::grid_in_field)].setInverted();
    }

    void BoardInit(){
        UpdateConfig();
        InvertPins();
        test_btn.getState() ? TestMove() : InitialMove();
    }

    void TestMove(){
        motor_controller_.MoveToPos(Dir::BACKWARDS, $mSTEPS(7));
        pending_move_ = [&]{
            motor_controller_.MoveToPos(Dir::FORWARD, $mSTEPS(7));
        };
    }

    void InitialMove(){
        if(isSignalHigh(Input::grid_home)){
            ChangeDeviceState(State::service_moving);
            motor_controller_.MoveToPos(Dir::FORWARD, RUN_OUT_STEPS);
            pending_move_ = [&]{ RasterMoveHome(MoveSpeed::slow); };
        }else{
            ChangeDeviceState(State::moving_home);
            motor_controller_.MoveToEndPointSlow(Dir::BACKWARDS);
        }
    }

    bool isInState(State status){
       return current_state_ == status;
   }

    bool isSignalHigh(Input pin){
        return static_cast<bool>(input_pins_[utils::get_idx(pin)].getState());
    }

    void ChangeDeviceState(State new_state){
        current_state_ = new_state;
    }

    void StopMotor(){
        motor_controller_.StopMotor();
    }

    void SlowStopMotor(){
        ChangeDeviceState(State::service_moving);
        motor_controller_.SlowDownAndStop();
    }

    void CorrectExpoSteps(){
        motor_controller_.EndSideStepsCorr();
    }

    void SetOutputSignal(Output sigType, logic_level level){
        output_pins_[utils::get_idx(sigType)].setValue(level);
    }

    void BtnEventHandle(){
        if(isInState(State::grid_in_field))
            RasterMoveHome(MoveSpeed::fast);
        else if(isInState(State::grid_home))
            RasterMoveInField(MoveSpeed::fast);
    }

    void ErrorsCheck(){
        if(motor_controller_.CurrentMoveMode() == MotorStatus::in_ERROR)
            currentError_ = Error::limit_switch_error;
        if(currentError_ != Error::no_error)
            ErrorHandler_(currentError_);
    }

    void LimitSwitchesCheck(){
        if(switch_ignore_flag_)
            return;
        HomeSwitchCheck();
        InFieldSwitchCheck();
    }

    void HomeSwitchCheck(){
        if(isSignalHigh(Input::grid_home)){
            SetOutputSignal(Output::indication_0, HIGH);
            switch (current_state_){
                case State::moving_home:
                    StopMotor();
                    ChangeDeviceState(State::grid_home);
                    motor_controller_.StandByModeOn();
                    break;
                case State::oscillation:
                    CorrectExpoSteps();
                    break;
                case State::grid_in_field:
                case State::grid_home:
                    StopMotor();
                    break;
                default:
                    break;
            }
        }else
            SetOutputSignal(Output::indication_0, LOW);
    }

    void InFieldSwitchCheck(){
        if(isSignalHigh(Input::grid_in_field)){
            switch (current_state_) {
                case State::moving_in_field:
                    StopMotor();
                    ChangeDeviceState(State::grid_in_field);
                    motor_controller_.StandByModeOn();
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
        if(isSignalHigh(Input::exp_req) || isInState(State::oscillation) || isInState(State::scanning))
            ExpositionProcedure();
    }

    void CheckPendingMove(){
        if(pending_move_ && (!motor_controller_.IsMotorMoving())){
            auto callable = *pending_move_;
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

    void RasterMoveInField(MoveSpeed speed){
        if(isSignalHigh(Input::grid_in_field)){
            ChangeDeviceState(State::grid_in_field);
            StopMotor();
            return;
        }
        ChangeDeviceState(State::moving_in_field);
        speed == MoveSpeed::slow ? motor_controller_.MoveToEndPointSlow(Dir::FORWARD) :
                                   motor_controller_.MoveToEndPointFast(Dir::FORWARD);
    }

    void RasterMoveHome(MoveSpeed speed){
        if(isSignalHigh(Input::grid_home)){
            ChangeDeviceState(State::grid_home);
            StopMotor();
            return;
        }
        ChangeDeviceState(State::moving_home);
        speed == MoveSpeed::slow ? motor_controller_.MoveToEndPointSlow(Dir::BACKWARDS) :
                                   motor_controller_.MoveToEndPointFast(Dir::BACKWARDS);
    }

    void FreezeSwitchCheck(uint16_t delay = 300){
        if(isInState(State::service_moving))
            return;
        current_tim_task_ = TimTask::freeze_switch;
        switch_ignore_flag_ = true;
        StartTaskTimIT(delay);
    }

    void RasterMoveInFieldAfterExpo(){
        SlowStopMotor();
        pending_move_ = [&]{
            motor_controller_.MoveToPos(Dir::BACKWARDS, RUN_OUT_STEPS);
            pending_move_ = [&]{ RasterMoveInField(MoveSpeed::slow); };
        };
    }

    void RasterMoveHomeAfterExpo(){
        SlowStopMotor();
        pending_move_ = [&]{
            motor_controller_.MoveToPos(Dir::FORWARD, RUN_OUT_STEPS);
            pending_move_ = [&]{ RasterMoveHome(MoveSpeed::slow); };
        };
    }

    void ExpRequestedOnHoneGrid(){
        ChangeDeviceState(State::error);
        currentError_ = Error::exp_req_error;
    }

    void SetInMotionSigWithDelay(){
        current_tim_task_ = TimTask::in_motion_delay;
        StartTaskTimIT(IN_MOTION_mSec_DELAY);
    }

    void SetInMotionSig(logic_level level){
        SetOutputSignal(Output::in_motion, level);
    }

    void StartOscillation(){
        ChangeDeviceState(State::service_moving);
        motor_controller_.MoveToPos(Dir::FORWARD, EXPO_OFFSET_STEPS);
        pending_move_ = [&]{
            lastPosition_ = State::grid_in_field;
            ChangeDeviceState(State::oscillation);
            motor_controller_.Exposition();
        };
    }

    bool IsInMotionSigReady(){
//        auto dir = motor_controller_.CurrentDirection() == StepperMotor::Direction::BACKWARDS;
        auto event = motor_controller_.GetEvent() == StepperMotor::EVENT_CSS;
        auto in_time = motor_controller_.TimeOfAccelPhase() >= IN_MOTION_uSec_DELAY;
//        return in_mode && in_time && dir;
        return event && in_time;
    }

    void ExpositionProcedure(){
        if(isSignalHigh(Input::exp_req)){
            switch (current_state_) {
                case State::grid_in_field:
                    lastPosition_ = current_state_;
                    if(!oscillation_enabled_){
                        ChangeDeviceState(State::scanning);
                        SetInMotionSigWithDelay();
                    }else
                        StartOscillation();
                    break;

                case State::grid_home:
                    if(!kRasterHomeExpReqIsOk_)
                        ExpRequestedOnHoneGrid();
                    else{
                        lastPosition_ = current_state_;
                        ChangeDeviceState(State::scanning);
                        SetInMotionSigWithDelay();
                    }
                    break;

                case State::oscillation:
                    if(IsInMotionSigReady())
                        SetInMotionSig(HIGH);
                    break;
                default:
                    break;
            }
        }
        else
            FinishExpoProcedure();
    }

    void FinishExpoProcedure(){
        SetInMotionSig(LOW);
        switch (lastPosition_){
            case State::grid_home:
                if(oscillation_enabled_)
                    RasterMoveHome(MoveSpeed::slow);
                else
                    ChangeDeviceState(State::grid_home);
                break;
            case State::grid_in_field:
                if(oscillation_enabled_)
                    RasterMoveInFieldAfterExpo();
                else
                    ChangeDeviceState(State::grid_in_field);
                break;
            default:
                break;
        }
    }

    void TimTaskHandler(){
        switch (current_tim_task_) {
            case TimTask::freeze_switch:
                UnFreezeSwitches();
                break;
            case TimTask::in_motion_delay:
                SetInMotionSig(HIGH);
                break;
            default:
                break;
        }
        current_tim_task_ = TimTask::no_task;
    }

    void UnFreezeSwitches(){
        switch_ignore_flag_ = false;
    }

private:
    explicit MainController(MotorController &incomeMotorController)
        :motor_controller_(incomeMotorController)
    {}

    static constexpr int kIN_PIN_CNT = 3;
    std::array<InputPin, kIN_PIN_CNT> input_pins_{
            InputPin(EXP_REQ_IN_GPIO_Port, EXP_REQ_IN_Pin),
            InputPin(GRID_INFIELD_DETECT_GPIO_Port, GRID_INFIELD_DETECT_Pin),
            InputPin(GRID_HOME_DETECT_GPIO_Port, GRID_HOME_DETECT_Pin),
    };
    static constexpr int kOUT_PIN_CNT = 3;
    std::array<OutputPin, kOUT_PIN_CNT> output_pins_{
            OutputPin(INDICATION_0_OUT_GPIO_Port, INDICATION_0_OUT_Pin),
            OutputPin(INDICATION_1_OUT_GPIO_Port, INDICATION_1_OUT_Pin),
            OutputPin(IN_MOTION_OUT_GPIO_Port, IN_MOTION_OUT_Pin)
    };
    InputPin test_btn {NOTUSED_PUSHBUTTON_GPIO_Port, NOTUSED_PUSHBUTTON_Pin};
//    InputSignal t_btn {InputPin{NOTUSED_PUSHBUTTON_GPIO_Port, NOTUSED_PUSHBUTTON_Pin}, 2};

    MotorController& motor_controller_;
    TimTask current_tim_task_ {TimTask::no_task};
    Error currentError_ {Error::no_error};
    State current_state_ {State::init_state};
    State lastPosition_ {State::grid_in_field};

    bool oscillation_enabled_ {false};
    bool switch_ignore_flag_ {false};
    const bool kRasterHomeExpReqIsOk_ {true};

    std::optional<std::function<void()>> pending_move_;

    void ErrorHandler_(Error error){
        StopMotor();
        SetOutputSignal(Output::indication_1, HIGH);
        switch (error) {
            case Error::initial_movement_error:
            case Error::limit_switch_error:
                break;
            case Error::exp_req_error:
                return;
            default:
                break;
        }
        Error_Handler();
    }
};