
#ifndef RASTERDRIVER_MAIN_CONTROLLER_HPP
#define RASTERDRIVER_MAIN_CONTROLLER_HPP

#include <array>
#include "IO/PIN.hpp"
#include "IO/Button.hpp"
#include "app_config.hpp"
//#include "Timer/app_timer.hpp"
#include "MotorImpl.hpp"

using namespace RB::types;
using namespace StepperMotor;
using namespace pin_impl;

class MainController {
    using InputPinType = PIN<PinReadable>;
    using OutputPinType = PIN<PinWriteable>;
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
        auto config = getDIPConfig();
        kShakingScanEnabled_ = config.shake_scan_enabled_;
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
        RasterMoveHome(true);
    }

    void StopMotor(){
        motor_controller_.StopMotor();
    }

    void ChangeMotorDirAbnormalExpo(){
        motor_controller_.StopMotor();
//        FreezeSwitchCheck();
//        motor_controller_.ChangeDirAbnormalExpo();
    }

    void CorrectExpoSteps(){
        FreezeSwitchCheck();
        motor_controller_.EndSideStepsCorr();
    }

    void BtnEventHandle(Button& btn){
//            if(isInState(DEVICE_SHAKE_SCANNING))
//                RasterMoveInField(true);
//            if(kShakingScanEnabled_ && isInState(DEVICE_GRID_IN_FIELD))
//                StartShakeExposition();
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
                case DEVICE_SERVICE_MOVING:
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
                case DEVICE_SCANNING:
                case DEVICE_ERROR:
                case DEVICE_INIT_STATE:
                    break;
            }
        }else
            SetOutputSignal(INDICATION_0, LOW);
    }

    void InFieldSwitchCheck(){
        if(isSignalHigh(GRID_INFIELD_DETECT)){
            switch (currentState_) {
                case DEVICE_SERVICE_MOVING:
                    StopMotor();
                    MoveCloserToSwitch();
                    ChangeDeviceState(DEVICE_GRID_IN_FIELD);
                    break;
                case DEVICE_SHAKE_SCANNING:
                    ChangeMotorDirAbnormalExpo();
                    break;
                case DEVICE_GRID_IN_FIELD:
                case DEVICE_INIT_STATE:
                case DEVICE_GRID_HOME:
                case DEVICE_SCANNING:
                case DEVICE_ERROR:
                    break;
            }
        }
    }

    void MoveCloserToSwitch(){
        FreezeSwitchCheck(200);
        motor_controller_.MakeStepsAfterSwitch();
    }

    void BoardUpdate(){
        ErrorsCheck();
        LimitSwitchesCheck();
        ExpStateCheck();
    }

    void RasterMoveInField(bool slow = false){
        if(isSignalHigh(GRID_INFIELD_DETECT)){
            ChangeDeviceState(DEVICE_GRID_IN_FIELD);
            StopMotor();
            return;
        }
        FreezeSwitchCheck();
        ChangeDeviceState(DEVICE_SERVICE_MOVING);
        motor_controller_.GetInFieldPosition(slow);
    }

    void RasterMoveHome(bool slow = false){
        if(isSignalHigh(GRID_HOME_DETECT)){
            ChangeDeviceState(DEVICE_GRID_HOME);
            StopMotor();
            return;
        }
        FreezeSwitchCheck();
        ChangeDeviceState(DEVICE_SERVICE_MOVING);
        motor_controller_.GetHomePosition(slow);
    }

    void ExpRequestedOnHoneGrid(){
        ChangeDeviceState(DEVICE_ERROR);
        currentError_ = EXP_REQ_ERROR;
    }

    void SetInMotionSigWithDelay(){
        current_tim_task_ = IN_MOTION_SIG_DELAY_TASK;
        StartTaskTimIT(150);
    }

    void SetInMotionSig(LOGIC_LEVEL level){
        SetOutputSignal(IN_MOTION, level);
    }

    void StartShakeExposition(){
        lastPosition_ = currentState_;
        ChangeDeviceState(DEVICE_SHAKE_SCANNING);
        FreezeSwitchCheck();
        motor_controller_.Exposition();
    }

    void ExpStateCheck(){
        if(isSignalHigh(EXP_REQ) || isInState(DEVICE_SHAKE_SCANNING) || isInState(DEVICE_SCANNING))
            ExpositionProcedure();
    }

    void ExpositionProcedure(){
        if(!isSignalHigh(EXP_REQ)){
            SetInMotionSig(LOW);
            switch (lastPosition_) {
                case DEVICE_GRID_HOME:
                    ChangeDeviceState(DEVICE_GRID_HOME);
                    if(kShakingScanEnabled_)
                        RasterMoveHome(true);
                    break;
                case DEVICE_GRID_IN_FIELD:
                    ChangeDeviceState(DEVICE_GRID_IN_FIELD);
                    if(kShakingScanEnabled_)
                        RasterMoveInField(true);
                    break;
                default:
                    break;
            }
            return;
        }
        switch (currentState_) {
            case DEVICE_GRID_IN_FIELD:
                lastPosition_ = currentState_;
                if(!kShakingScanEnabled_){
                    ChangeDeviceState(DEVICE_SCANNING);
                    SetInMotionSigWithDelay();
                }
                else
                    StartShakeExposition();
                break;
            case DEVICE_GRID_HOME:
                if(!kRasterHomeExpReqIsOk_){
                    ExpRequestedOnHoneGrid();
                    return;
                }
                lastPosition_ = currentState_;
                SetInMotionSigWithDelay();
                ChangeDeviceState(DEVICE_SCANNING);
                break;
            case DEVICE_SHAKE_SCANNING:
                if(motor_controller_.GetEvent() == StepperMotor::EVENT_CSS)
                    SetInMotionSig(HIGH);
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

//    void SysTickTimersTickHandler(){
//        for(auto & timer : timers_)
//            timer->TickHandle();
//    }
//    void ProcessMessage(){
//    }

private:
//    AppTimer msgReqTim1{[this](){ProcessMessage();}};
//    AppTimer msgReqTim2{[this](){ProcessMessage();}};
//    std::array<AppTimer*, 2> timers_{
//        &msgReqTim1,
//        &msgReqTim2
//    };

    explicit MainController(MotorController &incomeMotorController)
            :motor_controller_(incomeMotorController)
    {}

    static constexpr int kIN_PIN_CNT = 3;
    std::array<InputPinType, kIN_PIN_CNT> input_pin_container_{
            InputPinType(EXP_REQ_IN_GPIO_Port, EXP_REQ_IN_Pin),
            InputPinType(GRID_INFIELD_DETECT_GPIO_Port,
                         GRID_INFIELD_DETECT_Pin),
            InputPinType(GRID_HOME_DETECT_GPIO_Port, GRID_HOME_DETECT_Pin),
    };
    static constexpr int kOUT_PIN_CNT = 3;
    std::array<OutputPinType, kOUT_PIN_CNT> output_pin_container_{
            OutputPinType(INDICATION_0_OUT_GPIO_Port, INDICATION_0_OUT_Pin),
            OutputPinType(INDICATION_1_OUT_GPIO_Port, INDICATION_1_OUT_Pin),
            OutputPinType(IN_MOTION_OUT_GPIO_Port, IN_MOTION_OUT_Pin)
    };

    MotorController& motor_controller_;
    TIM_TASKS current_tim_task_ {NO_TASKS};
    BOARD_STATUS_ERROR currentError_ {NO_ERROR};
    BOARD_STATUS currentState_ {DEVICE_INIT_STATE};
    BOARD_STATUS lastPosition_ {DEVICE_SERVICE_MOVING};

    bool kShakingScanEnabled_ {false};
    bool switch_ignore_flag_ {false};
    const bool kRasterHomeExpReqIsOk_ {true};

    void ErrorHandler_(BOARD_STATUS_ERROR error){
        StopMotor();
        SetOutputSignal(INDICATION_1, HIGH);
        switch (error) {
            case STANDBY_MOVEMENT_ERROR:
                break;
            case LIMIT_SWITCH_ERROR:
                break;
            case EXP_REQ_ERROR:
                return;
            default:
                break;
        }
        Error_Handler();
        while (true){};
    }

    bool isInState(BOARD_STATUS status){
        return currentState_ == status;
    }

    bool isSignalHigh(INPUT_TYPE pin){
        return static_cast<bool>(input_pin_container_[pin].getValue());
    }

    void ChangeDeviceState(BOARD_STATUS new_status){
        currentState_ = new_status;
    }

    void UnFreezeSwitches(){
        switch_ignore_flag_ = false;
    }

    void FreezeSwitchCheck(uint16_t delay = 300){
        if(isInState(DEVICE_SERVICE_MOVING))
            return;
        current_tim_task_ = FREEZE_SWITCH_TASK;
        switch_ignore_flag_ = true;
        StartTaskTimIT(delay);
    }

    void SetOutputSignal(OUTPUT_TYPE sigType, LOGIC_LEVEL level){
        output_pin_container_[sigType].setValue(level);
    }
};

#endif //RASTERDRIVER_MAIN_CONTROLLER_HPP