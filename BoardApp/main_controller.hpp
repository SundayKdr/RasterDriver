
#ifndef RASTERDRIVER_MAIN_CONTROLLER_HPP
#define RASTERDRIVER_MAIN_CONTROLLER_HPP

#include <array>
#include "StepperMotor/StepperMotor.hpp"
#include "IO/PIN.hpp"
#include "IO/Button.hpp"
#include "global_.hpp"
#include "app_timer.hpp"

using namespace RB::types;
using namespace StepperMotor;
using namespace pin_impl;

class MainController {
    using InputPinType = PIN<PinReadable>;
    using OutputPinType = PIN<PinWriteable>;
    using Dir = MotorController::Direction;
    using MotorStatus = MotorController::Mode;
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

    void BoardInit(){
        MotorController::LoadDriver(DIPSwitches_configureDriver());
        input_pin_container_[EXP_REQ].setInverted();
        RasterMoveHome();
    }

    bool isInState(BOARD_STATUS status){
       return currentState_ == status;
   }

    static bool isSignalHigh(const LOGIC_LEVEL* signal){
        return *signal;
    }

    void ChangeDeviceState(BOARD_STATUS new_status){
       currentState_ = new_status;
    }

    void StopMotor(){
        motor_controller_.StopMotor();
    }

    void ChangeMotorDir(){
        FreezeSwitchCheck();
        motor_controller_.ChangeDirection();
    }

    void ChangeMotorDirAbnormal(){
        FreezeSwitchCheck();
        motor_controller_.StepsCorrectionHack();
    }

    void CorrectExpoSteps(){
        motor_controller_.EndSideStepsCorr();
    }

    void SetOutputSignal(OUTPUT_TYPE sigType, LOGIC_LEVEL level){
        if(output_pin_container_.size() < static_cast<int>(sigType))
            return;
        output_pin_container_[sigType].setValue(level);
    }

    void BtnEventHandle(Button& btn){
        if(btn.getState()){
            if(isInState(DEVICE_GRID_IN_FIELD))
                StartShakeExposition();
            else if((isInState(DEVICE_SHAKE_SCANNING) || isInState(DEVICE_GRID_HOME))){
                SetRasterInMotionSignal(LOW);
                RasterMoveInField();
            }
        }
    }

    void BtnEventHandle(BTN_TYPE btnT, LOGIC_LEVEL value){
        switch (btnT) {
            case GRID_BUTTON:
//                if(value && isInState(DEVICE_GRID_IN_FIELD))
//                    RasterMoveHome();
//                else if(value && isInState(DEVICE_GRID_HOME))
//                    RasterMoveInField();
                if(value && (isInState(DEVICE_GRID_IN_FIELD)))
                    StartShakeExposition();
                else if(value && (isInState(DEVICE_SHAKE_SCANNING) || isInState(DEVICE_GRID_HOME))){
                    SetRasterInMotionSignal(LOW);
                    RasterMoveInField();
                }
                break;
            default:
                break;
        }
    }

    void ErrorsCheck(){
        if(motor_controller_.GetMode() == MotorStatus::in_ERROR)
            currentError_ = LIMIT_SWITCH_ERROR;
        if(currentError_ != NO_ERROR)
            ErrorHandler_(currentError_);
    }

    void HomeSwitchCheck(){
        if(isSignalHigh(grid_home_sig_)){
            switch (currentState_) {
                case DEVICE_SERVICE_MOVING:
                    StopMotor();
                    ChangeDeviceState(DEVICE_GRID_HOME);
                    break;
                case DEVICE_SHAKE_SCANNING:
                    ChangeMotorDirAbnormal();
                    break;
                case DEVICE_GRID_IN_FIELD:
                case DEVICE_GRID_HOME:
                    StopMotor();
                case DEVICE_SCANNING:
                case DEVICE_ERROR:
                case DEVICE_INIT_STATE:
                    break;
            }
        }
    }

    void InFieldSwitchCheck(){
        if(isSignalHigh(grid_in_filed_sig_)){
            SetOutputSignal(INDICATION_0, HIGH);
            switch (currentState_) {
                case DEVICE_SERVICE_MOVING:
                    StopMotor();
                    ChangeDeviceState(DEVICE_GRID_IN_FIELD);
                    break;
                case DEVICE_SHAKE_SCANNING:
                    ChangeMotorDirAbnormal();
                    break;
                case DEVICE_GRID_IN_FIELD:
                case DEVICE_INIT_STATE:
                case DEVICE_GRID_HOME:
                case DEVICE_SCANNING:
                case DEVICE_ERROR:
                    break;
            }
        }else
            SetOutputSignal(INDICATION_0, LOW);
    }

    void AnySwitchActiveCheck(){
        if(switch_ignore_flag_)
            return;
        HomeSwitchCheck();
        InFieldSwitchCheck();
    }

    void ExpStateCheck(){
        if(isSignalHigh(exp_req_sig_) || isInState(DEVICE_SHAKE_SCANNING) || isInState(DEVICE_SCANNING))
            ExpositionProcedure();
    }

    void UpdateInputSignalLevels(){
        for(auto &pin: input_pin_container_)
            pin.refresh();
    }

    void BoardUpdate(){
        UpdateInputSignalLevels();
        ErrorsCheck();
        AnySwitchActiveCheck();
        ExpStateCheck();
    }

    void RasterMoveInField(){
        if(isSignalHigh(grid_in_filed_sig_)){
            ChangeDeviceState(DEVICE_GRID_IN_FIELD);
            StopMotor();
            return;
        }
        FreezeSwitchCheck();
        ChangeDeviceState(DEVICE_SERVICE_MOVING);
        motor_controller_.get_position(Dir::FORWARD, true);
    }

    void FreezeSwitchCheck(){
        if(isInState(DEVICE_SERVICE_MOVING))
            return;
        switch_ignore_flag_ = true;
        StartFreezeTimIT(170);
    }

    void RasterMoveHome(){
        if(isSignalHigh(grid_home_sig_)){
            ChangeDeviceState(DEVICE_GRID_HOME);
            StopMotor();
            return;
        }
        FreezeSwitchCheck();
        ChangeDeviceState(DEVICE_SERVICE_MOVING);
        motor_controller_.get_position(Dir::BACKWARDS, true);
    }

    void ExpRequestedOnHoneGrid(){
        ChangeDeviceState(DEVICE_ERROR);
        currentError_ = EXP_REQ_ERROR;
    }

    void SetRasterInMotionSignal(LOGIC_LEVEL level){
//        FreezeDeviceDelay(0);
        //todo change
        SetOutputSignal(INDICATION_1, level);
//        SetOutputSignal(IN_MOTION, level);
    }

    void StartShakeExposition(){
        lastPosition_ = currentState_;
        ChangeDeviceState(DEVICE_SHAKE_SCANNING);
        FreezeSwitchCheck();
        motor_controller_.Exposition();
        motor_controller_.StepsCorrectionHack();
    }

    inline void ExpositionProcedure(){
//        if(!*exp_req_sig_){
//            SetRasterInMotionSignal(LOW);
//            switch (lastPosition_) {
//                case DEVICE_GRID_HOME:
//                    RasterMoveHome();
//                    break;
//                case DEVICE_GRID_IN_FIELD:
//                    RasterMoveInField();
//                    break;
//                default:
//                    break;
//            }
//            return;
//        }
        switch (currentState_) {
            case DEVICE_GRID_IN_FIELD:
                lastPosition_ = currentState_;
                if(!kShakingScanEnabled_){
                    ChangeDeviceState(DEVICE_SCANNING);
                    SetRasterInMotionSignal(HIGH);
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
                SetRasterInMotionSignal(HIGH);
                ChangeDeviceState(DEVICE_SCANNING);
                break;
            case DEVICE_SHAKE_SCANNING:
                if(motor_controller_.GetEvent() == StepperMotor::EVENT_CSS)
                    SetRasterInMotionSignal(HIGH);
                break;
            default:
                break;
        }
    }

    void UnFreezeSwitches() {
        switch_ignore_flag_ = false;
    }

    void ProcessMessage(){
    }

    void TimersTickHandler(){
        for(auto & timer : timers_)
            timer->TickHandle();
    }

private:
    explicit MainController(MotorController &incomeMotorController)
            :motor_controller_(incomeMotorController)
    {}

    AppTimer msgReqTim1{[this](){ProcessMessage();}};
    AppTimer msgReqTim2{[this](){ProcessMessage();}};

    std::array<AppTimer*, 2> timers_{
        &msgReqTim1,
        &msgReqTim2
    };

    static constexpr int kIN_PIN_CNT = 3;
    std::array<InputPinType, kIN_PIN_CNT> input_pin_container_{
            InputPinType(EXP_REQ, EXP_REQ_IN_GPIO_Port, EXP_REQ_IN_Pin),
            InputPinType(GRID_HOME_DETECT, GRID_HOME_DETECT_GPIO_Port, GRID_HOME_DETECT_Pin),
            InputPinType(GRID_INFIELD_DETECT, GRID_INFIELD_DETECT_GPIO_Port,
                         GRID_INFIELD_DETECT_Pin),
    };
    static constexpr int kOUT_PIN_CNT = 3;
    std::array<OutputPinType, kOUT_PIN_CNT> output_pin_container_{
            OutputPinType(INDICATION_0, INDICATION_0_OUT_GPIO_Port, INDICATION_0_OUT_Pin),
            OutputPinType(INDICATION_1, INDICATION_1_OUT_GPIO_Port, INDICATION_1_OUT_Pin),
            OutputPinType(IN_MOTION, IN_MOTION_OUT_GPIO_Port, IN_MOTION_OUT_Pin)
    };

    const LOGIC_LEVEL* exp_req_sig_ = input_pin_container_[EXP_REQ].GetPinStatePtr();
    const LOGIC_LEVEL* grid_home_sig_ = input_pin_container_[GRID_HOME_DETECT].GetPinStatePtr();
    const LOGIC_LEVEL* grid_in_filed_sig_ = input_pin_container_[GRID_INFIELD_DETECT].GetPinStatePtr();

    MotorController& motor_controller_;

    BOARD_STATUS_ERROR currentError_ = NO_ERROR;
    volatile BOARD_STATUS currentState_ = DEVICE_INIT_STATE;
    BOARD_STATUS lastPosition_ = DEVICE_SERVICE_MOVING;
    bool exposition_ON_status_ = false;
    bool tomo_signal_ = false;
    const bool kShakingScanEnabled_ = true;
    const bool kRasterHomeExpReqIsOk_ = true;

    bool switch_ignore_flag_ = false;

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
};

#endif //RASTERDRIVER_MAIN_CONTROLLER_HPP