
#ifndef RASTERDRIVER_MAINCONTROLLER_HPP
#define RASTERDRIVER_MAINCONTROLLER_HPP

#include <array>
#include "StepperMotor/StepperMotor.hpp"
#include "IO/PIN.hpp"
#include "IO/Button.hpp"
#include "Global.hpp"

using namespace RB::types;

class MainController {
    using InputPinType = PIN<RB::types::INPUT_TYPE, PinReadable>;
    using OutputPinType = PIN<RB::types::OUTPUT_TYPE, PinWriteable>;
    using Dir = MotorController::Direction;
    using MotorStatus = MotorController::Mode;
public:
    const MainController& operator=(const MainController &) = delete;
    MainController& operator=(MainController &) = delete;
    MainController() = delete;
    MainController(MainController&) = delete;
    MainController(MainController&&)= delete;

    static MainController& GetRef(){
        static auto self = MainController(MotorController::getRef());
        return self;
    }

    void BoardInit(){
        input_pin_container_[EXP_REQ].setInverted();
        RasterMoveHome();
   }

    void BtnEventHandle(BTN_TYPE btnT, LOGIC_LEVEL value){
        switch (btnT) {
            case GRID_BUTTON:
//                if(value && currentStatus_ == DEVICE_GRID_IN_FIELD)
//                    RasterMoveHome();
//                else if(value && currentStatus_ == DEVICE_GRID_HOME)
//                    RasterMoveInField();
                if(value && (currentStatus_ == DEVICE_GRID_IN_FIELD))
                    StartShakeExposition();
                else if(value && (currentStatus_ == DEVICE_SHAKE_SCANNING || currentStatus_ == DEVICE_GRID_HOME))
                    RasterMoveInField();
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
        if(*grid_home_sig_){
            switch (currentStatus_) {
                case DEVICE_SERVICE_MOVING:
                    motor_controller_.StopMotor();
                    currentStatus_ = DEVICE_GRID_HOME;
                    break;
                case DEVICE_SHAKE_SCANNING:
                    motor_controller_.EndSideStepsCorr();
                    break;
                case DEVICE_GRID_IN_FIELD:
                case DEVICE_GRID_HOME:
                    motor_controller_.StopMotor();
                case DEVICE_SCANNING:
                case DEVICE_ERROR:
                    break;
            }
        }
    }

    void InFieldSwitchCheck(){
        if(*grid_in_filed_sig_){
            output_pin_container_[INDICATION_0].setValue(HIGH);
            switch (currentStatus_) {
                case DEVICE_SERVICE_MOVING:
                    motor_controller_.StopMotor();
                    currentStatus_ = DEVICE_GRID_IN_FIELD;
                    break;
                case DEVICE_SHAKE_SCANNING:
                    motor_controller_.ChangeDirection();
                    break;
                case DEVICE_GRID_IN_FIELD:
                case DEVICE_INIT_STATE:
                case DEVICE_GRID_HOME:
                case DEVICE_SCANNING:
                case DEVICE_ERROR:
                    break;
            }
        }else
            output_pin_container_[INDICATION_0].setValue(LOW);
    }

    void AnySwitchActiveCheck(){
        if(switch_ignore_flag_)
            return;
        HomeSwitchCheck();
        InFieldSwitchCheck();
    }

    void ExpStateCheck(){
        if(*exp_req_sig_ || currentStatus_ == DEVICE_SHAKE_SCANNING || currentStatus_ == DEVICE_SCANNING)
            ExpositionProcedure();
    }

    void BoardUpdate(){
        for(auto &pin: input_pin_container_)
            pin.refresh();
        ErrorsCheck();
        AnySwitchActiveCheck();
        ExpStateCheck();
    }

    void RasterMoveInField(){
        if(*grid_in_filed_sig_){
            currentStatus_ = DEVICE_GRID_IN_FIELD;
            motor_controller_.StopMotor();
            return;
        }
        FreezeSwitchCheck();
        currentStatus_ = DEVICE_SERVICE_MOVING;
        motor_controller_.get_position(Dir::FORWARD, true);
    }

    void FreezeSwitchCheck(){
        if(currentStatus_ == DEVICE_SERVICE_MOVING)
            return;
        switch_ignore_flag_ = true;
        StartFreezeTimIT(170);
    }

    void RasterMoveHome(){
        if(*grid_home_sig_){
            currentStatus_ = DEVICE_GRID_HOME;
            motor_controller_.StopMotor();
            return;
        }
        FreezeSwitchCheck();
        currentStatus_ = DEVICE_SERVICE_MOVING;
        motor_controller_.get_position(Dir::BACKWARDS, true);
    }

    void ExpRequestedOnHoneGrid(){
        currentStatus_ = DEVICE_ERROR;
        currentError_ = EXP_REQ_ERROR;
    }

    void SetRasterInMotionSignal(LOGIC_LEVEL level){
//        FreezeDeviceDelay(0);
        output_pin_container_[IN_MOTION].setValue(level);
    }

    void StartShakeExposition(){
        lastPosition_ = currentStatus_;
        currentStatus_ = DEVICE_SHAKE_SCANNING;
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
        switch (currentStatus_) {
            case DEVICE_GRID_IN_FIELD:
                lastPosition_ = currentStatus_;
                if(!kShakingScanEnabled_){
                    currentStatus_ = DEVICE_SCANNING;
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
                lastPosition_ = currentStatus_;
                SetRasterInMotionSignal(HIGH);
                currentStatus_ = DEVICE_SCANNING;
                break;
            case DEVICE_SHAKE_SCANNING:
                if(motor_controller_.GetEvent() == EVENT_CSS)
                    SetRasterInMotionSignal(HIGH);
                break;
            default:
                break;
        }
    }

    void UnFreezeSwitches() {
        switch_ignore_flag_ = false;
    }

private:
    explicit MainController(MotorController &incomeMotorController)
            :motor_controller_(incomeMotorController)
    {}

    inline static constexpr int kPIN_CNT = 3;
    std::array<InputPinType, kPIN_CNT> input_pin_container_{
            InputPinType(EXP_REQ, EXP_REQ_IN_GPIO_Port, EXP_REQ_IN_Pin),
            InputPinType(GRID_HOME_DETECT, GRID_HOME_DETECT_GPIO_Port, GRID_HOME_DETECT_Pin),
            InputPinType(GRID_INFIELD_DETECT, GRID_INFIELD_DETECT_GPIO_Port,
                         GRID_INFIELD_DETECT_Pin),
    };
    std::array<OutputPinType, kPIN_CNT> output_pin_container_{
            OutputPinType(INDICATION_0, INDICATION_0_OUT_GPIO_Port, INDICATION_0_OUT_Pin),
            OutputPinType(INDICATION_1, INDICATION_1_OUT_GPIO_Port, INDICATION_1_OUT_Pin),
            OutputPinType(IN_MOTION, IN_MOTION_OUT_GPIO_Port, IN_MOTION_OUT_Pin)
    };

    const LOGIC_LEVEL* exp_req_sig_ = &input_pin_container_[EXP_REQ].currentState;
    const LOGIC_LEVEL* grid_home_sig_ = &input_pin_container_[GRID_HOME_DETECT].currentState;
    const LOGIC_LEVEL* grid_in_filed_sig_ = &input_pin_container_[GRID_INFIELD_DETECT].currentState;

    const Button btn_grid_ = Button(GRID_BUTTON);

    MotorController& motor_controller_;

    BOARD_STATUS_ERROR currentError_ = NO_ERROR;
    volatile BOARD_STATUS currentStatus_ = DEVICE_INIT_STATE;
    BOARD_STATUS lastPosition_ = DEVICE_SERVICE_MOVING;
    bool exposition_ON_status_ = false;
    bool tomo_signal_ = false;
    const bool kShakingScanEnabled_ = true;
    const bool kRasterHomeExpReqIsOk_ = true;

    bool switch_ignore_flag_ = false;

    void ErrorHandler_(BOARD_STATUS_ERROR error){
        motor_controller_.StopMotor();
        output_pin_container_[INDICATION_1].setValue(HIGH);
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

#endif //RASTERDRIVER_MAINCONTROLLER_HPP