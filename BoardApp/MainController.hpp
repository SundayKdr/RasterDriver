
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
    explicit MainController(MotorController &incomeMotorController)
        :motor_controller_(incomeMotorController)
    {}

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
                if(value && currentStatus_ == DEVICE_GRID_IN_FIELD)
                    RasterMoveHome();
                else if(value && currentStatus_ == DEVICE_GRID_HOME)
                    RasterMoveInField();
                break;
            default:
                break;
        }
    }

    void ErrorChecks(){
        if(motor_controller_.GetMode() == MotorStatus::in_ERROR)
            currentError_ = LIMIT_SWITCH_ERROR;
        if(currentError_ != NO_ERROR)
            ErrorHandler_(currentError_);
    }

    bool SwitchesCheck(){
        if(*grid_home_){
            switch (currentStatus_) {
                case DEVICE_SERVICE_MOVING:
                    motor_controller_.StopMotor();
                    currentStatus_ = DEVICE_GRID_HOME;
                    output_pin_container_[INDICATION_0].setValue(LOW);
                    break;
                case DEVICE_SHAKE_SCANNING:
                    motor_controller_.EndSideStepsCorr();
                    break;
                case DEVICE_GRID_IN_FIELD:
                case DEVICE_GRID_HOME:
                case DEVICE_SCANNING:
                case DEVICE_ERROR:
                    break;
            }
            return false;
        }
        if(*grid_in_filed_){
            output_pin_container_[INDICATION_0].setValue(HIGH);
            switch (currentStatus_) {
                case DEVICE_SERVICE_MOVING:
                    motor_controller_.StopMotor();
                    currentStatus_ = DEVICE_GRID_IN_FIELD;
                    break;
                case DEVICE_GRID_IN_FIELD:
                case DEVICE_GRID_HOME:
                case DEVICE_SCANNING:
                case DEVICE_SHAKE_SCANNING:
                case DEVICE_ERROR:
                    break;
            }
            return false;
        }else{
            output_pin_container_[INDICATION_0].setValue(LOW);
        }

        return true;
    }

    void BoardUpdate(){
        for(auto &pin: input_pin_container_)
            pin.refresh();
        ErrorChecks();
        if(SwitchesCheck())
            ExpositionProcedure();
    }

    void RasterMoveInField(){
        if(*grid_in_filed_){
            currentStatus_ = DEVICE_GRID_IN_FIELD;
            return;
        }
        currentStatus_ = DEVICE_SERVICE_MOVING;
        motor_controller_.get_position(Dir::FORWARD, true);
        TimMSDelay(500);
    }

    void RasterMoveHome(){
        if(*grid_home_){
            currentStatus_ = DEVICE_GRID_HOME;
            return;
        }
        currentStatus_ = DEVICE_SERVICE_MOVING;
        motor_controller_.get_position(Dir::BACKWARDS, true);
        TimMSDelay(500);
    }

    void ReturnHome(){
        if(currentStatus_ != DEVICE_GRID_HOME){
            motor_controller_.StopMotor();
            currentStatus_ = DEVICE_SERVICE_MOVING;
            motor_controller_.get_position(Dir::BACKWARDS);
        }
    }

    void ReturnInField(){
        if(currentStatus_ != DEVICE_GRID_IN_FIELD){
            currentStatus_ = DEVICE_SERVICE_MOVING;
            motor_controller_.StopMotor();
            motor_controller_.get_position(Dir::BACKWARDS);
        }
    }

    void ExpReqOnHoneGrid(){
        currentStatus_ = DEVICE_ERROR;
        currentError_ = EXP_REQ_ERROR;
    }

    void SetRasterInMotionSignal(LOGIC_LEVEL level){
        TimMSDelay(0);
        output_pin_container_[IN_MOTION].setValue(level);
    }

    inline void ExpositionProcedure(){
        if(!*exp_req_){
            SetRasterInMotionSignal(LOW);
            switch (lastPosition_) {
                case DEVICE_GRID_HOME:
                    RasterMoveHome();
                    break;
                case DEVICE_GRID_IN_FIELD:
                    RasterMoveInField();
                    break;
                default:
                    break;
            }
            return;
        }
        switch (currentStatus_) {
            case DEVICE_GRID_IN_FIELD:
                lastPosition_ = currentStatus_;
                currentStatus_ = kShakingScan_ ? DEVICE_SHAKE_SCANNING : DEVICE_SCANNING;
                if(!kShakingScan_)
                    SetRasterInMotionSignal(HIGH);
                else
                    motor_controller_.Exposition();
                break;
            case DEVICE_GRID_HOME:
                if(!kRasterHomeOk_){
                    ExpReqOnHoneGrid();
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

private:
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

    const LOGIC_LEVEL* exp_req_ = &input_pin_container_[EXP_REQ].currentState;
    const LOGIC_LEVEL* grid_home_ = &input_pin_container_[GRID_HOME_DETECT].currentState;
    const LOGIC_LEVEL* grid_in_filed_ = &input_pin_container_[GRID_INFIELD_DETECT].currentState;

    const Button btn_grid = Button(GRID_BUTTON);

    MotorController& motor_controller_;

    BOARD_STATUS_ERROR currentError_ = NO_ERROR;
    volatile BOARD_STATUS currentStatus_ = DEVICE_SERVICE_MOVING;
    BOARD_STATUS lastPosition_ = DEVICE_SERVICE_MOVING;
    bool exposition_ON_ = false;
    bool tomo_signal_ = false;
    const bool kShakingScan_ = false;
    const bool kRasterHomeOk_ = true;

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