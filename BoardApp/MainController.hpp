
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

    static MainController& getRef(){
        static auto self = MainController(MotorController::getRef());
        return self;
    }

    void BoardInit(){
        input_pin_container_[EXP_REQ].setInverted();
        input_pin_container_[GRID_HOME_DETECT].setInverted();
        input_pin_container_[GRID_INFIELD_DETECT].setInverted();

        currentStatus = DEVICE_MOVING;
        motor_controller_.get_position(Dir::FORWARD);
        HAL_Delay(300);
   }

    void BtnEventHandle(BTN_TYPE btnT, LOGIC_LEVEL value){
        switch (btnT) {
            case GRID_BUTTON:
                if(currentStatus == DEVICE_GRID_IN_FIELD) RasterUnloadProcedure();
                else if(currentStatus == DEVICE_GRID_HOME) RasterLoadProcedure();
                break;
            default:
                break;
        }
    }

    void BoardUpdate(){
        for(auto &pin: input_pin_container_) pin.refresh();
        
        if(*grid_center_){
            if(currentStatus == DEVICE_RETURN_TO_INITIAL_STATE){
                motor_controller_.stopMotor();
                currentStatus = DEVICE_STANDBY;
            }
            else if(currentStatus == DEVICE_INITIAL_MOVEMENT || currentStatus == DEVICE_GRID_SUPPLY){
                if(!motor_controller_.noReturnMode()) motor_controller_.stopMotor();
                RasterCheck();
                currentStatus = DEVICE_STANDBY;
            }
        }
        
        else if(*bucky_call_ || exposition_ON_) ExpositionProcedure();

        if(motor_controller_.getMode() == MotorStatus::in_ERROR) currentError = LIMIT_SWITCH_ERROR;

        if(currentError != NO_ERROR) errorHandler(currentError);
    }

    void RasterLoadProcedure(){
        motor_controller_.get_position(Dir::BACKWARDS);
    }

    void RasterUnloadProcedure(){
        currentStatus = DEVICE_MOVING;
        motor_controller_.get_position(Dir::FORWARD, true);
        output_pin_container_[INDICATION_0].setValue(LOW);
    }

    void ReturnHome(){
        if(currentStatus != DEVICE_GRID_HOME){
            motor_controller_.stopMotor();
            currentStatus = DEVICE_MOVING;
            motor_controller_.get_position(Dir::BACKWARDS);
        }
    }

    void ReturnInField(){
        if(currentStatus != DEVICE_GRID_IN_FIELD){
            currentStatus = DEVICE_MOVING;
            motor_controller_.stopMotor();
            motor_controller_.get_position(Dir::BACKWARDS);
        }
    }

    inline void ExpositionProcedure(){
        if(!*exp_req_){
            exposition_ON_ = false;
            output_pin_container_[IN_MOTION].setValue(LOW);
            if(no_raster_) currentStatus = DEVICE_STANDBY;
            else ReturnToCenter();
            return;
        }
        switch (currentStatus) {
            case DEVICE_STANDBY:
                exposition_ON_ = true;
                if(*bucky_call_)
                    currentStatus = *on_tomo_ ? DEVICE_SCANING_TOMO_ON : DEVICE_SCANING_TOMO_OFF;
                if(!no_raster_)
                    motor_controller_.exposition();
                break;
            case DEVICE_SCANING_TOMO_OFF:
                if(!no_raster_)
                    output_pin_container_[BUCKY_READY].setValue(HIGH);
                else if(motor_controller_.getEvent() == EVENT_CSS)
                    output_pin_container_[BUCKY_READY].setValue(HIGH);
                break;
            case DEVICE_SCANING_TOMO_ON:
                if(!*on_tomo_ && !tomo_signal_){
                    tomo_signal_ = true;
                    output_pin_container_[BUCKY_READY].setValue(HIGH);
                }
                else if(*on_tomo_ && tomo_signal_){
                    tomo_signal_ = false;
                    output_pin_container_[BUCKY_READY].setValue(LOW);
                }
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
    const LOGIC_LEVEL* grip_home_ = &input_pin_container_[GRID_HOME_DETECT].currentState;
    const LOGIC_LEVEL* grid_in_filed_ = &input_pin_container_[GRID_INFIELD_DETECT].currentState;

    const Button btn_grid = Button(GRID_BUTTON);

    MotorController& motor_controller_;

    BOARD_STATUS_ERROR currentError = NO_ERROR;
    volatile BOARD_STATUS currentStatus = DEVICE_MOVING;
    bool exposition_ON_ = false;
    bool tomo_signal_ = false;

    [[noreturn]] void errorHandler(BOARD_STATUS_ERROR error){
        motor_controller_.stopMotor();
        currentStatus = DEVICE_ERROR;
        switch (error) {
            case STANDBY_MOVEMENT_ERROR:
                break;
            case LIMIT_SWITCH_ERROR:
                break;
            case ON_TOMO_EXP_REQ_ERROR:
                break;
            default:
                break;
        }
        Error_Handler();
        while (true){};
    }
};

#endif //RASTERDRIVER_MAINCONTROLLER_HPP