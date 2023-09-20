#include "main.h"
#include "tim.h"
#include "iwdg.h"
#include "MainController.hpp"
#include "Global.hpp"

extern "C"
{
    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
    {
        if(htim->Instance == TIM1){
            HAL_IWDG_Refresh(&hiwdg);
        }
        if(htim->Instance == TIM6){
            MainController::GetRef().UnFreezeSwitches();
            HAL_TIM_Base_Stop_IT(htim);
        }
        if(htim->Instance == TIM7){
            MainController::GetRef().BtnEventHandle(GRID_BUTTON, HIGH);
        }
    }

    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
    {
        if(htim->Instance == TIM4){
//            MotorController::getRef().motor_refresh();
            MotorController::getRef().MotorMakeStep();
        }
    }

    volatile bool btn_grid_pressed = true;
    volatile uint32_t time_gridBTN_pressed;
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        switch (GPIO_Pin) {
            case GRID_BUTTON_Pin:
    //            if(btn_grid_pressed){
    //                if((HAL_GetTick() - time_gridBTN_pressed) > TIME_GRID_BTN_LONG_PRESS)
    //                    mainController.BtnEventHandle(btn_grid_, HIGH);
    //            }else time_gridBTN_pressed = HAL_GetTick();
    //            btn_grid_pressed = !btn_grid_pressed;
                MainController::GetRef().BtnEventHandle(GRID_BUTTON,
                                                        static_cast<LOGIC_LEVEL>(HAL_GPIO_ReadPin(GRID_BUTTON_GPIO_Port,
                                                                                                  GRID_BUTTON_Pin)));
//                if(btn_grid_pressed) HAL_TIM_Base_Start_IT(&htim7);
//                else HAL_TIM_Base_Stop_IT(&htim7);
                break;
            default:
                break;
        }
    }

    void EXTI_clear_enable(){
        __HAL_GPIO_EXTI_CLEAR_IT(GRID_BUTTON_Pin);
        NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    }

    StepperCfg DIPSwitches_configureDriver(){
        StepperCfg cfg;
        cfg.Vmin = START_SPEED;
        cfg.htim = &htim4;
        if(HAL_GPIO_ReadPin(CONFIG_1_GPIO_Port, CONFIG_1_Pin)){
            if(HAL_GPIO_ReadPin(CONFIG_2_GPIO_Port, CONFIG_2_Pin)){
                cfg.Vmax = CONFIG1_SPEED;
                cfg.A = CONFIG1_ACCELERATION;
            }else{
                cfg.Vmax = CONFIG2_SPEED;
                cfg.A = CONFIG2_ACCELERATION;
            }
        }else{
            if(HAL_GPIO_ReadPin(CONFIG_2_GPIO_Port, CONFIG_2_Pin)){
                cfg.Vmax = CONFIG3_SPEED;
                cfg.A = CONFIG3_ACCELERATION;
            }else{
                cfg.Vmax = CONFIG4_SPEED;
                cfg.A = CONFIG4_ACCELERATION;
            }
        }

        if(!HAL_GPIO_ReadPin(CONFIG_3_GPIO_Port, CONFIG_3_Pin)){
            cfg.directionInverted = true;
        }
        return cfg;
    }

    void appInit(){
        EXTI_clear_enable();
        MotorController::getRef().load_driver(DIPSwitches_configureDriver());
        HAL_TIM_Base_Start_IT(&htim1);
        MainController::GetRef().BoardInit();
    }

    void while_in_main_App()
    {
        MainController::GetRef().BoardUpdate();
        MotorController::getRef().MotorUpdate();
    }
}