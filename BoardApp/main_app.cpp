#include "main.h"
#include "tim.h"
#include "iwdg.h"
#include "main_controller.hpp"
#include "app_config.hpp"

extern "C"
{
//    void HAL_IncTick() {
//        uwTick += uwTickFreq;
//        MainController::GetRef().SysTickTimersTickHandler();
//    }

    Button btn_grid_ = Button(GRID_BUTTON_GPIO_Port, GRID_BUTTON_Pin);

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
    {
        if(htim->Instance == TIM1){
            HAL_IWDG_Refresh(&hiwdg);
            MainController::GetRef().BoardUpdate();
        }
        if(htim->Instance == TIM3){
            MainController::GetRef().UpdateConfig();
        }
        if(htim->Instance == TIM6){
            HAL_TIM_Base_Stop_IT(htim);
            MainController::GetRef().TimTaskHandler();
        }
        if(htim->Instance == TIM7){
            HAL_TIM_Base_Stop_IT(&htim7);
            MainController::GetRef().BtnEventHandle(btn_grid_);
        }
    }

    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
    {
        if(htim->Instance == TIM4){
            MotorController::GetRef().MotorRefresh();
        }
    }

    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
//        if(GPIO_Pin == btn_grid_){
//            MainController::GetRef().BtnEventHandle(btn_grid_);
//        }
        if(GPIO_Pin == btn_grid_){
            btn_grid_.getState() ? HAL_TIM_Base_Start_IT(&htim7) : HAL_TIM_Base_Stop_IT(&htim7);
        }
    }

    void EXTI_clear_enable(){
        __HAL_GPIO_EXTI_CLEAR_IT(GRID_BUTTON_Pin);
        NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    }

    void TIM_IT_clear_(){
        __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
        __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    }

    void AppInit(){
        EXTI_clear_enable();
        TIM_IT_clear_();
        HAL_TIM_Base_Start_IT(&htim1);
        HAL_TIM_Base_Start_IT(&htim3);
        MainController::GetRef().BoardInit();
    }

    void AppLoop()
    {}
}