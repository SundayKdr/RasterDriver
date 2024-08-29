// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
#include "main.h"
#include "tim.h"
#include "iwdg.h"
#include "controller.hpp"

extern "C"
{
//    void HAL_IncTick() {
//        uwTick += uwTickFreq;
//        MainController::global().SysTickTimersTickHandler();
//    }

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
    {
        if(htim->Instance == TIM1){
            HAL_IWDG_Refresh(&hiwdg);
            MainController::global().BoardUpdate();
        }
        if(htim->Instance == TIM3){
            MainController::global().UpdateConfig();
        }
        if(htim->Instance == TIM6){
            HAL_TIM_Base_Stop_IT(htim);
            MainController::global().TimTaskHandler();
        }
        if(htim->Instance == TIM7){
            HAL_TIM_Base_Stop_IT(&htim7);
            MainController::global().BtnEventHandle();
        }
    }

    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
    {
        if(htim->Instance == TIM4){
            MotorController::global().MotorRefresh();
        }
    }

    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        if(GPIO_Pin == GRID_BUTTON_Pin){
            HAL_GPIO_ReadPin(GRID_BUTTON_GPIO_Port, GRID_BUTTON_Pin) ? HAL_TIM_Base_Start_IT(&htim7)
                                                                     : HAL_TIM_Base_Stop_IT(&htim7);
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
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    }

    void EnableTimFreezeInBreakpoint(){
        __HAL_DBGMCU_FREEZE_TIM4();
    }

    void AppInit(){
        EnableTimFreezeInBreakpoint();
        EXTI_clear_enable();
        TIM_IT_clear_();
        HAL_TIM_Base_Start_IT(&htim1);
        HAL_TIM_Base_Start_IT(&htim3);
        MainController::global().BoardInit();
    }

    void AppLoop()
    {}
}