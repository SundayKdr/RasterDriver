#include "main.h"
#include "tim.h"
#include "iwdg.h"
#include "main_controller.hpp"
#include "global_define.hpp"

extern "C"
{
    Button btn_grid_ = Button(GRID_BUTTON, GRID_BUTTON_GPIO_Port, GRID_BUTTON_Pin);

//    void HAL_IncTick() {
//        uwTick += uwTickFreq;
//        MainController::GetRef().TimersTickHandler();
//    }

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
    {
        if(htim->Instance == TIM1) {
            HAL_IWDG_Refresh(&hiwdg);
            MainController::GetRef().BoardUpdate();
        }
        if(htim->Instance == TIM6){
            MainController::GetRef().UnFreezeSwitches();
            HAL_TIM_Base_Stop_IT(htim);
        }
        if(htim->Instance == TIM7){
            MainController::GetRef().BtnEventHandle(btn_grid_);
        }
    }

    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
    {
        if(htim->Instance == TIM4){
            MotorController::GetRef().MotorRefresh();
        }
    }

//    bool btn_grid_pressed = true;
//    uint32_t time_gridBTN_pressed;
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        if(GPIO_Pin == GRID_BUTTON_Pin){
            MainController::GetRef().BtnEventHandle(btn_grid_);
//            MainController::GetRef().BtnEventHandle(GRID_BUTTON,
//                                                    static_cast<LOGIC_LEVEL>(HAL_GPIO_ReadPin(GRID_BUTTON_GPIO_Port,
//                                                                                              GRID_BUTTON_Pin)));
        }
//        switch (GPIO_Pin) {
//            case GRID_BUTTON_Pin:
//                if(btn_grid_pressed){
//                    if((HAL_GetTick() - time_gridBTN_pressed) > TIME_GRID_BTN_LONG_PRESS)
//                        mainController.BtnEventHandle(btn_grid_, HIGH);
//                }else
//                    time_gridBTN_pressed = HAL_GetTick();
//                btn_grid_pressed = !btn_grid_pressed;
//                if(btn_grid_pressed)
//                    HAL_TIM_Base_Start_IT(&htim7);
//                else
//                    HAL_TIM_Base_Stop_IT(&htim7);
//                break;
//            default:
//                break;
//        }
    }

    void EXTI_clear_enable(){
        __HAL_GPIO_EXTI_CLEAR_IT(GRID_BUTTON_Pin);
        NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    }

    void AppInit(){
        EXTI_clear_enable();
        HAL_TIM_Base_Start_IT(&htim1);
        MainController::GetRef().BoardInit();
    }

    void AppLoop()
    {
    }
}