
#ifndef RASTERDRIVER_APP_TIMER_HPP
#define RASTERDRIVER_APP_TIMER_HPP

#include "stm32g4xx_hal.h"
#include "functional"

class AppTimer {
public:
    using HandlerT = std::function<void()>;

    AppTimer() = delete;
    explicit AppTimer(HandlerT);
    explicit AppTimer(HandlerT, uint32_t);
    void TickHandle();
    void SetMSDelay(uint32_t delay);
    void StopTimer();
    void StartTimer();
private:
    __IO uint32_t count_{0};
    __IO uint32_t interval_{0};
    const uint32_t KTick_freq_ = HAL_TICK_FREQ_DEFAULT;
    bool disabled_ {true};
    const HandlerT handler_;
    void UpdateState();
};

#endif //RASTERDRIVER_APP_TIMER_HPP
