
#include "app_timer.hpp"

AppTimer::AppTimer(AppTimer::HandlerT handler)
    :handler_(std::move(handler))
{
    StartTimer();
}


AppTimer::AppTimer(AppTimer::HandlerT handler, uint32_t msDelay)
    :handler_(std::move(handler)),
     interval_(msDelay)
{
    StartTimer();
}

void AppTimer::UpdateState() {
    if(count_ >= interval_){
        interval_ = 0;
        handler_();
    }
}

void AppTimer::TickHandle() {
    if(disabled_)
        return;
    count_ += KTick_freq_;
    UpdateState();
}

void AppTimer::SetMSDelay(uint32_t delay) {
    interval_ = delay;
}

void AppTimer::StopTimer() {
    disabled_ = true;
}

void AppTimer::StartTimer() {
    disabled_ = false;
}

