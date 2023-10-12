//
// Created by 79162 on 25.09.2021.
//

#ifndef TOMO_A4BOARD_BUTTON_HPP
#define TOMO_A4BOARD_BUTTON_HPP

#include "PIN.hpp"

using namespace pin_impl;

struct Button{
    constexpr explicit Button(GPIO_TypeDef* port, uint16_t pin) noexcept
        : pin_(port, pin)
    {
//        pin_.setInverted();
    }

    PIN<PinReadable> pin_;

    constexpr uint16_t operator()() const{
        return pin_.getPin();
    }
    constexpr bool operator==(uint16_t otherPin) const{
        return pin_.getPin() == otherPin;
    }
    constexpr LOGIC_LEVEL getState(){
        return pin_.getValue();
    }
};

#endif //TOMO_A4BOARD_BUTTON_HPP
