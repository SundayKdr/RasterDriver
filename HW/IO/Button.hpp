//
// Created by 79162 on 25.09.2021.
//

#ifndef TOMO_A4BOARD_BUTTON_HPP
#define TOMO_A4BOARD_BUTTON_HPP

#include "PIN.hpp"

using namespace pin_impl;

struct Button{
    constexpr explicit Button(uint16_t incomeType, GPIO_TypeDef* port, uint16_t pin) noexcept
        : type_(incomeType),
        pin_(incomeType, port, pin)
    {
        pin_.setInverted();
    }

    const uint16_t type_;
    PIN<PinReadable> pin_;

    [[nodiscard]] constexpr uint16_t getType() const noexcept
    {
        return type_;
    }
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
