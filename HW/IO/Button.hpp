//
// Created by 79162 on 25.09.2021.
//

#ifndef TOMO_A4BOARD_BUTTON_HPP
#define TOMO_A4BOARD_BUTTON_HPP

#include "Global.hpp"

struct Button{
    constexpr explicit Button(BTN_TYPE incomeType) noexcept
        : type(incomeType)
    {}
    const BTN_TYPE type;
    [[nodiscard]] inline constexpr BTN_TYPE getType() const noexcept
    {return type;}
};

#endif //TOMO_A4BOARD_BUTTON_HPP
