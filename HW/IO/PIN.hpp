//
// Created by 79162 on 25.09.2021.
//

#ifndef TOMO_A4BOARD_IOS_HPP
#define TOMO_A4BOARD_IOS_HPP

#include <cstdint>
#include "type_traits"

enum LOGIC_LEVEL{
    LOW = 0,
    HIGH = 1,
};

struct PinReadable
{
};

struct PinWriteable
{
};

template<typename PinName, typename InterfaceType>
class PIN{
public:
    LOGIC_LEVEL currentState = LOW;
    const PinName pinName;

    template<typename T = InterfaceType>
    requires(std::is_base_of<PinReadable, T>::value)
    inline LOGIC_LEVEL getValue(){
        LOGIC_LEVEL retVal;
        if ((port->IDR & pin) != (uint32_t)LOGIC_LEVEL::LOW) retVal = LOGIC_LEVEL::HIGH;
        else retVal = LOGIC_LEVEL::LOW;
        if(inverted) return (retVal ? LOGIC_LEVEL::LOW : LOGIC_LEVEL::HIGH);
        else return retVal;
    }

    template<typename T = InterfaceType>
    requires(std::is_base_of<PinReadable, T>::value)
    inline LOGIC_LEVEL refresh(){
        currentState = getValue();
        return currentState;
    }

    template<typename T = InterfaceType>
    requires(std::is_base_of<PinWriteable, T>::value)
    inline void setValue(LOGIC_LEVEL value){
        if(inverted){
            if (value) port->BRR = (uint32_t)pin;
            else port->BSRR = (uint32_t)pin;
        }else{
            if (value) port->BSRR = (uint32_t)pin;
            else port->BRR = (uint32_t)pin;
        }
        currentState = value;
    }

    template<typename T = InterfaceType>
    requires(std::is_base_of<PinWriteable, T>::value)
    inline void togglePinState(){
        uint32_t odr = port->ODR;
        port->BSRR = ((odr & pin) << 16U) | (~odr & pin);
    }

//    PIN() = delete;
    const PIN& operator=(const PIN &) = delete;
    PIN& operator=(PIN &) = delete;

    void setInverted() {
        inverted = true;
    }

    constexpr explicit PIN(PinName incomeName, GPIO_TypeDef* incomePortPtr, uint16_t incomePin):
            pinName(incomeName),
            port(incomePortPtr),
            pin(incomePin)
    {};
protected:
private:
    GPIO_TypeDef* port;
    uint16_t pin;
    bool inverted = false;
};

#endif //TOMO_A4BOARD_IOS_HPP