
#ifndef RASTERDRIVER_GLOBAL_HPP
#define RASTERDRIVER_GLOBAL_HPP

#define TIME_GRID_BTN_LONG_PRESS        1000
#define TOTAL_DISTANCE_N_OF_STEPS       2140
#define EXPO_DISTANCE_N_OF_STEPS        1800
#define LOAD_UNLOAD_SPEED               1985
#define START_SPEED                     1569

#define CONFIG1_SPEED                   4535
#define CONFIG2_SPEED                   3969
#define CONFIG3_SPEED                   5003
#define CONFIG4_SPEED                   7129
#define CONFIG1_ACCELERATION            40
#define CONFIG2_ACCELERATION            32
#define CONFIG3_ACCELERATION            43
#define CONFIG4_ACCELERATION            52

namespace RB::types{

    enum OUTPUT_TYPE{
        INDICATION_0 = 0,
        INDICATION_1 = 1,
        IN_MOTION = 2,
    };

    enum INPUT_TYPE{
        EXP_REQ = 0,
        GRID_HOME_DETECT = 1,
        GRID_INFIELD_DETECT = 2,
    };

    enum BOARD_STATUS_ERROR{
        NO_ERROR,
        LIMIT_SWITCH_ERROR,
        STANDBY_MOVEMENT_ERROR,
        ON_TOMO_EXP_REQ_ERROR
    };

    enum BOARD_STATUS{
        DEVICE_MOVING,
        DEVICE_GRID_IN_FIELD,
        DEVICE_GRID_HOME,
        DEVICE_SCANNING,
        DEVICE_ERROR,
    };

    enum BTN_TYPE{
        GRID_BUTTON,
    };

    enum MOTOR_EVENT {
        EVENT_NULL = 0,
        EVENT_STOP,
        EVENT_CSS,  //	constant speed reached
        EVENT_CSE   //  constant speed end
    };
}

#endif //RASTERDRIVER_GLOBAL_HPP