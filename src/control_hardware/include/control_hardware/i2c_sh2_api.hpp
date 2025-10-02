#ifndef BNO08X_API_HPP
#define BNO08X_API_HPP

#include "sh2/sh2.h"
#include <chrono>

namespace control_hardware
{
    typedef struct i2c_hal_s {
        sh2_Hal_t sh2_hal;       // Must be first so we can cast (sh2_hal_t *) to (i2c_hal_t *)
        int i2cHandle;
        uint8_t i2c_addr;
        bool isOpen;
        std::chrono::steady_clock::time_point start;
    } i2c_hal_t;

    class BNO08X_Api
    {
    public:
        BNO08X_Api();
        int BNO08X_Configure();
        int BNO08X_Activate();

    private:
        int i2cHandle;
        i2c_hal_t* config;
    };
}

#endif // BNO08X_API_HPP