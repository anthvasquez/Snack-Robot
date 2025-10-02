#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "sh2/sh2_hal.h"
#include "sh2/sh2.h"
#include "sh2/sh2_err.h"
#include <lgpio.h>

#include "control_hardware/i2c_sh2_api.hpp"

using namespace std;
using namespace control_hardware;


static int BNO08X_Open(sh2_Hal_t* self)
{
  i2c_hal_t* data = (i2c_hal_t*)self;
  data->i2cHandle = lgI2cOpen(1, 0x4A, 0);
  if(data->i2cHandle < 0)
  {
    return SH2_ERR;
  }

  //assert reset pin to put it in a known state

  data->isOpen = true;
  return SH2_OK;
}

static void BNO08X_Close(sh2_Hal_t *self)
{
  i2c_hal_t* data = (i2c_hal_t*)self;
  if(data->i2cHandle != 0)
  {
    auto ret = lgI2cClose(data->i2cHandle);
    if(ret == 0)
    {
      data->isOpen = false;
    }
    //if this returns a nonzero number there was a problem
  }

}

static int BNO08X_Read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
  i2c_hal_t* data = (i2c_hal_t*)self;
  lgI2cReadI2CBlockData(data->i2cHandle, data->i2c_addr, (char*)pBuffer, len);
  return 0;
}

static int BNO08X_Write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
  return 0;
}

static uint32_t getTimeUs(sh2_Hal_t *self)
{
  auto time = chrono::steady_clock::now();
  return static_cast<uint32_t>(chrono::duration_cast<chrono::microseconds>(time.time_since_epoch()).count());
}

BNO08X_Api::BNO08X_Api()
{
    sh2_Hal_t sh2_interface = {
        .open = BNO08X_Open,
        .close = BNO08X_Close,
        .read = BNO08X_Read,
        .write = BNO08X_Write,
        .getTimeUs = getTimeUs
      };
      config->sh2_hal = sh2_interface;
      config->start = chrono::steady_clock::now();
}

int BNO08X_Api::BNO08X_Configure()
{
  auto openOk = sh2_open(&(config->sh2_hal), EventCallback, nullptr);
  if(openOk != SH2_OK)
  {
    return SH2_ERR;
  }
}

int BNO08X_Api::BNO08X_Activate()
{
  return 0;
}

static void EventCallback(void* cookie, sh2_AsyncEvent_t* pEvent)
{
  cout << "I literally don't care" << endl;
}

int main()
{
  sh2_Hal_t config = {
    .open = BNO08X_Open,
    .close = BNO08X_Close,
    .read = BNO08X_Read,
    .write = BNO08X_Write,
    .getTimeUs = getTimeUs
  };

  

}
