#pragma once
#include "SinglePortModule.h"
#define DATA_BUFFER_SIZE 5

class I2CModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    
    I2CModule() : SinglePortModule("i2cread", meshtastic_PortNum_PRIVATE_APP), OSThread("I2CRead") {}
  
  protected:
    virtual int32_t runOnce() override;
  private:
    struct Reading
    {
      int16_t x;
      int16_t y;
      int16_t z;
    };
    bool firstTime = true;
    uint8_t stored_samples = 0;
    Reading data_buffer[DATA_BUFFER_SIZE]; //Buffer to store readings
    uint32_t lastAcquired = 0;
    bool isBufferFull();
    void storeSample(Reading read);
    void sendPacket();
};




extern I2CModule *i2cmodule;
