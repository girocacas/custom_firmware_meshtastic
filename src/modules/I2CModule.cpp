#include "I2CModule.h"
#include "MeshService.h"
#include "configuration.h"
#include "main.h"
#include <Wire.h>
#include <Throttle.h>
#include "Default.h"
#include "NodeDB.h"
#include "PowerFSM.h"

#define POLLING_INTERVAL 100
#define DELAYED_INTERVAL 1000
#define ACQUISITION_INTERVAL 60


const int SENSOR_ADDR = 0x18; 
const int SENSOR_CTRL_REG1 = 0x20;
const int SENSOR_OUT_X_L = 0x28;

I2CModule *i2cmodule;

int32_t I2CModule::runOnce()
{
    if (firstTime) {
        Wire.begin();
  
        // Initialize LIS3DH: Set CTRL_REG1 to activate the sensor
        Wire.beginTransmission(SENSOR_ADDR);
        Wire.write(SENSOR_CTRL_REG1); // CTRL_REG1 address
        Wire.write(0x57);             // 0x57 = 0b01010111 (100 Hz, normal mode, all axes enabled)
        Wire.endTransmission();
        
        // This is the first time the OSThread library has called this function, so do some setup
        firstTime = false;
        LOG_INFO("I2C Module: Initializing");

        return DELAYED_INTERVAL;
    }

    if (!Throttle::isWithinTimespanMs(lastAcquired,
                                      Default::getConfiguredOrDefaultMs(ACQUISITION_INTERVAL))) {
          int16_t X, Y, Z;

        // Request sensor data
        LOG_DEBUG("Requesting sensor data...");
        Wire.beginTransmission(SENSOR_ADDR);
        Wire.write(SENSOR_OUT_X_L | 0x80);  // Starting register for accelerometer data, with auto-increment
        Wire.endTransmission();
        Wire.requestFrom(SENSOR_ADDR, 6);   // Request 6 bytes

        LOG_DEBUG("Reading sensor data...");
        X = Wire.read() | (Wire.read() << 8);  // X-axis
        Y = Wire.read() | (Wire.read() << 8);  // Y-axis
        Z = Wire.read() | (Wire.read() << 8);  // Z-axis

        LOG_DEBUG("Sensor data -> X: %d, Y: %d, Z: %d",X,Y,Z);
        
        Reading read = {X,Y,Z};

        storeSample(read);
        lastAcquired = millis();
        if(isBufferFull()){
            //When buffer full -> send full data
            sendPacket();
        }
        return DELAYED_INTERVAL;
    }
    
    return POLLING_INTERVAL;
}

bool I2CModule::isBufferFull(){
    return (stored_samples == DATA_BUFFER_SIZE);
}

void I2CModule::storeSample(Reading read){
    if(stored_samples < DATA_BUFFER_SIZE){
        data_buffer[stored_samples] = read;
        stored_samples++;
        LOG_DEBUG("Sample added to buffer, there are now %d stored samples", stored_samples);
    }
}

void I2CModule::sendPacket()
{
    for (size_t i = 0; i < DATA_BUFFER_SIZE; i++)
    {
        char* datastr = new char[18]; //x1 y1 z1 
        // Format the reading into a temporary buffer
        sprintf(datastr, "%d %d %d", data_buffer[i].x, data_buffer[i].y, data_buffer[i].z);
        meshtastic_MeshPacket *p = allocDataPacket(); // Allocate a packet for sending
        p->want_ack = false;
        p->decoded.payload.size = strlen(datastr); // You must specify how many bytes are in the reply
        memcpy(p->decoded.payload.bytes, datastr, p->decoded.payload.size);
        LOG_INFO("Sending message id=%d, dest=%x, msg=%.*s", p->id, p->to, p->decoded.payload.size, p->decoded.payload.bytes);    
        service->sendToMesh(p); 
    }
    stored_samples = 0;
}
