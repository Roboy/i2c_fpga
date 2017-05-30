#include <stdint.h>
#include "i2c_fpga/i2c.hpp"

uint8_t* initTLV(uint8_t deviceaddress, uint8_t* data, uint8_t devicepin);
void configureTLV(uint8_t deviceaddress, uint8_t* data, int count);
float convertToMilliTesla(uint8_t data);
void readTLV_B_MSB(int deviceaddress, uint8_t *data);
