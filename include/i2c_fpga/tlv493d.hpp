#include <stdint.h>
#include "i2c_fpga/i2c.hpp"
#include <vector>
#include <ros/ros.h>

using namespace std;

#define bitRead(byte,n) ((bool)((byte&(2^n))>>n))
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

class TLV493D{
public:
    TLV493D(void *i2c_base, vector<uint8_t> &deviceAddress, vector<int> &devicePin);
    bool initTLV(uint8_t deviceaddress, int devicepin);
    float convertToMilliTesla(uint8_t data);
    void readTLV_B_MSB(int deviceaddress, vector<uint8_t> &data);
    void readAllRegisters(int deviceaddress, vector<uint8_t> &reg);
private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
public:
    boost::shared_ptr<I2C> i2c;
    void *i2c_base;
};


