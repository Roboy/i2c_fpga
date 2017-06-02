#include <stdint.h>
#include "i2c_fpga/i2c.hpp"
#include <vector>
#include <ros/ros.h>

using namespace std;

#define bitRead(byte,n) ((byte&(2^n))>>n)

class TLV493D{
public:
    TLV493D(void *i2c_base, vector<uint8_t> &deviceAddress, vector<int> &devicePin);
    bool initTLV(uint8_t deviceaddress, int devicepin);
    void configureTLV(uint8_t deviceaddress, uint32_t data, int count);
    float convertToMilliTesla(uint8_t data);
    void readTLV_B_MSB(int deviceaddress, uint8_t *data);
private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
public:
    boost::shared_ptr<I2C> i2c;
    void *i2c_base;
};


