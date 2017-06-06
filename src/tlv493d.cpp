#include "i2c_fpga/tlv493d.hpp"

TLV493D::TLV493D(void *i2c_base, vector<uint8_t> &deviceAddress, vector<int> &devicePin):i2c_base(i2c_base){
    i2c = boost::shared_ptr<I2C>(new I2C(i2c_base));

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "tlv_magnetic_sensor",
                  ros::init_options::NoRosout);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

//    ROS_INFO("Checking device addresses by bruteforcing the entire range. Fuck you angry pixies!");
////    uint8_t sensor = 1;
////    for(uint i=0;i<deviceAddress.size();i++) {
////        ROS_INFO("checking sensor %d", i);
////        IOWR(i2c_base, i2c->GPIO_CONTROL, sensor);
//        for (uint8_t tmpaddr = 1; tmpaddr <= 127; tmpaddr++) {
//            uint32_t data;
//            i2c->read(tmpaddr, 1, 1);
//            if (!IORD(i2c_base, i2c->ACK_ERROR)) {
//                ROS_INFO("sensor active at: %d", tmpaddr);
//            }
//        }
////        sensor <<= 1;
////    }

//    ROS_INFO("Let's give each one its own device address");
//    for(int i=0; i<deviceAddress.size();i++){
//        initTLV(deviceAddress[i],devicePin[i]);
//    }
}

bool TLV493D::initTLV(uint8_t deviceaddress, int devicepin) {
    bool ADDR_pin;
    uint8_t IICAddr;
    uint8_t setaddr;
    uint8_t defaultaddr;
    uint8_t regdata[10];

    if (devicepin == 255){
        ADDR_pin = 1;
        IICAddr  = 0;
        setaddr  = 0b1011110;
        ROS_INFO("No device power pin selected, asuming device is on and continuing with the default address: %x .\n"
                         "Triggering general reset to make sure device is configured correctly",setaddr);
        i2c->write(0,1,1);   // Clock in address 0 to reset 'ALL' devices. And write 1,
                            // very important to keep SDA line up or else address changes to 0b00011111

    }else{
        ADDR_pin = bitRead(deviceaddress,6);
        IICAddr  = (!bitRead(deviceaddress,4)<<1)|(!bitRead(deviceaddress,2));
        setaddr  = (ADDR_pin<<6)|(!bitRead(IICAddr,1)<<4)|(1<<3)|(!bitRead(IICAddr,0)<<2)|(1<<1)|(!ADDR_pin);
    }

    if (setaddr != deviceaddress){
        ROS_ERROR("Invalid device address! Please check and try again");
//        return false;
    }else{
        ROS_INFO("Configuring device on pin %d with address: %x (ADDR_pin = %x, IICAddr = %x)", devicepin, setaddr, ADDR_pin, IICAddr);
    }

    uint tmpreg = IORD(i2c_base,i2c->GPIO_CONTROL); // Read previous pin status so as to not overwrite any values

    if (ADDR_pin != 1){
        // Unblock I2C and take control of the SDA line
        IOWR(i2c_base, i2c->GPIO_CONTROL, tmpreg|0b1000);
        ROS_INFO("Activating 'El cacharro' %d (SDA Low)", devicepin);
        // Power on device while SDA low to set ADDR bit to 0
        usleep(1);
        IOWR(i2c_base, i2c->GPIO_CONTROL, tmpreg|(1<<devicepin));
        usleep(1000);                     // At least during 200us
        // Release SDA line again
        IOWR(i2c_base, i2c->GPIO_CONTROL, tmpreg&0b0111);
        defaultaddr = 0b0011111;
    }else{
        if(devicepin != 255){
            IOWR(i2c_base, i2c->GPIO_CONTROL, tmpreg|(1<<devicepin));
            ROS_INFO("Activating 'El cacharro' %d", devicepin);
        }
        usleep(1000);
        defaultaddr = 0b1011110;
    }

    vector<uint8_t> data;
    i2c->read_continuous(defaultaddr, 10, data); // Beginning first read (for backup)
    for(uint32_t val:data){
        ROS_INFO("%x", val);
    }
//
//    // Begin config
//    // Static initial config for now
//    uint32_t cfgdata = 0;
//
//    cfgdata =  (uint8_t)((0b010<<5)|(regdata[9]&0b11111))|(regdata[8]<<8)|(uint8_t)((IICAddr<<5)|(regdata[7]&0b00011000)|0b010);  // Last 3 bits: INT/FAST/LP
//
////    // First 3 bits: Enable temp/Low power interval/Parity test
////
////    // Calculate parity bit     (well doesen't work for now so fuck it)
////    // bool parity = bitRead(cfgdata[0]+cfgdata[1]+cfgdata[2],0);
////    // printf("Setting parity bit to ");
////    // printfln(parity,BIN);
////    // bitWrite(cfgdata[0],7,parity);
////
////    // Write config
//    printf("Writing config now ...\n");
//    configureTLV(defaultaddr, cfgdata, sizeof(cfgdata));
////    // End config
////
//    printf("Checking new config ...\n");
//    reg = 0;
//    Wire.requestFrom(setaddr,(uint8_t) 10);
////    while(Wire.available()){
////        regdata[reg] = Wire.read();
////        reg++;
////    }
////    if ( reg != 10 ){   //Todo: Handle this as a true error and retry!
////        printf("ERROR: Data could not be read correctly and may be incomplete! Continuing, nevertheless ..."
////                       "  (reg = %d)\n",reg);
////    }

    return true;
}

void TLV493D::configureTLV(uint8_t deviceaddress, uint32_t data, int count) {
    i2c->write(deviceaddress,data,count);
}

float TLV493D::convertToMilliTesla(uint8_t data) {
    float mTs = 0;
    uint8_t bitmask = 1;
    for(int i=0; i<7; i++){
        mTs += (data&bitmask);
        bitmask <<= 1;
    }
    mTs -= (data&bitmask);
    return (mTs*1.56f);
}

void TLV493D::readTLV_B_MSB(int deviceaddress, uint8_t *data) {
    // Read the first 3 registers only. Corresponding to the 8bit MSB values of the magnetic field
//    uint32_t reg = i2c->read_continuous(deviceaddress,3);
//    data[0] = (uint8_t)(reg&0xff);
//    data[1] = (uint8_t)((reg>>8)&0xff);
//    data[2] = (uint8_t)((reg>>16)&0xff);
}
