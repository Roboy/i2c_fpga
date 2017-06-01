#include "i2c_fpga/tlv493d.h"
#define bitRead(byte,n) ((byte&(2^n))>>n)

uint8_t* initTLV(uint8_t deviceaddress, uint8_t* data, uint8_t devicepin)
{
    bool ADDR_pin;
    uint8_t IICAddr;
    uint8_t setaddr;
    uint8_t defaultaddr;
    uint8_t regdata[10];
    uint8_t cfgdata[3];


    if (devicepin == 255){
        ADDR_pin = 1;
        IICAddr  = 0;
        setaddr  = 0b1011110;
        printf("No device power pin selected, asuming device is on and continuing with the default address: %x .\n",setaddr);
        printf("Triggering general reset to make sure device is configured correctly\n");
        I2C::write(0,1,1);   // Clock in address 0 to reset 'ALL' devices. And write 1,
                            // very important to keep SDA line up or else address changes to 0b00011111

    }else{
        ADDR_pin = bitRead(deviceaddress,6);
        IICAddr  = (!bitRead(deviceaddress,4)<<1)|(!bitRead(deviceaddress,2));
        setaddr  = (ADDR_pin<<6)|(!bitRead(IICAddr,1)<<4)|(1<<3)|(!bitRead(IICAddr,0)<<2)|(1<<1)|(!ADDR_pin);
    }

    if (setaddr != deviceaddress){
        printf("Invalid device address! Please check and try again.\n");
        return NULL;
    }else{
        printf("Configuring device on pin %d with address: %x (ADDR_pin = %x, IICAddr = %x)\n", devicepin, setaddr, ADDR_pin, IICAddr);
    }

    uint tmpreg = IORD(h2p_lw_i2c_addr,5); // Read previous pin status so as to not overwrite any values

    if (ADDR_pin != 1){
        // Unblock I2C and take control of the SDA line
        IOWR(h2p_lw_i2c_addr, 5, tmpreg|0b1000);
        printf("Activating 'El cacharro' %d (SDA Low)\n", devicepin);
        // Power on device while SDA low to set ADDR bit to 0
        usleep(1);
        IOWR(h2p_lw_i2c_addr, 5, tmpreg|(1<<devicepin));
        usleep(1000);                     // At least during 200us
        // Release SDA line again
        IOWR(h2p_lw_i2c_addr, 5, tmpreg&0b0111);
        defaultaddr = 0b0011111;
    }else{
        if(devicepin != 255){
            IOWR(h2p_lw_i2c_addr, 5, tmpreg|(1<<devicepin));
            printf("Activating 'El cacharro' %d\n", devicepin);
        }
        usleep(1000);
        defaultaddr = 0b1011110;
    }

    printf("Backing up initial register config\n");
    int reg = 0;
//  printf("But first checking correct address by bruteforcing the entire range. Fuck you angry pixies!\n");
//  for (uint8_t tmpaddr = 1; tmpaddr <=127; tmpaddr++){
//    I2C::read(tmpaddr, 0, (uint8_t) 10);
//    if(Wire.available()){             //Todo: implement this feature in the I2C core in the FPGA
                                        //Todo: So, technically this is already in place, just read register 5 with IORD
//      printf("Address found: %x\n", tmpaddr);
//      break;
//    }
//  }
    I2C::read(defaultaddr, 0, 10); // Beginning first read (for backup)
    while(!IORD(h2p_lw_i2c_addr,5)){
        regdata[reg] = Wire.read();
        reg++;
    }
    if ( reg != 10 ){   // Todo: Handle this as a true error and retry!
        printf("ERROR: Data could not be read correctly and may be incomplete! Continuing, nevertheless ..."
               "  (reg = %d)\n",reg);
    }

    // Begin config
    // Static initial config for now
    cfgdata[0] = (IICAddr<<5)|(regdata[7]&0b00011000)|0b010;  // Last 3 bits: INT/FAST/LP
    cfgdata[1] = regdata[8];
    cfgdata[2] = (0b010<<5)|(regdata[9]&0b11111);
    // First 3 bits: Enable temp/Low power interval/Parity test

    // Calculate parity bit     (well doesen't work for now so fuck it)
    // bool parity = bitRead(cfgdata[0]+cfgdata[1]+cfgdata[2],0);
    // printf("Setting parity bit to ");
    // printfln(parity,BIN);
    // bitWrite(cfgdata[0],7,parity);

    // Write config
    printf("Writing config now ...\n");
    configureTLV(defaultaddr, cfgdata, sizeof(cfgdata));
    // End config

    printf("Checking new config ...\n");
    reg = 0;
    Wire.requestFrom(setaddr,(uint8_t) 10);
    while(Wire.available()){
        regdata[reg] = Wire.read();
        reg++;
    }
    if ( reg != 10 ){   //Todo: Handle this as a true error and retry!
        printf("ERROR: Data could not be read correctly and may be incomplete! Continuing, nevertheless ..."
                       "  (reg = %d)\n",reg);
    }

    return regdata;
}

void configureTLV(uint8_t deviceaddress, uint8_t* data, int count)
{
    I2C::write(deviceaddress,0,1);
    for (int i = 0; i < count; i++){
        I2C::write(deviceaddress,data[i],1);
    }
}

float convertToMilliTesla(uint8_t data)
{
    float mTs = 0;
    uint8_t bitmask = 1;
    for(int i=0; i<7; i++){
        mTs += (data&bitmask);
        bitmask <<= 1;
    }
    mTs -= (data&bitmask);
    return (mTs*1.56f);
}

void readTLV_B_MSB(int deviceaddress, uint8_t *data)
{
    // Read the first 3 registers only. Corresponding to the 8bit MSB values of the magnetic field
    Wire.requestFrom(deviceaddress,3);
    data[0] = Wire.read();          // Todo: change I2C fpga core
    data[1] = Wire.read();
    data[2] = Wire.read();
}
