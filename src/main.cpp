#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "i2c_fpga/hps_0.h"
#include "i2c_fpga/tlv493d.hpp"
#include <limits.h>

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

// Look in the device's user manual for allowed addresses! (Table 6)
vector<uint8_t> deviceaddress = {0b1001010, 0b1001110};//
vector<int> devicepin = {0,1};

int main(int argc, char *argv[]) {

	void *virtual_base;
	int fd;
	void *h2p_lw_i2c_addr;

	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span

	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	
	h2p_lw_i2c_addr = virtual_base + ( (unsigned long) ( ALT_LWFPGASLVS_OFST + I2C_AVALON_BRIDGE_0_BASE ) & (unsigned long) ( HW_REGS_MASK ) );

	if (!ros::isInitialized()) {
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "i2c_fpga", ros::init_options::AnonymousName);
	}

    // Initialize I2C bus and setup sensors

    TLV493D tlv493D(h2p_lw_i2c_addr, deviceaddress, devicepin);


//    vector<uint8_t> data;
//    tlv493D.i2c->read_continuous(deviceaddress[0], 10, data); // Beginning first read (for backup)
//    for(uint8_t val:data){
//        ROS_INFO("%x", val);
//    }

//    printf("Ok, initialized I2C bus\n");
//    printf("---------Activating sensors--------\n");
//    usleep(500);                                       // Letting things settle
//    factory_settings[0] = initTLV(deviceaddress[0], initcfg, devicepin[0]);    // Write device address and initial config
//    factory_settings[1] = initTLV(deviceaddress[1], initcfg, devicepin[1]);
//    factory_settings[2] = initTLV(deviceaddress[2], initcfg, devicepin[2]);
//    printf("---------Done configuring---------\n");
//
//    // Read data out and convert it


    ROS_INFO("deviceaddress0:\t" BYTE_TO_BINARY_PATTERN,BYTE_TO_BINARY(deviceaddress[0]));
    ROS_INFO("deviceaddress1:\t" BYTE_TO_BINARY_PATTERN,BYTE_TO_BINARY(deviceaddress[1]));

    while(ros::ok()){
        usleep(10000);
        vector<uint8_t> data;
        tlv493D.readTLV_B_MSB(deviceaddress[0], data);
        tlv493D.readTLV_B_MSB(deviceaddress[1], data);
        ROS_INFO("sensor 0:\n%f \t%f \t%f", tlv493D.convertToMilliTesla(data[0]),
                 tlv493D.convertToMilliTesla(data[1]),
                 tlv493D.convertToMilliTesla(data[2]));
        ROS_INFO("sensor 1:\n%f \t%f \t%f", tlv493D.convertToMilliTesla(data[3]),
                 tlv493D.convertToMilliTesla(data[4]),
                 tlv493D.convertToMilliTesla(data[5]));
    }

//	vector<int> deviceIDs = {0,1,2,3};
//	AM4096 jointAngle(h2p_lw_i2c_addr,deviceIDs);
//	while(ros::ok()){
//		vector<uint32_t> absAngles, relAngles, tacho;
//		vector<uint8_t> agcGain;
//		vector<bool> tooFar, tooClose;
//		jointAngle.readAbsAngle(absAngles);
//		jointAngle.readRelAngle(relAngles);
//		jointAngle.readMagnetStatus(tooFar, tooClose);
//		jointAngle.readTacho(tacho);
//		jointAngle.readAgcGain(agcGain);
//		for(uint i=0; i<jointAngle.i2cAddrs.size(); i++) {
//			printf("-------------sensor %d -------------\n", jointAngle.i2cAddrs[i]);
//			printf("magnet   %s\n", (tooFar[i] ? "too far" : (tooClose[i] ? "too close" : "ok")));
//			printf("agc gain %d\n", agcGain[i]);
//			printf("absPos   %d\n", absAngles[i]);
//			printf("relPos   %d\n", relAngles[i]);
//			printf("tacho    %d\n", tacho[i]);
//			printf("\n");
//		}
//		usleep(100000);
//	}


	// clean up our memory mapping and exit
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
