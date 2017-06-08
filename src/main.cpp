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
vector<uint8_t> deviceaddress0 = {0b1001010, 0b1001110};//
vector<uint8_t> deviceaddress1 = {0b1001010};//
vector<int> devicepin0 = {0,1};
vector<int> devicepin1 = {0};

int main(int argc, char *argv[]) {

	void *virtual_base;
	int fd;
	void *h2p_lw_i2c_addr0, *h2p_lw_i2c_addr1;

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

    h2p_lw_i2c_addr0 = virtual_base + ( (unsigned long) ( ALT_LWFPGASLVS_OFST + I2C_AVALON_BRIDGE_0_BASE ) & (unsigned long) ( HW_REGS_MASK ) );
    h2p_lw_i2c_addr1 = virtual_base + ( (unsigned long) ( ALT_LWFPGASLVS_OFST + I2C_AVALON_BRIDGE_1_BASE ) & (unsigned long) ( HW_REGS_MASK ) );

	if (!ros::isInitialized()) {
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "i2c_fpga", ros::init_options::AnonymousName);
	}

    // Initialize I2C bus and setup sensors

    TLV493D tlv493D0(h2p_lw_i2c_addr0, deviceaddress0, devicepin0);
    TLV493D tlv493D1(h2p_lw_i2c_addr1, deviceaddress1, devicepin1);

    ros::NodeHandlePtr nh(new ros::NodeHandle);
    ros::AsyncSpinner spinner(1);
    ros::Publisher magneticSensor_pub = nh->advertise<roboy_communication_middleware::MagneticSensor>("/roboy/middleware/MagneticSensor",1);
    spinner.start();

    ros::Rate rate(30);
    while(ros::ok()){
        roboy_communication_middleware::MagneticSensor msg;
        vector<uint8_t> data;
        uint i=0;
        for(uint8_t device:deviceaddress0){
            tlv493D0.readTLV_B_MSB(device,data);
            msg.x.push_back(tlv493D0.convertToMilliTesla(data[i*3+0]));
            msg.y.push_back(tlv493D0.convertToMilliTesla(data[i*3+1]));
            msg.z.push_back(tlv493D0.convertToMilliTesla(data[i*3+2]));
            i++;
        }
        for(uint8_t device:deviceaddress1){
            tlv493D1.readTLV_B_MSB(device,data);
            msg.x.push_back(tlv493D1.convertToMilliTesla(data[i*3+0]));
            msg.y.push_back(tlv493D1.convertToMilliTesla(data[i*3+1]));
            msg.z.push_back(tlv493D1.convertToMilliTesla(data[i*3+2]));
            i++;
        }
        magneticSensor_pub.publish(msg);
        rate.sleep();
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
