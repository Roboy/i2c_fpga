#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "i2c_fpga/hps_0.h"
#include "i2c_fpga/am4096.hpp"
#include <limits.h>

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

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

	vector<int> deviceIDs = {0,1};
	AM4096 jointAngle_leftLeg(h2p_lw_i2c_addr0,deviceIDs);
	AM4096 jointAngle_rightLeg(h2p_lw_i2c_addr1,deviceIDs);
	while(ros::ok()){
        {
            vector<uint32_t> absAngles, relAngles, tacho;
            vector<uint8_t> agcGain;
            vector<bool> tooFar, tooClose;
            jointAngle_leftLeg.readAbsAngle(absAngles);
            jointAngle_leftLeg.readRelAngle(relAngles);
            jointAngle_leftLeg.readMagnetStatus(tooFar, tooClose);
            jointAngle_leftLeg.readTacho(tacho);
            jointAngle_leftLeg.readAgcGain(agcGain);
            for (uint i = 0; i < jointAngle_leftLeg.i2cAddrs.size(); i++) {
                printf("-------------left leg sensor %d -------------\n", jointAngle_leftLeg.i2cAddrs[i]);
                printf("magnet   %s\n", (tooFar[i] ? "too far" : (tooClose[i] ? "too close" : "ok")));
                printf("agc gain %d\n", agcGain[i]);
                printf("absPos   %d\n", absAngles[i]);
                printf("relPos   %d\n", relAngles[i]);
                printf("tacho    %d\n", tacho[i]);
                printf("\n");
            }
        }
        {
            vector<uint32_t> absAngles, relAngles, tacho;
            vector<uint8_t> agcGain;
            vector<bool> tooFar, tooClose;
            jointAngle_rightLeg.readAbsAngle(absAngles);
            jointAngle_rightLeg.readRelAngle(relAngles);
            jointAngle_rightLeg.readMagnetStatus(tooFar, tooClose);
            jointAngle_rightLeg.readTacho(tacho);
            jointAngle_rightLeg.readAgcGain(agcGain);
            for (uint i = 0; i < jointAngle_rightLeg.i2cAddrs.size(); i++) {
                printf("-------------right leg sensor %d -------------\n", jointAngle_rightLeg.i2cAddrs[i]);
                printf("magnet   %s\n", (tooFar[i] ? "too far" : (tooClose[i] ? "too close" : "ok")));
                printf("agc gain %d\n", agcGain[i]);
                printf("absPos   %d\n", absAngles[i]);
                printf("relPos   %d\n", relAngles[i]);
                printf("tacho    %d\n", tacho[i]);
                printf("\n");
            }
        }
		usleep(100000);
	}


	// clean up our memory mapping and exit
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
