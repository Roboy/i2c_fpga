`timescale 1ns/10ps

module SpiControl (
   input clock,
	input [(8*32)-1:0] data,
	input dataReady,
	input reset_n,
	input di_req,
	input write_ack,
	input mpu_interrupt_in,
	output reg [7:0] byte,
	output reg wren,
	output mpu_interrupt_out
);
// Splits the 32-bit data into bitfield chunks
// 
// data -- input data
// dataReady -- signal data is ready for transmission
// send -- toggle the transmission
// bitfield -- which part should be send

reg [7:0] numberOfBytesTransmitted;
reg write_ack_prev;

assign mpu_interrupt_out = mpu_interrupt_in;

always @(posedge clock, negedge reset_n) begin: SPICONTROL_SPILOGIC
    if (reset_n == 0) begin
			numberOfBytesTransmitted <= 0;
			wren <= 0;
			write_ack_prev <= 0;
    end
	 
    else begin
			
			write_ack_prev <= write_ack;
			if( write_ack_prev==0 && write_ack == 1 ) begin
				wren <= 0;
				numberOfBytesTransmitted <= numberOfBytesTransmitted + 1;
			end
			
			if(di_req==1 && numberOfBytesTransmitted<34) begin
				case(numberOfBytesTransmitted)
					1: byte <= 0; // this is the address we want to write to on the esp(we start at 0)
					// sensor0
					2: byte <= data[7:0];
					3: byte <= data[15:8];
					4: byte <= data[23:16];
					5: byte <= data[31:24];
					// sensor1
					6: byte <= data[39:32];
					7: byte <= data[47:40];
					8: byte <= data[55:48];
					9: byte <= data[63:56];
					// sensor3
					10: byte <= data[71:64];
					11: byte <= data[79:72];
					12: byte <= data[87:80];
					13: byte <= data[95:88];
					// sensor4
					14: byte <= data[103:96];
					15: byte <= data[111:104];
					16: byte <= data[119:112];
					17: byte <= data[127:120];
					// sensor5
					18: byte <= data[135:128];
					19: byte <= data[143:136];
					20: byte <= data[153:144];
					21: byte <= data[159:152];
					// sensor6
					22: byte <= data[167:160];
					23: byte <= data[175:168];
					24: byte <= data[183:176];
					25: byte <= data[191:184];
					// sensor7
					26: byte <= data[199:192];
					27: byte <= data[209:200];
					28: byte <= data[215:208];
					29: byte <= data[223:216];
					// sensor8
					30: byte <= data[231:224];
					31: byte <= data[239:232];
					32: byte <= data[247:240];
					33: byte <= data[255:248];
					default: byte <= numberOfBytesTransmitted-2;
				endcase
				wren <= 1;
			end
			
			if (dataReady==1) begin
				numberOfBytesTransmitted<= 0;
				byte <= 2;
				wren <= 1;
			end
    end
end

endmodule