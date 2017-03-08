// spi.v

// Generated using ACDS version 16.1 196

`timescale 1 ps / 1 ps
module spi (
		input  wire        clk_clk,                           //                    clk.clk
		input  wire        reset_reset_n,                     //                  reset.reset_n
		input  wire        spi_0_external_MISO,               //         spi_0_external.MISO
		output wire        spi_0_external_MOSI,               //                       .MOSI
		output wire        spi_0_external_SCLK,               //                       .SCLK
		output wire        spi_0_external_SS_n,               //                       .SS_n
		input  wire [15:0] spi_0_spi_control_port_writedata,  // spi_0_spi_control_port.writedata
		output wire [15:0] spi_0_spi_control_port_readdata,   //                       .readdata
		input  wire [2:0]  spi_0_spi_control_port_address,    //                       .address
		input  wire        spi_0_spi_control_port_read_n,     //                       .read_n
		input  wire        spi_0_spi_control_port_chipselect, //                       .chipselect
		input  wire        spi_0_spi_control_port_write_n,     //                       .write_n
		output wire	ready_for_data
	);

	wire    rst_controller_reset_out_reset; // rst_controller:reset_out -> spi_0:reset_n

	spi_spi_0 spi_0 (
		.clk           (clk_clk),                           //              clk.clk
		.reset_n       (~rst_controller_reset_out_reset),   //            reset.reset_n
		.data_from_cpu (spi_0_spi_control_port_writedata),  // spi_control_port.writedata
		.data_to_cpu   (spi_0_spi_control_port_readdata),   //                 .readdata
		.mem_addr      (spi_0_spi_control_port_address),    //                 .address
		.read_n        (spi_0_spi_control_port_read_n),     //                 .read_n
		.spi_select    (spi_0_spi_control_port_chipselect), //                 .chipselect
		.write_n       (spi_0_spi_control_port_write_n),    //                 .write_n
		.irq           (),                                  //              irq.irq
		.MISO          (spi_0_external_MISO),               //         external.export
		.MOSI          (spi_0_external_MOSI),               //                 .export
		.SCLK          (spi_0_external_SCLK),               //                 .export
		.SS_n          (spi_0_external_SS_n),                //                 .export
		.readyfordata(ready_for_data)
	);

	altera_reset_controller #(
		.NUM_RESET_INPUTS          (1),
		.OUTPUT_RESET_SYNC_EDGES   ("deassert"),
		.SYNC_DEPTH                (2),
		.RESET_REQUEST_PRESENT     (0),
		.RESET_REQ_WAIT_TIME       (1),
		.MIN_RST_ASSERTION_TIME    (3),
		.RESET_REQ_EARLY_DSRT_TIME (1),
		.USE_RESET_REQUEST_IN0     (0),
		.USE_RESET_REQUEST_IN1     (0),
		.USE_RESET_REQUEST_IN2     (0),
		.USE_RESET_REQUEST_IN3     (0),
		.USE_RESET_REQUEST_IN4     (0),
		.USE_RESET_REQUEST_IN5     (0),
		.USE_RESET_REQUEST_IN6     (0),
		.USE_RESET_REQUEST_IN7     (0),
		.USE_RESET_REQUEST_IN8     (0),
		.USE_RESET_REQUEST_IN9     (0),
		.USE_RESET_REQUEST_IN10    (0),
		.USE_RESET_REQUEST_IN11    (0),
		.USE_RESET_REQUEST_IN12    (0),
		.USE_RESET_REQUEST_IN13    (0),
		.USE_RESET_REQUEST_IN14    (0),
		.USE_RESET_REQUEST_IN15    (0),
		.ADAPT_RESET_REQUEST       (0)
	) rst_controller (
		.reset_in0      (~reset_reset_n),                 // reset_in0.reset
		.clk            (clk_clk),                        //       clk.clk
		.reset_out      (rst_controller_reset_out_reset), // reset_out.reset
		.reset_req      (),                               // (terminated)
		.reset_req_in0  (1'b0),                           // (terminated)
		.reset_in1      (1'b0),                           // (terminated)
		.reset_req_in1  (1'b0),                           // (terminated)
		.reset_in2      (1'b0),                           // (terminated)
		.reset_req_in2  (1'b0),                           // (terminated)
		.reset_in3      (1'b0),                           // (terminated)
		.reset_req_in3  (1'b0),                           // (terminated)
		.reset_in4      (1'b0),                           // (terminated)
		.reset_req_in4  (1'b0),                           // (terminated)
		.reset_in5      (1'b0),                           // (terminated)
		.reset_req_in5  (1'b0),                           // (terminated)
		.reset_in6      (1'b0),                           // (terminated)
		.reset_req_in6  (1'b0),                           // (terminated)
		.reset_in7      (1'b0),                           // (terminated)
		.reset_req_in7  (1'b0),                           // (terminated)
		.reset_in8      (1'b0),                           // (terminated)
		.reset_req_in8  (1'b0),                           // (terminated)
		.reset_in9      (1'b0),                           // (terminated)
		.reset_req_in9  (1'b0),                           // (terminated)
		.reset_in10     (1'b0),                           // (terminated)
		.reset_req_in10 (1'b0),                           // (terminated)
		.reset_in11     (1'b0),                           // (terminated)
		.reset_req_in11 (1'b0),                           // (terminated)
		.reset_in12     (1'b0),                           // (terminated)
		.reset_req_in12 (1'b0),                           // (terminated)
		.reset_in13     (1'b0),                           // (terminated)
		.reset_req_in13 (1'b0),                           // (terminated)
		.reset_in14     (1'b0),                           // (terminated)
		.reset_req_in14 (1'b0),                           // (terminated)
		.reset_in15     (1'b0),                           // (terminated)
		.reset_req_in15 (1'b0)                            // (terminated)
	);

endmodule