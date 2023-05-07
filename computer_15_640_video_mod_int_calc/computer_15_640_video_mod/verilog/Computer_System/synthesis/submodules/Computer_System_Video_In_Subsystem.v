// Computer_System_Video_In_Subsystem.v

// Generated using ACDS version 18.1 625

`timescale 1 ps / 1 ps
module Computer_System_Video_In_Subsystem (
		input  wire [1:0]  edge_detection_control_slave_address,    // edge_detection_control_slave.address
		input  wire        edge_detection_control_slave_write_n,    //                             .write_n
		input  wire [31:0] edge_detection_control_slave_writedata,  //                             .writedata
		input  wire        edge_detection_control_slave_chipselect, //                             .chipselect
		output wire [31:0] edge_detection_control_slave_readdata,   //                             .readdata
		input  wire        sys_clk_clk,                             //                      sys_clk.clk
		input  wire        sys_reset_reset_n,                       //                    sys_reset.reset_n
		input  wire        video_in_TD_CLK27,                       //                     video_in.TD_CLK27
		input  wire [7:0]  video_in_TD_DATA,                        //                             .TD_DATA
		input  wire        video_in_TD_HS,                          //                             .TD_HS
		input  wire        video_in_TD_VS,                          //                             .TD_VS
		input  wire        video_in_clk27_reset,                    //                             .clk27_reset
		output wire        video_in_TD_RESET,                       //                             .TD_RESET
		output wire        video_in_overflow_flag,                  //                             .overflow_flag
		input  wire [1:0]  video_in_dma_control_slave_address,      //   video_in_dma_control_slave.address
		input  wire [3:0]  video_in_dma_control_slave_byteenable,   //                             .byteenable
		input  wire        video_in_dma_control_slave_read,         //                             .read
		input  wire        video_in_dma_control_slave_write,        //                             .write
		input  wire [31:0] video_in_dma_control_slave_writedata,    //                             .writedata
		output wire [31:0] video_in_dma_control_slave_readdata,     //                             .readdata
		output wire [31:0] video_in_dma_master_address,             //          video_in_dma_master.address
		input  wire        video_in_dma_master_waitrequest,         //                             .waitrequest
		output wire        video_in_dma_master_write,               //                             .write
		output wire [7:0]  video_in_dma_master_writedata            //                             .writedata
	);

	wire         video_in_chroma_resampler_avalon_chroma_source_valid;         // Video_In_Chroma_Resampler:stream_out_valid -> Edge_Detection_Subsystem:video_stream_sink_valid
	wire  [23:0] video_in_chroma_resampler_avalon_chroma_source_data;          // Video_In_Chroma_Resampler:stream_out_data -> Edge_Detection_Subsystem:video_stream_sink_data
	wire         video_in_chroma_resampler_avalon_chroma_source_ready;         // Edge_Detection_Subsystem:video_stream_sink_ready -> Video_In_Chroma_Resampler:stream_out_ready
	wire         video_in_chroma_resampler_avalon_chroma_source_startofpacket; // Video_In_Chroma_Resampler:stream_out_startofpacket -> Edge_Detection_Subsystem:video_stream_sink_startofpacket
	wire         video_in_chroma_resampler_avalon_chroma_source_endofpacket;   // Video_In_Chroma_Resampler:stream_out_endofpacket -> Edge_Detection_Subsystem:video_stream_sink_endofpacket
	wire         video_in_clipper_avalon_clipper_source_valid;                 // Video_In_Clipper:stream_out_valid -> Video_In_Scaler:stream_in_valid
	wire   [7:0] video_in_clipper_avalon_clipper_source_data;                  // Video_In_Clipper:stream_out_data -> Video_In_Scaler:stream_in_data
	wire         video_in_clipper_avalon_clipper_source_ready;                 // Video_In_Scaler:stream_in_ready -> Video_In_Clipper:stream_out_ready
	wire         video_in_clipper_avalon_clipper_source_startofpacket;         // Video_In_Clipper:stream_out_startofpacket -> Video_In_Scaler:stream_in_startofpacket
	wire         video_in_clipper_avalon_clipper_source_endofpacket;           // Video_In_Clipper:stream_out_endofpacket -> Video_In_Scaler:stream_in_endofpacket
	wire         video_in_csc_avalon_csc_source_valid;                         // Video_In_CSC:stream_out_valid -> Video_In_RGB_Resampler:stream_in_valid
	wire  [23:0] video_in_csc_avalon_csc_source_data;                          // Video_In_CSC:stream_out_data -> Video_In_RGB_Resampler:stream_in_data
	wire         video_in_csc_avalon_csc_source_ready;                         // Video_In_RGB_Resampler:stream_in_ready -> Video_In_CSC:stream_out_ready
	wire         video_in_csc_avalon_csc_source_startofpacket;                 // Video_In_CSC:stream_out_startofpacket -> Video_In_RGB_Resampler:stream_in_startofpacket
	wire         video_in_csc_avalon_csc_source_endofpacket;                   // Video_In_CSC:stream_out_endofpacket -> Video_In_RGB_Resampler:stream_in_endofpacket
	wire         video_in_avalon_decoder_source_valid;                         // Video_In:stream_out_valid -> Video_In_Chroma_Resampler:stream_in_valid
	wire  [15:0] video_in_avalon_decoder_source_data;                          // Video_In:stream_out_data -> Video_In_Chroma_Resampler:stream_in_data
	wire         video_in_avalon_decoder_source_ready;                         // Video_In_Chroma_Resampler:stream_in_ready -> Video_In:stream_out_ready
	wire         video_in_avalon_decoder_source_startofpacket;                 // Video_In:stream_out_startofpacket -> Video_In_Chroma_Resampler:stream_in_startofpacket
	wire         video_in_avalon_decoder_source_endofpacket;                   // Video_In:stream_out_endofpacket -> Video_In_Chroma_Resampler:stream_in_endofpacket
	wire         video_in_rgb_resampler_avalon_rgb_source_valid;               // Video_In_RGB_Resampler:stream_out_valid -> Video_In_Clipper:stream_in_valid
	wire   [7:0] video_in_rgb_resampler_avalon_rgb_source_data;                // Video_In_RGB_Resampler:stream_out_data -> Video_In_Clipper:stream_in_data
	wire         video_in_rgb_resampler_avalon_rgb_source_ready;               // Video_In_Clipper:stream_in_ready -> Video_In_RGB_Resampler:stream_out_ready
	wire         video_in_rgb_resampler_avalon_rgb_source_startofpacket;       // Video_In_RGB_Resampler:stream_out_startofpacket -> Video_In_Clipper:stream_in_startofpacket
	wire         video_in_rgb_resampler_avalon_rgb_source_endofpacket;         // Video_In_RGB_Resampler:stream_out_endofpacket -> Video_In_Clipper:stream_in_endofpacket
	wire         video_in_scaler_avalon_scaler_source_valid;                   // Video_In_Scaler:stream_out_valid -> Video_In_DMA:stream_valid
	wire   [7:0] video_in_scaler_avalon_scaler_source_data;                    // Video_In_Scaler:stream_out_data -> Video_In_DMA:stream_data
	wire         video_in_scaler_avalon_scaler_source_ready;                   // Video_In_DMA:stream_ready -> Video_In_Scaler:stream_out_ready
	wire         video_in_scaler_avalon_scaler_source_startofpacket;           // Video_In_Scaler:stream_out_startofpacket -> Video_In_DMA:stream_startofpacket
	wire         video_in_scaler_avalon_scaler_source_endofpacket;             // Video_In_Scaler:stream_out_endofpacket -> Video_In_DMA:stream_endofpacket
	wire         edge_detection_subsystem_video_stream_source_valid;           // Edge_Detection_Subsystem:video_stream_source_valid -> Video_In_CSC:stream_in_valid
	wire  [23:0] edge_detection_subsystem_video_stream_source_data;            // Edge_Detection_Subsystem:video_stream_source_data -> Video_In_CSC:stream_in_data
	wire         edge_detection_subsystem_video_stream_source_ready;           // Video_In_CSC:stream_in_ready -> Edge_Detection_Subsystem:video_stream_source_ready
	wire         edge_detection_subsystem_video_stream_source_startofpacket;   // Edge_Detection_Subsystem:video_stream_source_startofpacket -> Video_In_CSC:stream_in_startofpacket
	wire         edge_detection_subsystem_video_stream_source_endofpacket;     // Edge_Detection_Subsystem:video_stream_source_endofpacket -> Video_In_CSC:stream_in_endofpacket
	wire         rst_controller_reset_out_reset;                               // rst_controller:reset_out -> [Video_In:reset, Video_In_CSC:reset, Video_In_Chroma_Resampler:reset, Video_In_Clipper:reset, Video_In_DMA:reset, Video_In_RGB_Resampler:reset, Video_In_Scaler:reset]

	Computer_System_Video_In_Subsystem_Edge_Detection_Subsystem edge_detection_subsystem (
		.edge_detection_control_slave_address    (edge_detection_control_slave_address),                         // edge_detection_control_slave.address
		.edge_detection_control_slave_write_n    (edge_detection_control_slave_write_n),                         //                             .write_n
		.edge_detection_control_slave_writedata  (edge_detection_control_slave_writedata),                       //                             .writedata
		.edge_detection_control_slave_chipselect (edge_detection_control_slave_chipselect),                      //                             .chipselect
		.edge_detection_control_slave_readdata   (edge_detection_control_slave_readdata),                        //                             .readdata
		.sys_clk_clk                             (sys_clk_clk),                                                  //                      sys_clk.clk
		.sys_reset_reset_n                       (sys_reset_reset_n),                                            //                    sys_reset.reset_n
		.video_stream_sink_data                  (video_in_chroma_resampler_avalon_chroma_source_data),          //            video_stream_sink.data
		.video_stream_sink_startofpacket         (video_in_chroma_resampler_avalon_chroma_source_startofpacket), //                             .startofpacket
		.video_stream_sink_endofpacket           (video_in_chroma_resampler_avalon_chroma_source_endofpacket),   //                             .endofpacket
		.video_stream_sink_valid                 (video_in_chroma_resampler_avalon_chroma_source_valid),         //                             .valid
		.video_stream_sink_ready                 (video_in_chroma_resampler_avalon_chroma_source_ready),         //                             .ready
		.video_stream_source_ready               (edge_detection_subsystem_video_stream_source_ready),           //          video_stream_source.ready
		.video_stream_source_data                (edge_detection_subsystem_video_stream_source_data),            //                             .data
		.video_stream_source_startofpacket       (edge_detection_subsystem_video_stream_source_startofpacket),   //                             .startofpacket
		.video_stream_source_endofpacket         (edge_detection_subsystem_video_stream_source_endofpacket),     //                             .endofpacket
		.video_stream_source_valid               (edge_detection_subsystem_video_stream_source_valid)            //                             .valid
	);

	Computer_System_Video_In_Subsystem_Video_In video_in (
		.clk                      (sys_clk_clk),                                  //                   clk.clk
		.reset                    (rst_controller_reset_out_reset),               //                 reset.reset
		.stream_out_ready         (video_in_avalon_decoder_source_ready),         // avalon_decoder_source.ready
		.stream_out_startofpacket (video_in_avalon_decoder_source_startofpacket), //                      .startofpacket
		.stream_out_endofpacket   (video_in_avalon_decoder_source_endofpacket),   //                      .endofpacket
		.stream_out_valid         (video_in_avalon_decoder_source_valid),         //                      .valid
		.stream_out_data          (video_in_avalon_decoder_source_data),          //                      .data
		.TD_CLK27                 (video_in_TD_CLK27),                            //    external_interface.export
		.TD_DATA                  (video_in_TD_DATA),                             //                      .export
		.TD_HS                    (video_in_TD_HS),                               //                      .export
		.TD_VS                    (video_in_TD_VS),                               //                      .export
		.clk27_reset              (video_in_clk27_reset),                         //                      .export
		.TD_RESET                 (video_in_TD_RESET),                            //                      .export
		.overflow_flag            (video_in_overflow_flag)                        //                      .export
	);

	Computer_System_Video_In_Subsystem_Video_In_CSC video_in_csc (
		.clk                      (sys_clk_clk),                                                //               clk.clk
		.reset                    (rst_controller_reset_out_reset),                             //             reset.reset
		.stream_in_startofpacket  (edge_detection_subsystem_video_stream_source_startofpacket), //   avalon_csc_sink.startofpacket
		.stream_in_endofpacket    (edge_detection_subsystem_video_stream_source_endofpacket),   //                  .endofpacket
		.stream_in_valid          (edge_detection_subsystem_video_stream_source_valid),         //                  .valid
		.stream_in_ready          (edge_detection_subsystem_video_stream_source_ready),         //                  .ready
		.stream_in_data           (edge_detection_subsystem_video_stream_source_data),          //                  .data
		.stream_out_ready         (video_in_csc_avalon_csc_source_ready),                       // avalon_csc_source.ready
		.stream_out_startofpacket (video_in_csc_avalon_csc_source_startofpacket),               //                  .startofpacket
		.stream_out_endofpacket   (video_in_csc_avalon_csc_source_endofpacket),                 //                  .endofpacket
		.stream_out_valid         (video_in_csc_avalon_csc_source_valid),                       //                  .valid
		.stream_out_data          (video_in_csc_avalon_csc_source_data)                         //                  .data
	);

	Computer_System_Video_In_Subsystem_Video_In_Chroma_Resampler video_in_chroma_resampler (
		.clk                      (sys_clk_clk),                                                  //                  clk.clk
		.reset                    (rst_controller_reset_out_reset),                               //                reset.reset
		.stream_in_startofpacket  (video_in_avalon_decoder_source_startofpacket),                 //   avalon_chroma_sink.startofpacket
		.stream_in_endofpacket    (video_in_avalon_decoder_source_endofpacket),                   //                     .endofpacket
		.stream_in_valid          (video_in_avalon_decoder_source_valid),                         //                     .valid
		.stream_in_ready          (video_in_avalon_decoder_source_ready),                         //                     .ready
		.stream_in_data           (video_in_avalon_decoder_source_data),                          //                     .data
		.stream_out_ready         (video_in_chroma_resampler_avalon_chroma_source_ready),         // avalon_chroma_source.ready
		.stream_out_startofpacket (video_in_chroma_resampler_avalon_chroma_source_startofpacket), //                     .startofpacket
		.stream_out_endofpacket   (video_in_chroma_resampler_avalon_chroma_source_endofpacket),   //                     .endofpacket
		.stream_out_valid         (video_in_chroma_resampler_avalon_chroma_source_valid),         //                     .valid
		.stream_out_data          (video_in_chroma_resampler_avalon_chroma_source_data)           //                     .data
	);

	Computer_System_Video_In_Subsystem_Video_In_Clipper video_in_clipper (
		.clk                      (sys_clk_clk),                                            //                   clk.clk
		.reset                    (rst_controller_reset_out_reset),                         //                 reset.reset
		.stream_in_data           (video_in_rgb_resampler_avalon_rgb_source_data),          //   avalon_clipper_sink.data
		.stream_in_startofpacket  (video_in_rgb_resampler_avalon_rgb_source_startofpacket), //                      .startofpacket
		.stream_in_endofpacket    (video_in_rgb_resampler_avalon_rgb_source_endofpacket),   //                      .endofpacket
		.stream_in_valid          (video_in_rgb_resampler_avalon_rgb_source_valid),         //                      .valid
		.stream_in_ready          (video_in_rgb_resampler_avalon_rgb_source_ready),         //                      .ready
		.stream_out_ready         (video_in_clipper_avalon_clipper_source_ready),           // avalon_clipper_source.ready
		.stream_out_data          (video_in_clipper_avalon_clipper_source_data),            //                      .data
		.stream_out_startofpacket (video_in_clipper_avalon_clipper_source_startofpacket),   //                      .startofpacket
		.stream_out_endofpacket   (video_in_clipper_avalon_clipper_source_endofpacket),     //                      .endofpacket
		.stream_out_valid         (video_in_clipper_avalon_clipper_source_valid)            //                      .valid
	);

	Computer_System_Video_In_Subsystem_Video_In_DMA video_in_dma (
		.clk                  (sys_clk_clk),                                        //                      clk.clk
		.reset                (rst_controller_reset_out_reset),                     //                    reset.reset
		.stream_data          (video_in_scaler_avalon_scaler_source_data),          //          avalon_dma_sink.data
		.stream_startofpacket (video_in_scaler_avalon_scaler_source_startofpacket), //                         .startofpacket
		.stream_endofpacket   (video_in_scaler_avalon_scaler_source_endofpacket),   //                         .endofpacket
		.stream_valid         (video_in_scaler_avalon_scaler_source_valid),         //                         .valid
		.stream_ready         (video_in_scaler_avalon_scaler_source_ready),         //                         .ready
		.slave_address        (video_in_dma_control_slave_address),                 // avalon_dma_control_slave.address
		.slave_byteenable     (video_in_dma_control_slave_byteenable),              //                         .byteenable
		.slave_read           (video_in_dma_control_slave_read),                    //                         .read
		.slave_write          (video_in_dma_control_slave_write),                   //                         .write
		.slave_writedata      (video_in_dma_control_slave_writedata),               //                         .writedata
		.slave_readdata       (video_in_dma_control_slave_readdata),                //                         .readdata
		.master_address       (video_in_dma_master_address),                        //        avalon_dma_master.address
		.master_waitrequest   (video_in_dma_master_waitrequest),                    //                         .waitrequest
		.master_write         (video_in_dma_master_write),                          //                         .write
		.master_writedata     (video_in_dma_master_writedata)                       //                         .writedata
	);

	Computer_System_Video_In_Subsystem_Video_In_RGB_Resampler video_in_rgb_resampler (
		.clk                      (sys_clk_clk),                                            //               clk.clk
		.reset                    (rst_controller_reset_out_reset),                         //             reset.reset
		.stream_in_startofpacket  (video_in_csc_avalon_csc_source_startofpacket),           //   avalon_rgb_sink.startofpacket
		.stream_in_endofpacket    (video_in_csc_avalon_csc_source_endofpacket),             //                  .endofpacket
		.stream_in_valid          (video_in_csc_avalon_csc_source_valid),                   //                  .valid
		.stream_in_ready          (video_in_csc_avalon_csc_source_ready),                   //                  .ready
		.stream_in_data           (video_in_csc_avalon_csc_source_data),                    //                  .data
		.slave_read               (),                                                       //  avalon_rgb_slave.read
		.slave_readdata           (),                                                       //                  .readdata
		.stream_out_ready         (video_in_rgb_resampler_avalon_rgb_source_ready),         // avalon_rgb_source.ready
		.stream_out_startofpacket (video_in_rgb_resampler_avalon_rgb_source_startofpacket), //                  .startofpacket
		.stream_out_endofpacket   (video_in_rgb_resampler_avalon_rgb_source_endofpacket),   //                  .endofpacket
		.stream_out_valid         (video_in_rgb_resampler_avalon_rgb_source_valid),         //                  .valid
		.stream_out_data          (video_in_rgb_resampler_avalon_rgb_source_data)           //                  .data
	);

	Computer_System_Video_In_Subsystem_Video_In_Scaler video_in_scaler (
		.clk                      (sys_clk_clk),                                          //                  clk.clk
		.reset                    (rst_controller_reset_out_reset),                       //                reset.reset
		.stream_in_startofpacket  (video_in_clipper_avalon_clipper_source_startofpacket), //   avalon_scaler_sink.startofpacket
		.stream_in_endofpacket    (video_in_clipper_avalon_clipper_source_endofpacket),   //                     .endofpacket
		.stream_in_valid          (video_in_clipper_avalon_clipper_source_valid),         //                     .valid
		.stream_in_ready          (video_in_clipper_avalon_clipper_source_ready),         //                     .ready
		.stream_in_data           (video_in_clipper_avalon_clipper_source_data),          //                     .data
		.stream_out_ready         (video_in_scaler_avalon_scaler_source_ready),           // avalon_scaler_source.ready
		.stream_out_startofpacket (video_in_scaler_avalon_scaler_source_startofpacket),   //                     .startofpacket
		.stream_out_endofpacket   (video_in_scaler_avalon_scaler_source_endofpacket),     //                     .endofpacket
		.stream_out_valid         (video_in_scaler_avalon_scaler_source_valid),           //                     .valid
		.stream_out_data          (video_in_scaler_avalon_scaler_source_data)             //                     .data
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
		.reset_in0      (~sys_reset_reset_n),             // reset_in0.reset
		.clk            (sys_clk_clk),                    //       clk.clk
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
