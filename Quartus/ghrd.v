// ============================================================================
// Copyright (c) 2014 by Terasic Technologies Inc.
// ============================================================================
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// ============================================================================
//           
//  Terasic Technologies Inc
//  9F., No.176, Sec.2, Gongdao 5th Rd, East Dist, Hsinchu City, 30070. Taiwan
//  
//  
//                     web: http://www.terasic.com/  
//                     email: support@terasic.com
//
// ============================================================================
//Date:  Tue Dec  2 09:28:38 2014
// ============================================================================

`define ENABLE_HPS
//`define ENABLE_CLK

module ghrd(

      ///////// ADC /////////
      output             ADC_CONVST,
      output             ADC_SCK,
      output             ADC_SDI,
      input              ADC_SDO,

      ///////// ARDUINO /////////
      inout       [15:0] ARDUINO_IO,
      inout              ARDUINO_RESET_N,

`ifdef ENABLE_CLK
      ///////// CLK /////////
      output             CLK_I2C_SCL,
      inout              CLK_I2C_SDA,
`endif /*ENABLE_CLK*/

      ///////// FPGA /////////
      input              FPGA_CLK1_50,
      input              FPGA_CLK2_50,
      input              FPGA_CLK3_50,

      ///////// GPIO /////////
      inout       [35:0] GPIO_0,
      inout       [35:0] GPIO_1,

`ifdef ENABLE_HPS
      ///////// HPS /////////
      inout              HPS_CONV_USB_N,
      output      [14:0] HPS_DDR3_ADDR,
      output      [2:0]  HPS_DDR3_BA,
      output             HPS_DDR3_CAS_N,
      output             HPS_DDR3_CKE,
      output             HPS_DDR3_CK_N,
      output             HPS_DDR3_CK_P,
      output             HPS_DDR3_CS_N,
      output      [3:0]  HPS_DDR3_DM,
      inout       [31:0] HPS_DDR3_DQ,
      inout       [3:0]  HPS_DDR3_DQS_N,
      inout       [3:0]  HPS_DDR3_DQS_P,
      output             HPS_DDR3_ODT,
      output             HPS_DDR3_RAS_N,
      output             HPS_DDR3_RESET_N,
      input              HPS_DDR3_RZQ,
      output             HPS_DDR3_WE_N,
      output             HPS_ENET_GTX_CLK,
      inout              HPS_ENET_INT_N,
      output             HPS_ENET_MDC,
      inout              HPS_ENET_MDIO,
      input              HPS_ENET_RX_CLK,
      input       [3:0]  HPS_ENET_RX_DATA,
      input              HPS_ENET_RX_DV,
      output      [3:0]  HPS_ENET_TX_DATA,
      output             HPS_ENET_TX_EN,
      inout              HPS_GSENSOR_INT,
      inout              HPS_I2C0_SCLK,
      inout              HPS_I2C0_SDAT,
      inout              HPS_I2C1_SCLK,
      inout              HPS_I2C1_SDAT,
      inout              HPS_KEY,
      inout              HPS_LED,
      inout              HPS_LTC_GPIO,
      output             HPS_SD_CLK,
      inout              HPS_SD_CMD,
      inout       [3:0]  HPS_SD_DATA,
      output             HPS_SPIM_CLK,
      input              HPS_SPIM_MISO,
      output             HPS_SPIM_MOSI,
      inout              HPS_SPIM_SS,
      input              HPS_UART_RX,
      output             HPS_UART_TX,
      input              HPS_USB_CLKOUT,
      inout       [7:0]  HPS_USB_DATA,
      input              HPS_USB_DIR,
      input              HPS_USB_NXT,
      output             HPS_USB_STP,
`endif /*ENABLE_HPS*/

      ///////// KEY /////////
      input       [1:0]  KEY,

      ///////// LED /////////
      output      [7:0]  LED,

      ///////// SW /////////
      input       [3:0]  SW
		
		
);


//=======================================================
//  REG/WIRE declarations
//=======================================================
// internal wires and registers declaration
  wire [1:0]  fpga_debounced_buttons;
  wire [7:0]  fpga_led_internal;
  wire        hps_fpga_reset_n;
  wire [2:0]  hps_reset_req;
  wire        hps_cold_reset;
  wire        hps_warm_reset;
  wire        hps_debug_reset;
  wire [27:0] stm_hw_events;
  
//my wires

  wire [7:0] reg1_to_add;
  wire [7:0] reg2_to_add;
  wire [7:0] add_to_reg3; 

// connection of internal logics
  assign stm_hw_events    = {{13{1'b0}},SW, fpga_led_internal, fpga_debounced_buttons};

  
  
//=======================================================
// Controls for HPS_to_FPGA FIFO
//=======================================================

reg [31:0] hps_to_fpga_readdata ; 
reg hps_to_fpga_read = 1'b0; // read command

reg hps_to_fpga_out_csr_read ; // status regs read cmd

reg[31:0] hps_to_fpga_out_csr_readdata ;

//reg [7:0] HPS_to_FPGA_state ;
//reg [31:0] data_buffer ;
//reg data_buffer_valid ;
  
  
  
  
reg oneSecClock = 1'b0;  
reg [31:0]  timecounter = 32'd25000000;  
always @(posedge FPGA_CLK1_50) 
begin 
	if(timecounter == 32'd0)
	begin
		oneSecClock <= ~oneSecClock;
		timecounter <= 32'd25000000;
	end
	else
	begin
		timecounter <= timecounter - 32'd1;
	end
end  

//=========== 取消LED预设连接 ==============
reg [7:0] LED_reg = 8'b00000000;
assign LED = LED_reg;

//=======================================================
//  PWM ADC 
//=======================================================
parameter   SAMPLE_PERIOD = 32'd1000; // 100M / 100KHz = 1000 clocks
reg [31:0]  sample_counter = 32'd0;

// 读音频，1000个clock读一下
always @(negedge PWM_ADC_CLOCK_100M) begin
	
	if(sample_counter >= (SAMPLE_PERIOD - 1)) begin
		sample_counter <= 32'd0;
		if(hps_to_fpga_out_csr_readdata > 0) begin //FIFO is not empty
			hps_to_fpga_read <= 1;
		end
	end
	else begin
		hps_to_fpga_read <= 0;
		sample_counter <= sample_counter + 32'd1;
	end
end


parameter TERMINAL = 233333;
parameter Waiting = 0, Reading = 1;
reg state;		// 当前状态
reg next;		// 下一个状态

// 读延迟
reg[31:0] delay_out_csr_readdata;
reg[31:0] delay_readdata ; 
reg delay_read = 1'b0; // read command
reg[31:0] delay_received;
reg[15:0] index;	// 数组索引
reg[31:0] PWM_delay[15:0];	// int delay[16]，改一下喇叭数量，到时候用这个储存延迟
always @(negedge PWM_ADC_CLOCK_100M) begin
	if(delay_out_csr_readdata > 0) begin
		delay_read <= 1;
	end
	else begin
		delay_read <= 0;
	end
end

always @(posedge PWM_ADC_CLOCK_100M) begin
	if(delay_read == 1'b1) begin
			delay_received <= delay_readdata[31:0];
	end
end

// 组合逻辑状态转移
always @(*) begin
	next = Waiting;
	case(state)
		Waiting: begin 
			if(delay_received == TERMINAL)
				next = Reading;
			else next = Waiting;
		end
		Reading: begin
			if(delay_received == TERMINAL)
				next = Waiting;
			else
				next = Reading;
		end
		default: next = Waiting;
	endcase
end
// 状态更新
always @(posedge PWM_ADC_CLOCK_100M) begin
	state <= next;
end

always @(posedge PWM_ADC_CLOCK_100M) begin
	case(next)
		Waiting: begin
			if(state == Waiting) begin
				// do nothing
			end
			else begin
				index <= 0;
			end
		end
		Reading: begin
			if(state == Waiting) begin
				index <= 0;
			end
			else begin
				PWM_delay[index] <= delay_received;
				index <= index + 1;
			end
		end
	endcase
end

wire PWM_ADC_CLOCK_100M;
reg [9:0] audio_sample;			// int10 的模拟信号
reg [10:0] PWM_accumulator;
reg [999999:0] queue;			// 延迟队列
wire [31:0] test_delay;
reg PWM_out = 1'b0;

assign GPIO_0[8] = PWM_out;
assign GPIO_0[1] = PWM_out;
assign GPIO_0[2] = PWM_out;
assign GPIO_1[8] = PWM_out;

assign GPIO_0[10] = ~PWM_out;
assign GPIO_0[9] = PWM_ADC_CLOCK_100M;



// 延时模块
always @(*) begin
	// 测试延时队列是否可用
	// LED_reg[0] = queue[0];
	// LED_reg[1] = queue[999999];
	// LED_reg[2] = queue[0];
	// LED_reg[3] = queue[999999];
	// LED_reg[4] = queue[0];
	// LED_reg[5] = queue[999999];
	// LED_reg[6] = queue[0];
	// LED_reg[7] = queue[999999];
	// 最终测试
	LED_reg[0] = queue[PWM_delay[0]];
	LED_reg[1] = queue[PWM_delay[2]];
	LED_reg[2] = queue[PWM_delay[4]];
	LED_reg[3] = queue[PWM_delay[6]];
	LED_reg[4] = queue[PWM_delay[8]];
	LED_reg[5] = queue[PWM_delay[10]];
	LED_reg[6] = queue[PWM_delay[12]];
	LED_reg[7] = queue[PWM_delay[14]];
end

always @(posedge PWM_ADC_CLOCK_100M) begin
	if(hps_to_fpga_read == 1'b1) begin
			audio_sample <= hps_to_fpga_readdata[9:0];
	end
end


// 1-bit 翻转
parameter   DELTA = 1000;
always @(posedge PWM_ADC_CLOCK_100M) begin

	if(PWM_accumulator >= DELTA) begin
	   PWM_out <= 1'b1;
		PWM_accumulator <= PWM_accumulator - DELTA + audio_sample;
	end
	else begin
		PWM_out <= 1'b0;
		PWM_accumulator <= PWM_accumulator + audio_sample;
	end
	// 把PWM_out送入队列
	queue <= {queue[999998:0], PWM_out};
end

//reg PWM_out_1 = 1'b0;
//assign GPIO_0[12] = PWM_out_1;
//assign GPIO_0[13] = PWM_out_1;


//=======================================================
//  Structural coding
//=======================================================


 soc_system u0 (
	  //.mybus_external_connection_export(GPIO_0[7:0]), 
	  .pio_led_external_connection_export(),
	  .pio_reg1_external_connection_export(reg1_to_add),
	  .pio_reg2_external_connection_export(reg2_to_add),
	  .pio_reg3_external_connection_export(add_to_reg3),
	  //.pio_testdelay_external_connection_export (test_delay),
 
		//Clock&Reset
	  .clk_clk                               (FPGA_CLK1_50 ),                        //  clk.clk
	  .reset_reset_n                         (1'b1         ),                        //  reset.reset_n
	  //HPS ddr3
	  .memory_mem_a                          ( HPS_DDR3_ADDR),                       //  memory.mem_a
	  .memory_mem_ba                         ( HPS_DDR3_BA),                         // .mem_ba
	  .memory_mem_ck                         ( HPS_DDR3_CK_P),                       // .mem_ck
	  .memory_mem_ck_n                       ( HPS_DDR3_CK_N),                       // .mem_ck_n
	  .memory_mem_cke                        ( HPS_DDR3_CKE),                        // .mem_cke
	  .memory_mem_cs_n                       ( HPS_DDR3_CS_N),                       // .mem_cs_n
	  .memory_mem_ras_n                      ( HPS_DDR3_RAS_N),                      // .mem_ras_n
	  .memory_mem_cas_n                      ( HPS_DDR3_CAS_N),                      // .mem_cas_n
	  .memory_mem_we_n                       ( HPS_DDR3_WE_N),                       // .mem_we_n
	  .memory_mem_reset_n                    ( HPS_DDR3_RESET_N),                    // .mem_reset_n
	  .memory_mem_dq                         ( HPS_DDR3_DQ),                         // .mem_dq
	  .memory_mem_dqs                        ( HPS_DDR3_DQS_P),                      // .mem_dqs
	  .memory_mem_dqs_n                      ( HPS_DDR3_DQS_N),                      // .mem_dqs_n
	  .memory_mem_odt                        ( HPS_DDR3_ODT),                        // .mem_odt
	  .memory_mem_dm                         ( HPS_DDR3_DM),                         // .mem_dm
	  .memory_oct_rzqin                      ( HPS_DDR3_RZQ),                        // .oct_rzqin                                  
	  //HPS ethernet		
	  .hps_0_hps_io_hps_io_emac1_inst_TX_CLK ( HPS_ENET_GTX_CLK),       					//  hps_0_hps_io.hps_io_emac1_inst_TX_CLK
	  .hps_0_hps_io_hps_io_emac1_inst_TXD0   ( HPS_ENET_TX_DATA[0] ),   					// .hps_io_emac1_inst_TXD0
	  .hps_0_hps_io_hps_io_emac1_inst_TXD1   ( HPS_ENET_TX_DATA[1] ),  					// .hps_io_emac1_inst_TXD1
	  .hps_0_hps_io_hps_io_emac1_inst_TXD2   ( HPS_ENET_TX_DATA[2] ),   					// .hps_io_emac1_inst_TXD2
	  .hps_0_hps_io_hps_io_emac1_inst_TXD3   ( HPS_ENET_TX_DATA[3] ),   					// .hps_io_emac1_inst_TXD3
	  .hps_0_hps_io_hps_io_emac1_inst_RXD0   ( HPS_ENET_RX_DATA[0] ),   					// .hps_io_emac1_inst_RXD0
	  .hps_0_hps_io_hps_io_emac1_inst_MDIO   ( HPS_ENET_MDIO ),         					// .hps_io_emac1_inst_MDIO
	  .hps_0_hps_io_hps_io_emac1_inst_MDC    ( HPS_ENET_MDC  ),         					// .hps_io_emac1_inst_MDC
	  .hps_0_hps_io_hps_io_emac1_inst_RX_CTL ( HPS_ENET_RX_DV),         					// .hps_io_emac1_inst_RX_CTL
	  .hps_0_hps_io_hps_io_emac1_inst_TX_CTL ( HPS_ENET_TX_EN),         					// .hps_io_emac1_inst_TX_CTL
	  .hps_0_hps_io_hps_io_emac1_inst_RX_CLK ( HPS_ENET_RX_CLK),        					// .hps_io_emac1_inst_RX_CLK
	  .hps_0_hps_io_hps_io_emac1_inst_RXD1   ( HPS_ENET_RX_DATA[1] ),   					// .hps_io_emac1_inst_RXD1
	  .hps_0_hps_io_hps_io_emac1_inst_RXD2   ( HPS_ENET_RX_DATA[2] ),   					// .hps_io_emac1_inst_RXD2
	  .hps_0_hps_io_hps_io_emac1_inst_RXD3   ( HPS_ENET_RX_DATA[3] ),   					// .hps_io_emac1_inst_RXD3		  
	  //HPS SD card 
	  .hps_0_hps_io_hps_io_sdio_inst_CMD     ( HPS_SD_CMD    ),           				// .hps_io_sdio_inst_CMD
	  .hps_0_hps_io_hps_io_sdio_inst_D0      ( HPS_SD_DATA[0]     ),      				// .hps_io_sdio_inst_D0
	  .hps_0_hps_io_hps_io_sdio_inst_D1      ( HPS_SD_DATA[1]     ),      				// .hps_io_sdio_inst_D1
	  .hps_0_hps_io_hps_io_sdio_inst_CLK     ( HPS_SD_CLK   ),            				// .hps_io_sdio_inst_CLK
	  .hps_0_hps_io_hps_io_sdio_inst_D2      ( HPS_SD_DATA[2]     ),      				// .hps_io_sdio_inst_D2
	  .hps_0_hps_io_hps_io_sdio_inst_D3      ( HPS_SD_DATA[3]     ),      				// .hps_io_sdio_inst_D3
	  //HPS USB 		  
	  .hps_0_hps_io_hps_io_usb1_inst_D0      ( HPS_USB_DATA[0]    ),      				// .hps_io_usb1_inst_D0
	  .hps_0_hps_io_hps_io_usb1_inst_D1      ( HPS_USB_DATA[1]    ),      				// .hps_io_usb1_inst_D1
	  .hps_0_hps_io_hps_io_usb1_inst_D2      ( HPS_USB_DATA[2]    ),      				// .hps_io_usb1_inst_D2
	  .hps_0_hps_io_hps_io_usb1_inst_D3      ( HPS_USB_DATA[3]    ),      				// .hps_io_usb1_inst_D3
	  .hps_0_hps_io_hps_io_usb1_inst_D4      ( HPS_USB_DATA[4]    ),      				// .hps_io_usb1_inst_D4
	  .hps_0_hps_io_hps_io_usb1_inst_D5      ( HPS_USB_DATA[5]    ),      				// .hps_io_usb1_inst_D5
	  .hps_0_hps_io_hps_io_usb1_inst_D6      ( HPS_USB_DATA[6]    ),      				// .hps_io_usb1_inst_D6
	  .hps_0_hps_io_hps_io_usb1_inst_D7      ( HPS_USB_DATA[7]    ),      				// .hps_io_usb1_inst_D7
	  .hps_0_hps_io_hps_io_usb1_inst_CLK     ( HPS_USB_CLKOUT    ),       				// .hps_io_usb1_inst_CLK
	  .hps_0_hps_io_hps_io_usb1_inst_STP     ( HPS_USB_STP    ),          				// .hps_io_usb1_inst_STP
	  .hps_0_hps_io_hps_io_usb1_inst_DIR     ( HPS_USB_DIR    ),          				// .hps_io_usb1_inst_DIR
	  .hps_0_hps_io_hps_io_usb1_inst_NXT     ( HPS_USB_NXT    ),          				// .hps_io_usb1_inst_NXT
		//HPS SPI 		  
	  .hps_0_hps_io_hps_io_spim1_inst_CLK    ( HPS_SPIM_CLK  ),           				// .hps_io_spim1_inst_CLK
	  .hps_0_hps_io_hps_io_spim1_inst_MOSI   ( HPS_SPIM_MOSI ),           				// .hps_io_spim1_inst_MOSI
	  .hps_0_hps_io_hps_io_spim1_inst_MISO   ( HPS_SPIM_MISO ),           				// .hps_io_spim1_inst_MISO
	  .hps_0_hps_io_hps_io_spim1_inst_SS0    ( HPS_SPIM_SS   ),             			// .hps_io_spim1_inst_SS0
		//HPS UART		
	  .hps_0_hps_io_hps_io_uart0_inst_RX     ( HPS_UART_RX   ),          				// .hps_io_uart0_inst_RX
	  .hps_0_hps_io_hps_io_uart0_inst_TX     ( HPS_UART_TX   ),          				// .hps_io_uart0_inst_TX
		//HPS I2C1
	  .hps_0_hps_io_hps_io_i2c0_inst_SDA     ( HPS_I2C0_SDAT  ),        					//  .hps_io_i2c0_inst_SDA
	  .hps_0_hps_io_hps_io_i2c0_inst_SCL     ( HPS_I2C0_SCLK  ),        					// .hps_io_i2c0_inst_SCL
		//HPS I2C2
	  .hps_0_hps_io_hps_io_i2c1_inst_SDA     ( HPS_I2C1_SDAT  ),        					// .hps_io_i2c1_inst_SDA
	  .hps_0_hps_io_hps_io_i2c1_inst_SCL     ( HPS_I2C1_SCLK  ),        					// .hps_io_i2c1_inst_SCL
		//GPIO 
	  .hps_0_hps_io_hps_io_gpio_inst_GPIO09  ( HPS_CONV_USB_N ),  							// .hps_io_gpio_inst_GPIO09
	  .hps_0_hps_io_hps_io_gpio_inst_GPIO35  ( HPS_ENET_INT_N ),  							// .hps_io_gpio_inst_GPIO35
	  .hps_0_hps_io_hps_io_gpio_inst_GPIO40  ( HPS_LTC_GPIO   ),  							// .hps_io_gpio_inst_GPIO40
	  .hps_0_hps_io_hps_io_gpio_inst_GPIO53  ( HPS_LED   ),  								// .hps_io_gpio_inst_GPIO53
	  .hps_0_hps_io_hps_io_gpio_inst_GPIO54  ( HPS_KEY   ),  								// .hps_io_gpio_inst_GPIO54
	  .hps_0_hps_io_hps_io_gpio_inst_GPIO61  ( HPS_GSENSOR_INT ),  						// .hps_io_gpio_inst_GPIO61
    
	  .hps_0_f2h_stm_hw_events_stm_hwevents  (stm_hw_events),  								//  hps_0_f2h_stm_hw_events.stm_hwevents
	  .hps_0_h2f_reset_reset_n               (hps_fpga_reset_n),   							//  hps_0_h2f_reset.reset_n
	  .hps_0_f2h_warm_reset_req_reset_n      (~hps_warm_reset),      						//  hps_0_f2h_warm_reset_req.reset_n	
	  .hps_0_f2h_debug_reset_req_reset_n     (~hps_debug_reset),     						//  hps_0_f2h_debug_reset_req.reset_n  
	  .hps_0_f2h_cold_reset_req_reset_n      (~hps_cold_reset),      						//  hps_0_f2h_cold_reset_req.reset_n
         
			
		.fifo_0_out_readdata                   (hps_to_fpga_readdata),                   //                   fifo_0_out.readdata
      .fifo_0_out_read                       (hps_to_fpga_read),                       //                             .read
      .fifo_0_out_waitrequest                (),                //                             .waitrequest
      .fifo_0_out_csr_address                (32'd0),                //               fifo_0_out_csr.address
      .fifo_0_out_csr_read                   (1'b1),                   //  是否处于读状态                           .read
      .fifo_0_out_csr_writedata              (),              //                             .writedata
      .fifo_0_out_csr_write                  (1'b0),                  // 是否处于写状态                            .write
      .fifo_0_out_csr_readdata               (hps_to_fpga_out_csr_readdata) ,               //                             .readdata
		//.clock_bridge_0_in_clk_clk             (PWM_ADC_CLOCK_100M),
		//.pwm_adc_pll_outclk0_clk                (PWM_ADC_CLOCK_100M)
		.pwm_adc_clk_out_clk_clk               (PWM_ADC_CLOCK_100M),
		//pwm_adc_clk_out_clk_clk
		
		.fifo_delay_pipe_out_readdata          (delay_readdata),          //          fifo_delay_pipe_out.readdata
      .fifo_delay_pipe_out_read              (delay_read),              //                             .read
      .fifo_delay_pipe_out_waitrequest       (),       //                             .waitrequest
      .fifo_delay_pipe_out_csr_address       (8'd0),       //      fifo_delay_pipe_out_csr.address
      .fifo_delay_pipe_out_csr_read          (1'b1),          //                             .read
      .fifo_delay_pipe_out_csr_writedata     (),     //                             .writedata
      .fifo_delay_pipe_out_csr_write         (1'b0),         //                             .write
      .fifo_delay_pipe_out_csr_readdata      (delay_out_csr_readdata)       //                             .readdata
 );
 
 // Source/Probe megawizard instance
hps_reset hps_reset_inst (
  .source_clk (),
  .source     (hps_reset_req)
);

altera_edge_detector pulse_cold_reset (
  .clk       (FPGA_CLK1_50),
  .rst_n     (hps_fpga_reset_n),
  .signal_in (hps_reset_req[0]),
  .pulse_out (hps_cold_reset)
);
  defparam pulse_cold_reset.PULSE_EXT = 6;
  defparam pulse_cold_reset.EDGE_TYPE = 1;
  defparam pulse_cold_reset.IGNORE_RST_WHILE_BUSY = 1;

altera_edge_detector pulse_warm_reset (
  .clk       (FPGA_CLK1_50),
  .rst_n     (hps_fpga_reset_n),
  .signal_in (hps_reset_req[1]),
  .pulse_out (hps_warm_reset)
);
  defparam pulse_warm_reset.PULSE_EXT = 2;
  defparam pulse_warm_reset.EDGE_TYPE = 1;
  defparam pulse_warm_reset.IGNORE_RST_WHILE_BUSY = 1;
  
altera_edge_detector pulse_debug_reset (
  .clk       (FPGA_CLK1_50),
  .rst_n     (hps_fpga_reset_n),
  .signal_in (hps_reset_req[2]),
  .pulse_out (hps_debug_reset)
);
  defparam pulse_debug_reset.PULSE_EXT = 32;
  defparam pulse_debug_reset.EDGE_TYPE = 1;
  defparam pulse_debug_reset.IGNORE_RST_WHILE_BUSY = 1;
  
  //my add
  SimpleAdd MyAdd(
	.reg1		(reg1_to_add),
	.reg2		(reg2_to_add),
	.reg3		(add_to_reg3)
	
  );

endmodule
