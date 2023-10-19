`timescale 1ns / 1ps
//Verilog modules for cdaq firmware on the iCE40UP5K-B-EVN evaluation board. FPGA model number: iCE40UP5K - SG48
//Drive and buffer adc[11:0] at 60 MHz.
//A waveform starts several clock cycles (pre_trig) before external discriminator goes up.
//A waveform ends after disc goes and stay down for several clock cycles (post_trig).
//Save waveforms into data and header fifos, including pre and post triggger.
//Send those waveforms via uart when possible.
//ty@wisc.edu
//Last Update: August 10, 2018


//----------------------------------------***Main Module***-----------------------------------------------//
module cdaq_top(
	input ICE_CLK,			//Pin 35 , 12MHz oscillator (to be multiplied by 5)
	// output LED_BLUE,		//Pin 39
	// output LED_GREEN,		//Pin 40
	// output LED_RED,			//Pin 41
	//input  ss,				//Pin 2  (IOB_6A)
	//input  mosi,			//Pin 46 (IOB_0A)
	output miso,			//Pin 47 (IOB_2A)
	//input  sclk,			//Pin 44 (IOB_3B_G6)	2MHz?
	input disc_trig,		//Pin 4 (IOB_8A)
	input[11:0] adc1,		//Pin 28, 38(50B), 42, 36, 43, 34, 31, 32, 27, 26, 25, 23
	//input[11:0] adc2,		//Pin
	input resetn,			//active low, Pin 45 (IOB_5B)
	output trig_out,		//Pin 3 (IOB_9B)
	input clk_from_adc,		//Pin 20 (IOB_25B_G3)
	output clk_to_adc			//Pin 44 (IOB_3B_G6)
	);
	
	//Parameters--------------------------------------------------------------
	parameter post_trig_len = 5;
	parameter pre_trig_len  = 15;
	//------------------------------------------------------------------------
	
	//--PLL, input: 12MHz external oscillator, outputs: x10 and x5------------
	wire clk_A;				//12MHz x 10 = 120MHz
	wire clk_B;				//clk_A/2  = 60MHz
	SB_PLL40_2F_PAD main_clks(.PACKAGEPIN(ICE_CLK),
								.PLLOUTGLOBALA(clk_A),
								.PLLOUTGLOBALB(clk_B),
								.BYPASS(1'b0),
								.RESETB(resetn)
								);
	//\\ Fin=12, Fout=120;	(DIVF+1)/(2^DIVQ *(DIVR+1))
	defparam main_clks.DIVR = 4'b0000;
	defparam main_clks.DIVF = 7'b1001111;
	defparam main_clks.DIVQ = 2'b11;
	defparam main_clks.FILTER_RANGE = 3'b001;
	defparam main_clks.FEEDBACK_PATH = "SIMPLE";
	defparam main_clks.DELAY_ADJUSTMENT_MODE_FEEDBACK = "FIXED";
	defparam main_clks.FDA_FEEDBACK = 4'b0000;
	defparam main_clks.DELAY_ADJUSTMENT_MODE_RELATIVE = "FIXED";
	defparam main_clks.FDA_RELATIVE = 4'b0000;
	defparam main_clks.SHIFTREG_DIV_MODE = 2'b00;
	defparam main_clks.PLLOUT_SELECT_PORTA = "GENCLK";
	defparam main_clks.PLLOUT_SELECT_PORTB = "GENCLK_HALF";
	defparam main_clks.ENABLE_ICEGATE_PORTA = 1'b0;
	defparam main_clks.ENABLE_ICEGATE_PORTB = 1'b0;
	//------------------------------------------------------------------------
	
	
	
	//Sequential always@ block runs on clk_A-------------------------------------------------------
	reg[pre_trig_len-1:0] adc1_buff0, adc1_buff1, adc1_buff2, adc1_buff3,
							adc1_buff4, adc1_buff5, adc1_buff6, adc1_buff7,
							adc1_buff8, adc1_buff9, adc1_buff10, adc1_buff11;
	reg[3:0] post_trig=0;				//Keeps track of how many post triggered data left to save
	reg[9:0] data_len=0;
	reg      write_data1;
	reg      end_waveform;
	reg      write_header1;
	wire     keeping = disc_trig || (post_trig > 0);
	always@(posedge clk_A)
	begin
		
		//Start storing waveform when disc_trig goes up
		if(disc_trig) post_trig <= post_trig_len + pre_trig_len;	//When discriminator is triggered, reload post_trig, which also keeps keeping on
		
		//Keeps track of how many samples have been loaded into fifo_data
		if(keeping)
		begin
			write_data1 <= 1;
			data_len <= data_len + 1;
			if(!disc_trig) post_trig <= post_trig - 4'b0001;	//Count down to closure once discriminator falls back down
			if(post_trig == 4'b001) end_waveform <= 1;			//If the countdown is allowed to reach 1, then the waveform is ended
		end
		else write_data1 <= 0;	//Need to make write_data1 stay on for one extra cycle because the fifo writes on negedge of wclk
		
		//Takes care of finalizing waveform storage: saving time_counter value and length of waveform to fifo_header
		if(end_waveform)
		begin
			end_waveform  <= 0;			//This block only runs one clock cycle per waveform saved
			write_header1 <= 1;
		end
		if(write_header1) begin
			data_len      <= 0;			//Reset data_len after header of waveform written
			write_header1 <= 0;
		end
		
	end
	//---------------------------------------------------------------------------------------------
	
	//digital buffer runs on clk output from adc---------------------------------------------------
	always@(posedge clk_from_adc)
	begin
		adc1_buff0 <= {adc1[0],adc1_buff0[pre_trig_len-1:1]};
		adc1_buff1 <= {adc1[1],adc1_buff1[pre_trig_len-1:1]};
		adc1_buff2 <= {adc1[2],adc1_buff2[pre_trig_len-1:1]};
		adc1_buff3 <= {adc1[3],adc1_buff3[pre_trig_len-1:1]};
		adc1_buff4 <= {adc1[4],adc1_buff4[pre_trig_len-1:1]};
		adc1_buff5 <= {adc1[5],adc1_buff5[pre_trig_len-1:1]};
		adc1_buff6 <= {adc1[6],adc1_buff6[pre_trig_len-1:1]};
		adc1_buff7 <= {adc1[7],adc1_buff7[pre_trig_len-1:1]};
		adc1_buff8 <= {adc1[8],adc1_buff8[pre_trig_len-1:1]};
		adc1_buff9 <= {adc1[9],adc1_buff9[pre_trig_len-1:1]};
		adc1_buff10 <= {adc1[10],adc1_buff10[pre_trig_len-1:1]};
		adc1_buff11 <= {adc1[11],adc1_buff11[pre_trig_len-1:1]};
	end
	//---------------------------------------------------------------------------------------------
	
	//-----Generate slow clock------------------------------------------
 	reg[3:0] slow_clk_counter = 4'b0;
	reg slow_clk = 0;
	always@(posedge clk_A)
	begin
		slow_clk_counter <= slow_clk_counter + 1;
		if(slow_clk_counter == 4'b0110) slow_clk <= 1;
		if(slow_clk_counter == 4'b1100)
		begin
			slow_clk <= 0;
			slow_clk_counter <= 4'b0;
		end
	end
	//------------------------------------------------------------------
	//adc's clock driven by either clk_A or slow_clk
	assign clk_to_adc = keeping ? clk_A : slow_clk;
	
	reg[24:0] trig_counter=25'b0;
	reg       trig_state=0;
	assign trig_out = trig_state;
	always@(posedge clk_B)
	begin
		trig_counter <= trig_counter + 1;
		if(trig_counter == 25'b1110010011100001101101100) trig_state <= 1;
		if(trig_counter == 25'b1110010011100001110000000)
		begin
			trig_counter <= 25'b0;						//Counter is 1Hz?
			trig_state <= 0;							//Trigger for 20 ticks
		end
	end
	//-------------------------------------------------------------------------------
	
	
	//Data and Header FIFOs--------------------------------------------------------------
	wire[11:0] data_in1 = {adc1_buff11[0], adc1_buff10[0], adc1_buff9[0], adc1_buff8[0], 
							adc1_buff7[0], adc1_buff6[0], adc1_buff5[0], adc1_buff4[0], 
							adc1_buff3[0], adc1_buff2[0], adc1_buff1[0], adc1_buff0[0]};
	wire[11:0] data1_out;
	reg        read_data1;
	wire[9:0]  adc1_fifo_lvl;
	reg [14:0] time_counter=0;		//~1ms total at 30MHz
	wire[15:0] header1_in = {time_counter[14:9], data_len};
	wire[15:0] header1_out;
	reg        read_header1;
	wire[ 7:0] header1_fifo_lvl;
	fifo_data adc1_fifo(.wclk(clk_A),
						.resetn(resetn),
						.data_in(data_in1),
						.write_en(write_data1),
						.rclk(clk_B),
						.data_out(data1_out),
						.read_en(read_data1),
						.fill_lvl(adc1_fifo_lvl));
	fifo_header header1_fifo(.wclk(clk_A),
							.resetn(resetn),
							.data_in(header1_in),
							.write_en(write_header1),
							.rclk(clk_B),
							.data_out(header1_out),
							.read_en(read_header1),
							.fill_lvl(header1_fifo_lvl));
	//-----------------------------------------------------------------------------------

	
	//Communication Logics----------------------------------------------------------------------------------------------
	reg[23:0] data_buffer;				//24 = 12*3 = 8*4
	reg[ 9:0] num_data_read;
	reg[ 2:0] readout_machine;
	reg       byte_en;
	reg       another_data;
	reg       quit_tx;
	wire[7:0] byte_to_send;
	wire      uart_ready;
	my_uart my_uart0(
					.clk(clk_B),					//60MHz
					.byte_to_send(byte_to_send),
					.byte_en(byte_en),				//pulse one clk of this to load in byte_to_send
					.ready(uart_ready),				//uart_ready means can load in another byte to byte_to_send
					.tx(miso));
	
	assign byte_to_send = (readout_machine == 3'b001) ? header1_out[ 7: 0]:
						  (readout_machine == 3'b010) ? header1_out[15: 8]:
						  (readout_machine == 3'b011) ? data_buffer[ 7: 0]:
						  (readout_machine == 3'b100) ? data_buffer[15: 8]:data_buffer[23:16];
	
	always@(posedge clk_B)
	begin
	
		//Really coarse time counter ~1ms
		time_counter <= time_counter + 1;
		
		//State machine for sending out data to microcontroller
		if(read_header1) read_header1 <= 0;
		if(read_data1)
		begin
			if(!another_data) data_buffer[11: 0] <= data1_out;		//Store popped data in buffer
			if(another_data)  data_buffer[23:12] <= data1_out;		//Store possibly popped another-data in buffer
			another_data  <= !another_data;
			num_data_read <= num_data_read + 1;
			read_data1    <= 0;
		end
		if(uart_ready)
		case(readout_machine)
		default:if(header1_fifo_lvl > 0)				//Get to work when header fifo not empty
				begin
					byte_en         <= 1;				//This switch allows the tx machine to work
					read_header1    <= 1;				//Pop header
					num_data_read   <= 0;				//Reset num_data_read at start of new waveform readout
					another_data    <= 0;
					quit_tx         <= 0;
					readout_machine <= 3'b001;			//This connects first part of header to byte_to_send (LSB first)
				end
		3'b001: readout_machine <= 3'b010;				//This connects second part of header to byte_to_send
		3'b010: begin
					read_data1      <= 1;				//Pop a data (12 bits)
					readout_machine <= 3'b011;			//This connects first 2/3 of data to byte_to_send
				end
		3'b011: begin
					if(num_data_read < header1_out[9:0]) read_data1 <= 1;	//Pop another data if have it
					else quit_tx <= 1;										//If not have it, quit at 3'b100
					readout_machine <= 3'b100;			//This connects the last 1/3 of data and possibly first 1/3 of another-data
				end
		3'b100: begin
					if(quit_tx)
					begin
						byte_en         <= 0;
						readout_machine <= 3'b000;
					end
					else readout_machine <= 3'b101;
				end
		3'b101: begin
					if(num_data_read < header1_out[9:0])
					begin
						read_data1 <= 1;				//Pop data for next round if have it
						readout_machine <= 3'b011;
					end
					else								//If not have it, quit
					begin
						byte_en         <= 0;
						readout_machine <= 3'b000;
					end
				end
		endcase
		
	end
	//------------------------End of Communication Logics---------------------------------------------------------------
	
endmodule


//----------------------------------------***END OF MAIN MODULE***----------------------------------------//




//uart -- tx only -- 2Mbaud
module my_uart(
		input clk,					//60MHz
		input[7:0] byte_to_send,
		input byte_en,
		output reg ready = 1,		//For more byte_en
		output reg tx = 1);
	reg[3:0] tx_counter;
	reg[4:0] two_MHz_counter;	//Run only when transmitting
	reg[8:0] uart_reg;
	
	localparam[4:0] baud_setting = 29;		//baud rate = clk/(baud_setting + 1)
	
	always@(posedge clk)
	begin
		
		//Ready state - ready for byte_en
		if(ready && byte_en)
		begin
			uart_reg        <= {byte_to_send, 1'b0};
			two_MHz_counter <= 0;
			tx_counter      <= 0;
			ready           <= 0;
		end
		
		//Not ready state
		if(!ready)					//Do things to become ready again
		begin
			two_MHz_counter <= two_MHz_counter + 1'b1;
			if(two_MHz_counter == baud_setting)
			begin
				two_MHz_counter <= 0;
				if(tx_counter < 9)							//tx machine. 10 total bits sent per trigger
				begin
					tx_counter <= tx_counter + 1'b1;
					tx         <= uart_reg[tx_counter];		//Send bit, LSB first, unless when tx_counter is zero. In that case, uart start-bit is sent (zero)
				end
				if(tx_counter == 9)
				begin
					tx         <= 1;
					ready      <= 1;
				end
			end
		end
	end
endmodule


//Data FIFO module
module fifo_data(
	input        wclk,
	input        rclk,
	input        resetn,
	input[11:0]  data_in,
	input        write_en,
	output[11:0] data_out,
	input        read_en,
	output[9:0]  fill_lvl
	);
	
	reg[9:0] write_addr = 0;
	reg[9:0] read_addr  = 0;
	assign fill_lvl = write_addr - read_addr;
	
	//For managing writing to EBR
	always@(negedge wclk or negedge resetn)
	begin
		if(!resetn) write_addr <= 0;
		else if(write_en) write_addr <= write_addr + 1;
	end
	
	//For managing reading from EBR
	always@(negedge rclk or negedge resetn)
	begin
		if(!resetn) read_addr <= 0;
		else if(read_en && (read_addr < write_addr)) read_addr <= read_addr + 1;
	end
	
	//12-bit width not an option, thus use three 4-bit width EBR blocks
	SB_RAM1024x4NRNW ram1024x4_inst2(					//Note: Updates at negedge clk
								.RDATA(data_out[11:8]),
								.RADDR(read_addr),
								.RCLKN(rclk),
								.RCLKE(read_en),
								.RE(read_en),
								.WADDR(write_addr),
								.WCLKN(wclk),
								.WCLKE(write_en),
								.WDATA(data_in[11:8]),
								.WE(write_en)
								);
	SB_RAM1024x4NRNW ram1024x4_inst1(					//Note: Updates at negedge clk
								.RDATA(data_out[7:4]),
								.RADDR(read_addr),
								.RCLKN(rclk),
								.RCLKE(read_en),
								.RE(read_en),
								.WADDR(write_addr),
								.WCLKN(wclk),
								.WCLKE(write_en),
								.WDATA(data_in[7:4]),
								.WE(write_en)
								);
	SB_RAM1024x4NRNW ram1024x4_inst0(					//Note: Updates at negedge clk
								.RDATA(data_out[3:0]),
								.RADDR(read_addr),
								.RCLKN(rclk),
								.RCLKE(read_en),
								.RE(read_en),
								.WADDR(write_addr),
								.WCLKN(wclk),
								.WCLKE(write_en),
								.WDATA(data_in[3:0]),
								.WE(write_en)
								);
endmodule

//Header FIFO module, dual clocks
module fifo_header(
	input        wclk,
	input        rclk,
	input        resetn,
	input[15:0]  data_in,
	input        write_en,
	output[15:0] data_out,
	input        read_en,
	output[7:0]  fill_lvl
	);
	
	reg[7:0] write_addr = 0;
	reg[7:0] read_addr  = 0;
	assign fill_lvl = write_addr - read_addr;
	
	//For managing writing to EBR
	always@(negedge wclk or negedge resetn)
	begin
		if(!resetn) write_addr <= 0;
		else if(write_en) write_addr <= write_addr + 1;
	end
	
	//For managing reading from EBR
	always@(negedge rclk or negedge resetn)
	begin
		if(!resetn) read_addr <= 0;
		else if(read_en && (read_addr < write_addr)) read_addr <= read_addr + 1;
	end
	
	//
	SB_RAM256x16NRNW ram256x16_inst0(					//Note: Updates at negedge clk
								.RDATA(data_out),
								.RADDR(read_addr),
								.RCLKN(rclk),
								.RCLKE(read_en),
								.RE(read_en),
								.WADDR(write_addr),
								.WCLKN(wclk),
								.WCLKE(write_en),
								.WDATA(data_in),
								.WE(write_en),
								.MASK(16'b0)
								);
endmodule