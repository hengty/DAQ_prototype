//Testbench for simulating cdaq verilog codes in modelsim
//Lattice's SB_RAM primitives are removed because I don't know how to include them
//ty@wisc.edu
//Last Update: June 04, 2018

`timescale 1ns/100ps

module testbench();
	
	wire tx;
	wire disc_trig;
	wire trig_out;
	wire adc_clk;
	reg[11:0] adc1=12'b000000000000;
	
	reg osc = 0;
	always #50 osc = !osc;		//This is 10MHz, not 12MHz
	
	reg resetn = 0;
	
	reg trig_out2=0;
	assign disc_trig = trig_out2;
	
	initial
	begin
		#1000 resetn = 1;		//Appearently need to hold reset for 1us for Lattice's primitives to work in simulation
		#1000 adc1=12'b100000000000;
		#1    trig_out2 <= 1;
		#50    adc1=12'b000000000000;
		#250  trig_out2 <= 0;
	end
	
	cdaq_top cdaq_tops(
						.ICE_CLK(osc),
						.miso(tx),		
						.disc_trig(disc_trig),	
						.adc1(adc1),						
						.adc_clk(adc_clk),
						
						.trig_out(trig_out),
						
						.resetn(resetn)
						);
	
	//Dumping data to file
	integer fileout;
	integer ticks = 0;	//Each tick is 16.67ns
	
	initial fileout = $fopen("tx.csv", "w");
	
	always@(posedge adc_clk)
	begin
		ticks <= ticks + 1;
		$fwrite(fileout, "%d, %b \n", ticks, tx);
	end
	
	



/*	//my_uart test-----------------------------------------------	
	reg[7:0] byte_to_send = 8'b11110000;
	reg      byte_en;
	wire     byte_ready;
	wire     tx;
	my_uart my_uart0(
					.clk(logiclk_B),					//10MHz
					.byte_to_send(byte_to_send),
					.byte_en(byte_en),
					.byte_ready(byte_ready),
					.tx(tx)
					);
	
	integer fileout;
	integer ticks=0;	//Each tick is 50ns
	
	initial
	begin
		fileout = $fopen("tx.csv", "w");
	end
	
	always@(posedge logiclk_B)
	begin
		ticks <= ticks + 1;
		if(byte_en) byte_en <= 0;
		if(byte_ready == 0)
		begin
			byte_en <= 1;
			byte_to_send <= 8'b00110101;
		end
		$fwrite(fileout, "%d, %b \n", ticks, tx);
	end
	//------------------------------------------------------------ */
	
	// always@(posedge logiclk)
	// begin
		// ticks <= ticks + 1;
		// $fscanf(file,"%b, %b, %b, %b, %b, %b, %b, %b, %b, %b, %b, %b, %b, %b, \n", com_adc[0], com_adc[1], com_adc[2], com_adc[3], com_adc[4], com_adc[5], com_adc[6], com_adc[7], com_adc[8], com_adc[9], com_adc[10], com_adc[11], com_adc[12], com_adc[13]);
		// // $fwrite(fileout, "%b, %b, %b, %b, %b, %b, %b, %b, %b, %b, %b, %b, \n", com_dac[0], com_dac[1], com_dac[2], com_dac[3], com_dac[4], com_dac[5], com_dac[6], com_dac[7], com_dac[8], com_dac[9], com_dac[10], com_dac[11]);
		
		// decoding_success_mon <= decoding_success_mon << 1;
		// decoding_success_mon[0] <= decoding_success;
		// if(decoding_success_mon == 2'b01) $fwrite(fileout, "\n");
		
		// if(new_bit_enable) begin
			// $fwrite(fileout, "%b", new_bit);
			// n <= n + 1;
			// if(n==7) begin
				// $fwrite(fileout, " ");
				// n<=0; end end
		
		// if(rspns_read) $fwrite(g, "%b \n", rspns);
		
		// if(data_to_nios_write) $fwrite(h, "%b \n", data_to_nios);
		
		// if(ticks == numofdata+1) begin
			// $fclose(fileout);
			// $fclose(g);
			// $fclose(h); end
			
	// end
			
endmodule
