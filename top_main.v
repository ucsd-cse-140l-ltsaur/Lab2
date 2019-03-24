// --------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
// --------------------------------------------------------------------
// Copyright (c) 2019 by UCSD CSE 140L
// --------------------------------------------------------------------
//
// Permission:
//
//   This code for use in UCSD CSE 140L.
//   It is synthesisable for Lattice iCEstick 40HX.  
//
// Disclaimer:
//
//   This Verilog source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  
//
// --------------------------------------------------------------------
//           
//                     Lih-Feng Tsaur
//                     UCSD CSE Department
//                     9500 Gilman Dr, La Jolla, CA 92093
//                     U.S.A
//
// --------------------------------------------------------------------
//
// Revision History : 0.0

// UART of Lattice iCEstick

module uart (
         
         input wire   clk_in        ,  //etern pin defined in pcf file
         input wire   from_pc       ,  //pin 9 UART RxD (from PC to Dev)
         output wire  to_ir         ,
         output wire  sd            ,
         input wire   i_serial_data ,
         output wire  o_serial_data ,
         
         output       test1         ,
         output       test2         ,
         output       test3         ,
         output [7:0] led   
         );

// parameters (constants)
parameter clk_freq = 27'd12000000;  // in Hz for 12MHz clock

reg [26:0]  rst_count ;
wire        i_rst ;
wire        CLKOP ;
wire        CLKOS ;        

wire [7:0]  o_rx_data       ; //output from UART RX
reg  [7:0]  uart_rx_data    ; //latch UART rx data for delay pulse

wire        o_rx_data_ready ; //output from UART RX
wire        uart_rx_rdy     ; //UART RX is read
wire        uart_rx_data_rdy; //UART data is latched

wire        adder_o_rdy     ;
wire        adder_data_ready; //narrow pulse for uart tx to tx data
wire        adder_rdy       ; //wider pulse for local logic to latch in data
wire [7:0]  adder_o_data    ;

wire        i_start_tx      ;  //connect to UART TX
wire [7:0]  i_tx_data       ;  //connect to UART TX
wire        tsr_is_empty    ;  //has data tx out   
 
// internal reset generation
// no more than 0.044 sec reset high
wire [7:0] i_rst_test; 
//assign i_rst_test[7:0] = {uart_rx_data[7]^1'b0, uart_rx_data[6]^1'b1,                    0, uart_rx_data[4]^1'b0,
//                          uart_rx_data[3]^1'b0, uart_rx_data[2]^1'b0, uart_rx_data[1]^1'b1, uart_rx_data[0]^1'b1};
// --- use esc char to reset (0x1B)
assign i_rst_test[7:0] = {is_uart_rx_b7_0, is_uart_rx_b6_0, is_uart_rx_b5_0, is_uart_rx_b4_1,
                          is_uart_rx_b3_1, is_uart_rx_b2_0, is_uart_rx_b1_1, is_uart_rx_b0_1};

always @ (posedge clk_in) begin
    if (rst_count >= (clk_freq/2)) begin
	     
	    if(&i_rst_test) begin //letter is ESC 
	       //generate reset pulse to adder
	       rst_count <= 0;
	    end
    end else begin                   
            rst_count <= rst_count + 1;
    end
	
end
	
assign i_rst = ~rst_count[19] ;

// PLL instantiation
ice_pll ice_pll_inst(
     .REFERENCECLK ( clk_in        ),  // input 12MHz
     .PLLOUTCORE   ( CLKOP         ),  // output 38MHz
     .PLLOUTGLOBAL ( PLLOUTGLOBAL  ),
     .RESET        ( 1'b1  )
     );

reg [5:0] clk_count ; 
reg CLKOS ;

always @ (posedge CLKOP) begin
    if ( clk_count == 9 ) clk_count <= 0 ;
    else clk_count <= clk_count + 1 ;          //0 - 9
    end

always @ (posedge CLKOP) begin
    if ( clk_count == 9 ) CLKOS <= ~CLKOS ;    //1.9Mhz
    end

// UART RX instantiation
uart_rx_fsm uut1 (                   
     .i_clk                 ( CLKOP           ), //38MHz
     .i_rst                 ( i_rst           ),
     .i_rx_clk              ( CLKOS           ),
     .i_start_rx            ( 1'b1            ),
     .i_loopback_en         ( 1'b0            ),
     .i_parity_even         ( 1'b0            ),
     .i_parity_en           ( 1'b0            ),               
     .i_no_of_data_bits     ( 2'b10           ),  
     .i_stick_parity_en     ( 1'b0            ),
     .i_clear_linestatusreg ( 1'b0            ),               
     .i_clear_rxdataready   ( 1'b0            ),
     .o_rx_data             ( o_rx_data       ), 
     .o_timeout             (                 ),               
     .bit_sample_en         ( bit_sample_en   ), 
     .o_parity_error        (                 ),
     .o_framing_error       (                 ),
     .o_break_interrupt     (                 ),               
     .o_rx_data_ready       ( o_rx_data_ready ),
     .i_int_serial_data     (                 ),
     .i_serial_data         ( from_pc         ) // from_pc UART signal
    );

reg [3:0] count ;
reg [17:0] shift_reg1 ;  //increase by 1 to delay strobe
reg [19:0] shift_reg2 ;
wire [15:0] shift_reg1_wire;

always @ (posedge CLKOS) count <= count + 1 ; //1.9MHz/16 = 1187500 3% error from 115200

always @ (posedge CLKOP) begin  //38MHz
    if(i_rst) begin
	    shift_reg2[19:0] <= 5'h00000;
	end
	else begin
        shift_reg2[19:0] <= {shift_reg2[18:0], o_rx_data_ready} ; //38Mhz
	end
end

always @ (posedge CLKOS) begin  //1.9MHz
    if(i_rst) begin
	    shift_reg1[17:0] <= {2'b00, 4'h0000};
	end
	else begin
        shift_reg1[17:0] <= {shift_reg1[16:0], rx_rdy} ; //1.9MHz
    end		
end

//implicity defined rx_rdy as 1 bit wire
assign rx_rdy = |shift_reg2 ; //catching 0...000 (20 bit 0s)
assign uart_rx_rdy = |shift_reg1 ; //used to latch in uart rx data

//wire uart_rx_rdy4tx = shift_reg1[1]; //used to set up the mux of uart rx or local data

//delay uart_rx_data_rdy strob by 1 1.9MHz clk tick
assign shift_reg1_wire[15:0] = shift_reg1[16:1];
assign uart_rx_data_rdy = ((&i_rst_test))? 0:(|shift_reg1_wire); 
//latch in UART Rx Data
// this logic is in clk_in domain and 
// uart_rx_rdy is at CLKOS 1.9MHz from PLL
// sync the signal first
reg [1:0] uart_rx_rdy_sync_tap;

always @ (posedge clk_in)
begin
    if(i_rst)
	    uart_rx_rdy_sync_tap[1:0] <= 2'b00;
	else
	    uart_rx_rdy_sync_tap[1:0] <= {uart_rx_rdy_sync_tap[0], uart_rx_rdy};
end
wire l_uart_rx_rdy_sync = uart_rx_rdy_sync_tap[0] & ~uart_rx_rdy_sync_tap[1];
always @ (posedge l_uart_rx_rdy_sync or posedge i_rst) 
begin
        if(i_rst)
		uart_rx_data[7:0] <= 2'h00;
		else begin
        uart_rx_data[7:0] <= {o_rx_data[7:6], o_rx_data[5], o_rx_data[4:0]} ; //flip bit5 to convert to uppter case
		end
end


//---------------------------------------
// interface to 4-bit adder here
//  r1 + r2 = adder_o_data
// local variables
reg [7:0] r1, r2; //4-bit buffers
reg [1:0] adder_input_count; //internal state machine
reg adder_start;
reg adder_ctrl;
reg adder_substrate;
reg [7:0] adder_ctrl_char;
//reg adder_valid_num;

wire [7:0] adder_p_test;
wire [7:0] adder_n_test;
wire [3:0] adder_num_test;
wire adder_p_test_wire;
wire adder_n_test_wire;
wire adder_valid_num_wire;

assign is_uart_rx_b0_1 = uart_rx_data[0] ^ 1'b0;
assign is_uart_rx_b1_1 = uart_rx_data[1] ^ 1'b0;
assign is_uart_rx_b2_1 = uart_rx_data[2] ^ 1'b0;
assign is_uart_rx_b3_1 = uart_rx_data[3] ^ 1'b0;
assign is_uart_rx_b4_1 = uart_rx_data[4] ^ 1'b0;
assign is_uart_rx_b5_1 = uart_rx_data[5] ^ 1'b0;
assign is_uart_rx_b6_1 = uart_rx_data[6] ^ 1'b0;
assign is_uart_rx_b7_1 = uart_rx_data[7] ^ 1'b0;
assign is_uart_rx_b0_0 = uart_rx_data[0] ^ 1'b1;
assign is_uart_rx_b1_0 = uart_rx_data[1] ^ 1'b1;
assign is_uart_rx_b2_0 = uart_rx_data[2] ^ 1'b1;
assign is_uart_rx_b3_0 = uart_rx_data[3] ^ 1'b1;
assign is_uart_rx_b4_0 = uart_rx_data[4] ^ 1'b1;
assign is_uart_rx_b5_0 = uart_rx_data[5] ^ 1'b1;
assign is_uart_rx_b6_0 = uart_rx_data[6] ^ 1'b1;
assign is_uart_rx_b7_0 = uart_rx_data[7] ^ 1'b1;

assign adder_p_test[7:0] = {is_uart_rx_b7_0 , is_uart_rx_b6_0 , is_uart_rx_b5_1, is_uart_rx_b4_0,
                            is_uart_rx_b3_1 , is_uart_rx_b2_0 , is_uart_rx_b1_1, is_uart_rx_b0_1};

assign adder_n_test[7:0] = {is_uart_rx_b7_0 , is_uart_rx_b6_0 , is_uart_rx_b5_1, is_uart_rx_b4_0,
                            is_uart_rx_b3_1 , is_uart_rx_b2_1 , is_uart_rx_b1_0, is_uart_rx_b0_1};
					   					   
assign adder_num_test[3:0] ={is_uart_rx_b7_0, is_uart_rx_b6_0 , is_uart_rx_b5_1, is_uart_rx_b4_1};		

assign  adder_pluse_wire     = (&adder_p_test);
assign 	adder_substrate_wire = (&adder_n_test);
assign  adder_valid_num_wire = (&adder_num_test);
   
//------------------------------------------------------------------------
// use uart_rx_rdy to make sure UART Rx data is latched in
// logci is running at clk_in domain, uart_rx_data_rdy is running at 
// CLKOS 1.9MHz from PLL.  resync this signal first
reg [7:0] l_uart_rx_data_rdy_tap;
always @ (posedge clk_in) 
begin
    if(i_rst)
	    l_uart_rx_data_rdy_tap[7:0] <= 2'h00;
	else begin
	    l_uart_rx_data_rdy_tap[7:0] <= {l_uart_rx_data_rdy_tap[6:0], uart_rx_data_rdy};
	end
end
wire uart_rx_data_rdy_sync = l_uart_rx_data_rdy_tap[0] & l_uart_rx_data_rdy_tap [1] & 
                             ~l_uart_rx_data_rdy_tap[6] & ~l_uart_rx_data_rdy_tap[7];
always @ (posedge uart_rx_data_rdy_sync or posedge i_rst)
begin
    if(i_rst) begin
        adder_input_count <= 2'b00;
        adder_start <= 1'b0;
		adder_ctrl <= 1'b0;
        r1 <= 2'h00;
        r2 <= 2'h00;
    end else 
	begin 
	    if ( adder_input_count >= 2'b10) begin
	        if ( adder_pluse_wire) begin // 3th letter is '+' or '-'
				adder_substrate <= 0;
	            adder_start <= 1'b1;
		    end else 
			if ( adder_substrate_wire) begin
			    adder_substrate <= 1'b1;
		        adder_start <= 1'b1;
		    end 
			else begin
			    adder_substrate <= 0;
			end		
            adder_ctrl <= 0;
	        adder_input_count <= 2'b00;
	    end else 
		begin
	        if(adder_valid_num_wire) begin
	            if ( adder_input_count == 2'b00)
	                r1[7:0] <= uart_rx_data[7:0];
	            else if ( adder_input_count == 2'b01)
	                r2[7:0] <= uart_rx_data[7:0];					 	
		        adder_input_count <= adder_input_count + 1;
				adder_ctrl <= 0;
		    end 
			else begin 
			    adder_ctrl_char[7:0] <= uart_rx_data[7:0];
				adder_ctrl <= 1;
		        adder_input_count <= 2'b00;
			end
	        adder_start <= 0;
		end
    end
end

//-------------------- Lab2
wire [7:0] debug_led;
wire o_debug_test1;
wire o_debug_test2;
wire o_debug_test3;
wire [7:0] adder_ctrl_char_wire;
assign adder_ctrl_char_wire[7:0] = adder_ctrl_char[7:0];
wire [7:0] r1_wire;
assign r1_wire[7:0] = r1[7:0];
wire [7:0] r2_wire;
assign r2_wire[7:0] = r2[7:0];

// define input clk
wire input_clk = clk_in;  //CLKOP or clk_in or CLKOS
reg [2:0] adder_start_tap;

// generate start strob, sync to input clk
always @ (posedge input_clk)begin
    if(i_rst)
	    adder_start_tap[2:0] <= 3'b000;
	else 
	    adder_start_tap[2:0] <= {adder_start_tap[1:0], adder_start};
end
wire input_adder_start;
assign input_adder_start = adder_start_tap[0] & ~adder_start_tap[2];

Lab2_140L Lab_UT(
 .i_rst   (i_rst)                     , // reset signal
 .i_clk_in  (input_clk)               ,
 .i_data_rdy (input_adder_start)      , //(posedge)
 .i_ctrl_signal (adder_ctrl)          , 
 .i_substrate_signal (adder_substrate), 
 .i_r1    (r1_wire[7:0])                        , // 8bit number 1
 .i_r2    (r2_wire[7:0])                        , // 8bit number 1
 .i_cin   ( )                          , //carry in
 .i_ctrl (adder_ctrl_char_wire[7:0])            , //ctrl letter
 .o_sum   (adder_o_data[3:0])         ,
 .o_cout  (adder_o_data[4])           ,
 .o_rdy (adder_o_rdy)                 , //output rdy pulse, 2 i_clk_in cycles
 .o_debug_test1 (o_debug_test1)         , //output test point1
 .o_debug_test2 (o_debug_test2)         , //output test point2
 .o_debug_test3 (o_debug_test3)         , //output test point3
 .o_debug_led   (debug_led[7:0])        //output LED
);

//assign adder_o_data[7:5] 
//convert adder_o_data 000~1FF to ASCII chars @,A,B,C,...,_
wire [7:0] o_DUT2UART_data = {3'b010, adder_o_data[4:0]};
//--------------------------------  -----------------------------------------------------------
// generate adder data ready pulse.
// local variables
reg [19:0] adder_shift_reg1 ;  
reg [19:0] adder_shift_reg2 ;
wire        adder_shift_tmp;
reg adder_data_rdy;
reg adder_data_rdy_sync;
reg [1:0] adder_data_rdy_tap;
wire adder_data_rdy_tap_tst = adder_data_rdy_tap[0] & ~adder_data_rdy_tap[1];
reg [3:0] uart_tx_bit_delay_tap;
reg [1:0] uart_tx_bit_clk_posedge;
      
always @ (posedge CLKOP)  //38MHz
begin
    if(i_rst) begin
	    uart_tx_bit_clk_posedge <= 2'b00;
		uart_tx_bit_delay_tap <= 1'h0;
	    adder_data_rdy_tap[1:0] <= 2'b00;
	    adder_data_rdy      <= 1'b0;
		adder_data_rdy_sync <= 1'b0;
		adder_shift_reg2[19:0] <= 5'h00000;
	end
	else begin
	    adder_shift_reg2[19:0] <= {adder_shift_reg2[18:0], adder_data_rdy_sync};
		
		//catching the rising edge of adder_o_rdy
	    adder_data_rdy_tap[1:0] <= {adder_data_rdy_tap[0], adder_o_rdy};  
		
		//catching uart tx block i_clk_in, which is a slow ck @ 1.9MHz/16
	    uart_tx_bit_clk_posedge[1:0] <= {uart_tx_bit_clk_posedge[0], count[3]};
	    
		// latch in @ posedge of adder_o_rdy 
		// and wait for current uart tx is finished
		// ignore rising edge if preparing for tx one byte as no buffer at tx
		if(adder_data_rdy_tap_tst & !adder_shift_tmp & !adder_data_rdy)  begin
		    uart_tx_bit_delay_tap <= 1'h0;
		    adder_data_rdy <= 1'b1;
        end			
		else if(adder_shift_tmp)  begin
		    adder_data_rdy <= 1'b0;  //clear the status as only one bit is needed in the tap line
		end		
	    
		//set adder_data_rdy_sync after tx uart is idle
		if(uart_rx_rdy | !tsr_is_empty) begin //wait for tx release and tx fifo empty 
		    uart_tx_bit_delay_tap <= 1'h0;
	        adder_data_rdy_sync <= 1'b0;
	    end else
		begin
		    //wait until the previous tx is done + 2 bit cycles (3 i_clks posedge to uart_tx)
		    if(adder_data_rdy) begin  
			
			    if((uart_tx_bit_clk_posedge[0] & !uart_tx_bit_clk_posedge[1])) 
			        uart_tx_bit_delay_tap[3:0] <= {uart_tx_bit_delay_tap[2:0], 1'b1};
					
                if(uart_tx_bit_delay_tap[3])						
	                adder_data_rdy_sync <= 1'b1;
					
			end else 
			begin
			    adder_data_rdy_sync <= 1'b0;
			end
		end
	end
end

assign adder_shift_tmp = |adder_shift_reg2 ; //catching 0...000 (20 bit 0s)

//always @ (posedge CLKOP) adder_shift_reg2[19:0] <= {adder_shift_reg2[18:0], adder_data_rdy_sync} ; //38Mhz
always @ (posedge CLKOS) begin  //1.9MHz
    if(i_rst)
	    adder_shift_reg1[19:0] <= 5'h00000;
	else
        adder_shift_reg1[19:0] <= {adder_shift_reg1[18:0], adder_shift_tmp} ; //1.9MHz
end

//delay adder_rdy strob by 1 1.9MHz clk tick
wire [19:0] adder_shift_reg1_wire1 = adder_shift_reg1[19:0]; //use wire to control the starting time
assign adder_rdy = |adder_shift_reg1_wire1; //last for 20 1.9MHz clk

reg adder_tx_sm;
reg [3:0] tx_sm_count;

//count is on PLL same as adder_rdy
always @ (posedge count[3]) begin  //118KHz bit clk (16 1.9MHz clk) is enough to sample adder_rdy
    if(i_rst) begin
	    adder_tx_sm <= 1'b0;
		//db_test_point <= 1'h0;
	end
	else begin
	    if(adder_tx_sm) 
		begin
		    if(tsr_is_empty) begin
			    
				tx_sm_count [3:0] <= {tx_sm_count [2:0], 1'b0};
				
				if(tx_sm_count[3] == 1'b1)
			        adder_tx_sm <= 1'b0;
			end
		end 
		else if(adder_rdy) 
		begin
		    //db_test_point <= 1'h0;		
		    tx_sm_count [3:0] <= 1'h1;
	        adder_tx_sm <= 1'b1;
		end
	end
end
wire adder_tx_rdy_hold = adder_tx_sm;

wire [15:0] adder_shift_reg1_wire2 = adder_shift_reg1[17:2]; //use wire to control the starting time
assign adder_data_ready = |adder_shift_reg1_wire2;

wire [7:0] o_uart_data_byte;

assign o_uart_data_byte [7:0] = (adder_tx_rdy_hold)? o_DUT2UART_data[7:0] : uart_rx_data[7:0];

//determine the input -- from adder or loopback from UART RX
assign  i_start_tx = adder_data_ready | uart_rx_data_rdy;
assign	i_tx_data[7:0] = o_uart_data_byte[7:0];
//assign	i_tx_data[7:0] = uart_rx_data[7:0];
//assign  i_start_tx = uart_rx_data_rdy;

// UART TX instantiation
uart_tx_fsm uut2(                                
    .i_clk                 ( count[3]      ),   
    .i_rst                 ( i_rst         ),   
    .i_tx_data             ( i_tx_data  ),   
    .i_start_tx            ( i_start_tx    ),   
    .i_tx_en               ( 1'b1          ),   
    .i_tx_en_div2          ( 1'b0          ),   
    .i_break_control       ( 1'b0          ),   
    .o_tx_en_stop          (  ),                
    .i_loopback_en         ( 1'b0          ),   
    .i_stop_bit_15         ( 1'b0          ),   
    .i_stop_bit_2          ( 1'b0          ),   
    .i_parity_even         ( 1'b0          ),   
    .i_parity_en           ( 1'b0          ),   
    .i_no_of_data_bits     ( 2'b11         ),   
    .i_stick_parity_en     ( 1'b0          ),   
    .i_clear_linestatusreg ( 1'b0          ),   
    .o_tsr_empty           (tsr_is_empty),                
    .o_int_serial_data     (  ),                
    .o_serial_data_local   ( o_serial_data )    //pin 8 UART TXD (from Dev to PC)
    );                                          

/*
//-------------------------------------------------------------------------------
// write to IR tx
 reg [4:0] ir_tx_reg ;  
 wire ir_tx ;
 
 assign sd = 0 ;  // 0: enable  
 always @ (posedge CLKOP) ir_tx_reg[4:0] <= {ir_tx_reg[3:0], bit_sample_en} ; 
 assign ir_tx = |ir_tx_reg ;
 assign to_ir = ir_tx & ~from_pc ;
  
 // test points
 assign test1 =  to_ir ;   //data to IR 
 assign test2 =  from_pc ; //RxD
 assign test3 =  i_rst ;   //internal reset (active low for 0.5sec)
//---------------------------------------------------------------------------------
*/

// DUT module can control 3 test pins
assign test1 =  o_debug_test1;   // 
assign test2 =  o_debug_test2;   //
assign test3 =  o_debug_test3;   //
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
// LED and DEBUG
//sample and hold reg
reg [4:0] debug_test;
reg [4:0] debug_test1;
reg [4:0] debug_test2;
reg [4:0] debug_test3;

//from wrapper to adder
wire db_uart_rx_rdy = uart_rx_rdy;
wire db_uart_rx_data_rdy = uart_rx_data_rdy;
wire db_adder_input_count0 = adder_input_count[0];
wire db_adder_start = adder_start;
wire db_input_adder_start = input_adder_start;
wire db_adder_valid_num_wire = adder_valid_num_wire;

//from adder to wrapper
wire db_adder_shift_tmp = adder_shift_tmp;
wire db_adder_data_rdy = adder_data_rdy;
wire db_adder_rdy = adder_rdy;
wire db_adder_data_rdy_sync = adder_data_rdy_sync;
wire db_adder_tx_rdy_hold = adder_tx_rdy_hold;
wire db_adder_tx_sm = adder_tx_sm;

wire db_tsr_is_empty = tsr_is_empty;

/*
reg [1:0] db_tsr_is_empty_edge;
wire db_tsr_is_empty_posedge;
wire db_tsr_is_empty_negedge;

always @ (posedge CLKOS)
begin
    if(i_rst)
	    db_tsr_is_empty_edge <= 2'b00;
	else
	    db_tsr_is_empty_edge[1:0] <= {db_tsr_is_empty_edge[0], db_tsr_is_empty};
end
assign db_tsr_is_empty_posedge = db_tsr_is_empty_edge[0] & ~db_tsr_is_empty_edge[1];
assign db_tsr_is_empty_negedge = db_tsr_is_empty_edge[1] & ~db_tsr_is_empty_edge[0];
*/

wire [3:0] db_count = count[3:0];
wire [3:0] db_tx_sm_count = tx_sm_count[3:0];
/*
wire [7:0] db_uart_rx_dat_wire = uart_rx_data [7:0]
reg [7:0] db_uart_rx_char_0;
reg [7:0] db_uart_rx_char_1;
reg [7:0] db_uart_rx_char_2;

wire [3:0] db_uart_tx_bit_delay_tap = uart_tx_bit_delay_tap[3:0];
wire [1:0] db_uart_tx_bit_clk_posedge = uart_tx_bit_clk_posedge[1:0];
*/


/*
//sampling rate at 38MHz at PLL domain
//consume more pwr to catch transitions of logic
always @ (posedge CLKOP) begin  //sample at 38MHz high speek clk
    if(i_rst) begin
	    debug_test  <= 5'b00000;
		debug_test1 <= 5'b00000;
		debug_test2 <= 5'b00000;
		debug_test3 <= 5'b00000;
    end else 
	debug_test[4:0] <= {};
	
	    // test0
	    // output from DUT
		if(db_adder_shift_tmp)              //check this pulse ever goes high 
		debug_test[0] <= 1'b1; 
		debug_test[1] <= db_adder_shift_tmp;//check it goes low at the end
	    if(db_adder_rdy)
	    debug_test[2] <= 1'b1;	       //check DUT module generate a pulse, should be high
		debug_test[3] <= db_adder_rdy; //strobe, check if go low
		debug_test[4] <= db_adder_tx_rdy_hold; //mux for selecting data src to uart tx
		// test1
		// output from DUT
		
//		if(db_tsr_is_empty_posedge)
//		debug_test1[0] <= 1'b1;
//		if(db_tsr_is_empty_negedge)
//		debug_test1[1] <= 1'b1;
		
		debug_test1[0] <= db_tsr_is_empty; //check if it stay high at the end
		debug_test1[1] <= db_adder_tx_sm;
		debug_test1[2] <= db_count[2] & ~db_count[3];
		
		if(db_tx_sm_count[3] & db_tx_sm_count[2]) //3:0 4'b11xx
		debug_test1[3] <= 1'b1;
		if(db_tx_sm_count[0] & ~db_tx_sm_count[3]) //3:0 4'b0xx1
		debug_test1[4] <= 1'b1;
	    
		// test2
		// output from UART RX
//		debug_test2[0] <= db_test_point[0];
//		debug_test2[1] <= db_test_point[1];
//		debug_test2[2] <= db_test_point[2];
//		debug_test2[3] <= db_test_point[3];
//		debug_test2[4] <= db_tsr_is_empty & db_adder_tx_sm;


	    if(db_uart_rx_rdy)
		debug_test2[0] <= 1'b1;
		if(db_uart_rx_data_rdy)
		debug_test2[1] <= 1'b1;
		if(adder_valid_num_wire)
		debug_test2[2] <= 1'b1;
		if(db_adder_input_count0)
		debug_test2[3] <= 1'b1;
		if(db_adder_start)
		debug_test2[4] <= 1'b1;
		
		//test3
		//output from UART RX
        if(db_input_adder_start)
		debug_test3[0] <= 1'b1;
		//output to UART TX
		debug_test3[1] <= db_count[3] & ~db_count[2];
		debug_test3[2] <= db_count[2] & ~db_count[3];
		if(db_tx_sm_count[1] & ~db_tx_sm_count[3]) //3:0 4'b0x1x
		debug_test3[3] <= 1'b1;
		debug_test3[4] <= |db_tx_sm_count;  //should go to 4'b1111		
end
*/


//sampling rate at 118KHz at PLL domain
//used to catch slow transition or output after the operations
//input to DUT module

wire [7:0] db_DUT_in1 = r1_wire[7:0];
wire [7:0] db_DUT_in2 = r2_wire[7:0];
wire [7:0] db_DUT_in3 = adder_ctrl_char_wire[7:0]; //control char
wire       db_DUT_in4 = adder_ctrl;
wire       db_DUT_in5 = adder_substrate;

wire [7:0] db_DUT_out1 = adder_o_data[7:0];

wire db_adder_ctrl = adder_ctrl;
wire db_adder_substrate = adder_substrate;

always @ (posedge count[3]) begin  //sample at 118KHz low speek clk
    if(i_rst) begin
	    debug_test  <= 5'b00000;
		debug_test1 <= 5'b00000;
		debug_test2 <= 5'b00000;
		debug_test3 <= 5'b00000;
    end else 
	begin
	
	    // test0
	    // output from DUT
		debug_test[0] <= db_adder_shift_tmp;//check it goes low at the end
		debug_test[1] <= db_adder_rdy; //strobe, check if go low
		debug_test[2] <= db_adder_tx_rdy_hold; //mux for selecting data src to uart tx
//		if(db_tsr_is_empty_posedge)
//		debug_test[3] <= 1'b1;
//		if(db_tsr_is_empty_negedge)
//		debug_test[4] <= 1'b1;
		
		debug_test1[0] <= db_tsr_is_empty; //check if it stay high at the end
		debug_test1[1] <= db_adder_tx_sm;
		debug_test1[2] <= db_count[2] & ~db_count[3];
	    
		// test2
		// output from UART RX
//		debug_test2[0] <= db_test_point[0];
//		debug_test2[1] <= db_test_point[1];
//		debug_test2[2] <= db_test_point[2];
//		debug_test2[3] <= db_test_point[3];
//		debug_test2[4] <= db_tsr_is_empty & db_adder_tx_sm;

		
		//test3
		//output from UART RX
		debug_test3[1] <= db_count[3] & ~db_count[2];
		debug_test3[2] <= db_count[2] & ~db_count[3];
		debug_test3[4] <= |db_tx_sm_count;  //should go to 4'b1111
		
    end
end

// MUX: select the output to LEDs
wire [7:0] debug_CR_test;
assign debug_CR_test[7:0] = {is_uart_rx_b7_0 , is_uart_rx_b6_0 , is_uart_rx_b5_0, is_uart_rx_b4_0,
                             is_uart_rx_b3_1 , is_uart_rx_b2_1 , is_uart_rx_b1_0, is_uart_rx_b0_1};
assign debug_is_CR = &debug_CR_test;  

wire [7:0] debug_DEL_test; // DEL 0x7F  
assign debug_DEL_test[7:0] = {is_uart_rx_b7_0 , is_uart_rx_b6_1 , is_uart_rx_b5_1, is_uart_rx_b4_1,
                              is_uart_rx_b3_1 , is_uart_rx_b2_1 , is_uart_rx_b1_1, is_uart_rx_b0_1};
assign debug_is_DEL = &debug_DEL_test;  

wire [7:0] debug_BS_test; // 40 = 0x28 bachspace
assign debug_BS_test[7:0] = {is_uart_rx_b7_0 , is_uart_rx_b6_0 , is_uart_rx_b5_1, is_uart_rx_b4_0,
                              is_uart_rx_b3_1 , is_uart_rx_b2_0 , is_uart_rx_b1_0, is_uart_rx_b0_0};
assign debug_is_BS = &debug_BS_test;  

wire [7:0] debug_TAB_test; // 41 = 0x29 TAB
assign debug_TAB_test[7:0] = {is_uart_rx_b7_0 , is_uart_rx_b6_0 , is_uart_rx_b5_1, is_uart_rx_b4_0,
                              is_uart_rx_b3_1 , is_uart_rx_b2_0 , is_uart_rx_b1_0, is_uart_rx_b0_1};
assign debug_is_TAB = &debug_TAB_test;  
/*
wire [7:0] debug_91_test; //  91 = 0x5b [
assign debug_91_test[7:0] = {is_uart_rx_b7_0 , is_uart_rx_b6_1 , is_uart_rx_b5_0, is_uart_rx_b4_1,
                              is_uart_rx_b3_1 , is_uart_rx_b2_0 , is_uart_rx_b1_1, is_uart_rx_b0_1};
assign debug_is_91 = &debug_91_test;  

wire [7:0] debug_93_test; //  93 = 0x5d ]
assign debug_93_test[7:0] = {is_uart_rx_b7_0 , is_uart_rx_b6_1 , is_uart_rx_b5_0, is_uart_rx_b4_1,
                              is_uart_rx_b3_1 , is_uart_rx_b2_1 , is_uart_rx_b1_0, is_uart_rx_b0_1};
assign debug_is_93 = &debug_93_test;  
*/

wire [7:0] debug_wire;
assign debug_wire[7:0] = 
                         debug_is_CR?  {1'b0,1'b0, 1'b0, debug_test[4:0]}:  
                         debug_is_DEL? {1'b0,1'b0, 1'b0, debug_test1[4:0]}:
						 debug_is_BS?  {1'b0,1'b0, 1'b0, debug_test2[4:0]}: 
						 debug_is_TAB? {1'b0,1'b0, 1'b0, debug_test3[4:0]}: 
//						 debug_is_91?  {1'b0,1'b0, 1'b0, debug_test4[4:0]}: // '['
//						 debug_is_93?  {1'b0,1'b0, 1'b0, debug_test5[4:0]}: // ']'
						 debug_led[7:0];

 assign led[0] = debug_wire[0];                      
 assign led[1] = debug_wire[1];                      
 assign led[2] = debug_wire[2];                      
 assign led[3] = debug_wire[3];                      
 assign led[4] = debug_wire[4];                      

endmodule





