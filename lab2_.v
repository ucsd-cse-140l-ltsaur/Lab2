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

//-------------------- Lab2 ----------------------
module Lab2_140L (
 input wire i_rst           , // reset signal (active high)
 input wire i_clk_in          , //for internal state machine
 input wire i_data_rdy        , //r1, r2, OP are ready  
 input wire i_ctrl_signal     , 
 input wire i_substrate_signal,
 input wire [7:0] i_r1           , // 8bit number 1
 input wire [7:0] i_r2           , // 8bit number 1
 input wire i_cin           , // carry in
 input wire [7:0] i_ctrl         , // input ctrl char
 output wire [3:0] o_sum    ,
 output wire o_cout         ,
 output wire o_rdy          , //pulse
 output wire o_debug_test1  ,
 output wire o_debug_test2  ,
 output wire o_debug_test3  ,
 output wire [7:0] o_debug_led   
);

//------------ Add your adder here ----------

endmodule
         

