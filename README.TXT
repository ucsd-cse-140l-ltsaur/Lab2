In this assignment, you are asked to implement a module, named Lab2_140L with the following interface. Lab2_140L module is referred by UART Wrapper module that provides the input signals.  UART Wrapper module also reads the output signals from Lab2_140L.  User is using terminal software running on PC to type in the instructions to the HW logic.  The syntax is Number1 Number2 + or Number1 Number2 -, e.g. 1+9 is represented as 19+ and 3-8 is represented as 38-.  Both Number1 and Number2 are 4-bit binary numbers chosen between 0000 and 1111.  The letters input by the terminal software is translated to the 4-bit binary numbers based on the following mapping table.
0000 <= '0'
0001 <= '1'
0010 <= '2'
0011 <= '3'
0100 <= '4'
0101 <= '5'
0110 <= '6'
0111 <= '7'
1000 <= '8'
1001 <= '9'
1010 <= ':'
1011 <= ';'
1100 <= '<'
1101 <= '='
1110 <= '>'
1111 <= '?'

From Number1 and Number2 and the operator (+ or -), the digital hardware logic gnerates a 4-bit sum and a 1-bit carry.  The 4-bit sum, (sum[3], sum[2] sum[1], sum[0]) is displayed by LED4, LED3, LED2, LED1 on the iCEstick Dev. Kit and 1-bit Carry is displayed by LED5 on the iCEstick Dev. Kit.  If sum[x] is a binary 1, LEDx will be turned on respectively; otherwise LEDx will be turned off.  Similarly if carry is a binary 1, LED5 will be turned on respectively; otherwise LED5 will be turned off.  In addition to displaying on LEDs, the hardwar logic can generate a pulse when the result is ready and UART wrapper module will output the result to the terminal software's screen.  The 5-bit results R[4:0] (which is {1-bit carry, sum[3:0]}) is mapped to letters based on the following table
00000 <= '@'
00001 <= 'A'
00010 <= 'B'
00011 <= 'C'
00100 <= 'D'
00101 <= 'E'
00110 <= 'F'
00111 <= 'G'
01000 <= 'H'
01001 <= 'I'
01010 <= 'J'
01011 <= 'K'
01100 <= 'L'
01101 <= 'M'
01110 <= 'N'
01111 <= 'O'
10000 <= 'P'
10001 <= 'Q'
10010 <= 'R'
10011 <= 'S'
10100 <= 'T'
10101 <= 'U'
10110 <= 'V'
10111 <= 'W'
11000 <= 'X'
11001 <= 'Y'
11010 <= 'Z'
11011 <= '['
11100 <= '\'
11101 <= ']'
11110 <= '^'
11111 <= '_'

The wrapper module is provided to students.  It is constructed in the following 4 verilog files: uart_ir_tx_ice.v, uart_tx_fsm.v, uart_rx_fsm.v, and ice_pll.v.  A brief description of these 4 files is provided below. 
uart_ir_tx_ice.v -- top module that interface with student's module (Lab2_140L)
uart_tx_fsm.v -- UART Tx (from iCEstick to PC)
uart_rx_fsm.v -- UART Rx (from PC to iCEstick)
ice_pll.v -- PLL control logic to generate 38MHz clock

A constraint file, named uart_pcf_sbt.pcf, is provided for students to generate the bipmap file (.hex) using Lattice iCEcube2 software.  This file define the pins of the FPGA.  The definition of pins can also be found on the schematics of iCEstick Dev Kit that is attached in the user manu of iCEstick Dev. Kit.

.project file is added that has a correct setup for iCEcube2 project.
.xcf file is added that has the correct setting for Diamond Programmer SW that is used to download FPGA bit file.

Students are ask to implement a module, name Lab2_140L that has the following interface to the wrapper.

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

This assignment contributes 10 points toward the final grade.  The distribution of 10 points is based on the completeness of the code implemented and is distributed as the following: 
  80% -- Add 4 bits i_r1[3:0] and i_r2[3:0] and store the 4-bit sum to o_sum[3:0] and 1-bit carry to o_cout. AND use o_debug_led [3:0] to displace o_sum[3:0]; AND use o_debug_led[4] to displace o_cout.
  10% -- When i_substrate_signal is high,  o_sum[3:0]  = i_r1[3:0] - i_r2[3:0]. AND use o_debug_led [3:0] to displace o_sum[3:0]; AND use o_debug_led[4] to displace o_cout.  Otherwise, o_sum[3:0]  = i_r1[3:0] + i_r2[3:0].
  5% -- Generate a pulse when the result is ready to signal the wrapper module to output the 4-bit sum and 1-bit carry to terminal screen on PC.
  5% -- define some special letters for debugging Student's implementation

