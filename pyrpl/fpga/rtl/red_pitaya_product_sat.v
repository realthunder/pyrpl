`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 13.12.2015 18:04:09
// Design Name: 
// Module Name: product_sat
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
/*
###############################################################################
#    pyrpl - DSP servo controller for quantum optics with the RedPitaya
#    Copyright (C) 2014-2016  Leonhard Neuhaus  (neuhaus@spectro.jussieu.fr)
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
############################################################################### 
*/


module red_pitaya_product_sat
#( parameter BITS_IN1 = 50,
   parameter BITS_IN2 = 50,
   parameter BITS_OUT = 50,
   parameter SHIFT = 10,
   // PIPELINE=0: combinational (original behaviour)
   // PIPELINE=2: two-stage registered pipeline — cuts DSP→saturation path across a
   //             register boundary so each half fits in one 8 ns clock period.
   //             Adds 2 cycles of latency; caller must account for this.
   parameter PIPELINE = 0
)
(
    input                          clk_i,       // used only when PIPELINE != 0
    input  signed [BITS_IN1-1:0]  factor1_i,
    input  signed [BITS_IN2-1:0]  factor2_i,
    output signed [BITS_OUT-1:0]  product_o,
    output                         overflow
    );

localparam PROD_BITS = BITS_IN1 + BITS_IN2;

wire signed [PROD_BITS-1:0] product;
assign product = factor1_i * factor2_i + $signed(1 << (SHIFT-1));

// Saturation mux shared by both modes (combinational, driven from either
// the raw product wire or the stage-1 pipeline register).
`define SAT_MUX(p) \
    ({p[PROD_BITS-1], |p[PROD_BITS-2:SHIFT+BITS_OUT-1]} == 2'b01) ? \
        {{1'b0,{(BITS_OUT-1){1'b1}}},1'b1} : \
    ({p[PROD_BITS-1], &p[PROD_BITS-2:SHIFT+BITS_OUT-1]} == 2'b10) ? \
        {{1'b1,{(BITS_OUT-1){1'b0}}},1'b1} : \
    {p[SHIFT+BITS_OUT-1:SHIFT],1'b0}

generate
    if (PIPELINE == 0) begin : g_comb
        // Original purely combinational path.
        assign {product_o, overflow} = `SAT_MUX(product);
    end else begin : g_pipe
        // Stage 1: raw DSP multiply only — no rounding constant added here.
        // For wide multiplies (PROD_BITS > 48), the rounding addition requires
        // a fabric carry chain that would otherwise sit on the critical path
        // between the DSP cascade and this register.
        reg signed [PROD_BITS-1:0] product_r;
        always @(posedge clk_i) product_r <= factor1_i * factor2_i;

        // Stage 2: add rounding, apply saturation, register.
        // Rounding + saturation from a register is well under one clock period.
        wire signed [PROD_BITS-1:0] product_r_rounded;
        assign product_r_rounded = product_r + $signed(1 << (SHIFT-1));

        wire [BITS_OUT:0] sat_wire;
        assign sat_wire = `SAT_MUX(product_r_rounded);

        reg [BITS_OUT:0] sat_r;
        always @(posedge clk_i) sat_r <= sat_wire;

        assign {product_o, overflow} = sat_r;
    end
endgenerate

`undef SAT_MUX

endmodule
