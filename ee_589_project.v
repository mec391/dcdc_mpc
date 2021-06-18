//hardware in the loop model predictive control system for DC-DC converter
//assumes signed, two's complement, 32 bit input with 16 fractional 
//assumes I have an analog stream of data (for FPGA-in-loop purposes) that I am sampling every 15e-6 seconds
`timescale 1ns/1ns
module ee_589_project_TB();
reg i_clk;
reg i_reset_n;
reg signed [31:0] i_Vpv;
reg signed [31:0] i_Vout;
wire signed [31:0] o_MPC_DC;
wire [31:0] debug;
wire [31:0] debug1;
wire signed [31:0] IPV = UUT.IPV;
wire signed [31:0] IPV_plus = UUT.IPV_plus;
wire signed [31:0] IPV_minus = UUT.IPV_minus;
wire KF_DV = UUT.KF_DV;
wire signed [31:0] r_MPC_Vpv = UUT.r_MPC_Vpv;
wire signed [31:0] r_MPC_Vout = UUT.r_MPC_Vout;
wire r_MPC_calc_DV = UUT.r_MPC_calc_DV;

ee_589_project UUT(
.i_clk (i_clk),
.i_reset_n (i_reset_n),
.i_Vpv (i_Vpv),
.i_Vout (i_Vout),
.o_MPC_DC (o_MPC_DC),
.debug (debug),
.debug1 (debug1)
);

initial begin
i_clk = 1;
i_reset_n = 0;
i_Vpv =       32'b00000000_00100011_00000000_00000000;//35
i_Vout =      32'b00000000_01010000_00000000_00000000;//80
#2
i_reset_n = 1;
end

always begin
#1 i_clk = !i_clk;
end

endmodule

module ee_589_project(
input i_clk,
input i_reset_n,

input signed [31:0] i_Vpv,
input signed [31:0] i_Vout,

output reg signed [31:0] o_MPC_DC,
output signed [31:0] debug,
output signed [31:0] debug1,
output signed [31:0] debug2,
output signed [31:0] debug3
	);

parameter word_size = 32;
parameter tsample = 0.000015; //66.67kHz sampling rate
parameter tclk =0.000000040; //25mhz 
reg [word_size-1:0] r_MPC_Vpv;
reg [word_size-1:0] r_MPC_Vout;
reg r_MPC_calc_DV;
reg [9:0] r_sample_cnt;

wire signed [31:0] state0;
wire signed [31:0] state1;
wire signed [31:0] state2;
wire signed [31:0] state3;
wire signed [31:0] IPV;
wire signed [31:0] IPV_plus;
wire signed [31:0] IPV_minus;
wire signed [31:0] IREF;
wire  KF_DV;

assign debug = IREF;
assign debug1 = IPV_plus;
assign debug2 = IPV_minus;
assign debug3 = IPV;

always@(posedge i_clk) //sample input data once every 15e-6 seconds and run it through the algorithm
begin
if(~i_reset_n) begin 
r_sample_cnt <= 0;
r_MPC_calc_DV <= 0;
r_MPC_Vout <= 0;
r_MPC_Vpv <= 0;
end
else
begin //10'd375 25Mhz clock cycles for a 66.67kHz sample update //10'd75 5Mhz clock cycles for 66.67kHz sample update
if(r_sample_cnt == 10'd75) 
begin
r_MPC_calc_DV <= 1;
r_sample_cnt <= 10'd0;
r_MPC_Vpv <= i_Vpv;
r_MPC_Vout <= i_Vout;
end
else begin 
r_sample_cnt <= r_sample_cnt + 1; 
r_MPC_calc_DV <= 0;
r_MPC_Vpv <= r_MPC_Vpv;
r_MPC_Vout <= r_MPC_Vout;
end
end
end 


wire w_MPC_DV;
wire signed [31:0] w_MPC_DC_update;
always@(posedge i_clk) //update the output duty cycle only when there is DV from MPC module
begin
if(~i_reset_n) o_MPC_DC <= 32'b00000000_00000001_00000000_00000000; //initialize duty cycle to 1
else begin
if (w_MPC_DV)
o_MPC_DC <= w_MPC_DC_update;
else o_MPC_DC <= o_MPC_DC;
end
end

//instantiate Kalman filter module:
kalman_filter kf0(
.i_clk (i_clk),
.i_rst_n (i_reset_n),
.i_u (r_MPC_Vpv),
.i_y (r_MPC_Vout),
.i_DC (o_MPC_DC), 
.i_begin (r_MPC_calc_DV),
.o_state0 (state0),
.o_state1 (state1),
.o_state2 (state2),
.o_state3 (state3),
.o_IPV (IPV),
.o_IPV_plus (IPV_plus),
.o_IPV_minus (IPV_minus),
.o_DV (KF_DV)
	);

//instantiate MPC module:
MPC_INC_COND mpc0(
.i_clk (i_clk),
.i_rst_n (i_reset_n),
.i_Vpv (r_MPC_Vpv),
.i_Ipv (IPV),
.i_Vout (state3),
.i_Ipv_plus (IPV_plus),
.i_Ipv_minus (IPV_minus),
.i_calc_DV (KF_DV),
.o_DC_control (w_MPC_DC_update),
.o_DV (w_MPC_DV),
.o_IREF (IREF)
	);

endmodule

