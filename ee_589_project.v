//EE 589 hardware in the loop model predictive control system for DC-DC converter
//assumes signed 32 bit input with 16 fractional 
//assumes I have an analog stream of data (for FPGA-in-loop purposes) that I am sampling every 15e-6 seconds

//LEFT OFF: the mult and division need fixed https://projectf.io/posts/division-in-verilog/
// add if (iref < 0) iref == .001

module ee_589_project(
input i_clk,
input i_reset_n,

input [31:0] i_Vpv,
input [31:0] i_Ipv,
input [31:0] i_Vout,

output o_MPC_switch,
output [31:0] debug,
output [31:0] debug1
	);



parameter word_size = 32;
parameter tsample = 0.000015; //66.67kHz sampling rate
parameter tclk =0.000000040; //25mhz 
reg [word_size-1:0] r_MPC_Vpv;
reg [word_size-1:0] r_MPC_Ipv;
reg [word_size-1:0] r_MPC_Vout;
reg  r_MPC_calc_DV;
reg [9:0] r_sample_cnt;

wire signed [31:0] state0;
wire signed [31:0] state1;
wire signed [31:0] state2;
wire signed [31:0] state3;
wire signed [31:0] IPV;
wire signed [31:0] IPV_plus;
wire signed [31:0] IPV_minus;
wire  KF_DV;


always@(posedge i_clk) //sample input data once every 15e-6 seconds and run it through the algorithm
begin
if(~i_reset_n) begin 
r_sample_cnt <= 0;
r_MPC_calc_DV <= 0;
r_MPC_Ipv <= 0;
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
r_MPC_Ipv <= i_Ipv;
	end
else begin 
r_sample_cnt <= r_sample_cnt + 1; 
r_MPC_calc_DV <= 0;
r_MPC_Vpv <= r_MPC_Vpv;
r_MPC_Ipv <= r_MPC_Ipv;
r_MPC_Vout <= r_MPC_Vout;
end
end
end


reg [7:0] algo_SM = 0;
always@(posedge i_clk)
begin
 //write code that passes data from KF to MPC when DV is received from KF
case(algo_SM)
0: begin
if(KF_DV) begin
	//
end
end


endcase 
end


//instantiate MPC module:

//instantiate Kalman filter module:
kalman_filter kf0(
.i_clk (i_clk),
.i_rst_n (i_rest_n),
.i_u (i_Vpv),
.i_y (i_y),
.i_DC (i_Vout), 
.o_state0 (state0),
.o_state1 (state1),
.o_state2 (state2),
.o_state3 (state3),
.o_IPV (IPV),
.o_IPV_plus (IPV_plus),
.o_IPV_minus (IPV_minus),
.o_DV (KF_DV)
	);


endmodule

