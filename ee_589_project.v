//EE 589 hardware in the loop model predictive control system for DC-DC converter
//assumes signed 32 bit input with 16 fractional -- SHOULD PROBABLY BE CHANGED TO 20 BIT FRACTIONAL, leaving 12 bits for whole number, when signed goes up to ~+-2k whole numbers
//assumes I have an analog stream of data (for FPGA-in-loop purposes) that I am sampling every 15e-6 seconds
module ee_589_project#(
parameter word_size = 32,
parameter tsample = 0.000015, //66.67kHz sampling rate
parameter tclk =0.000000040) //25mhz 
(
input i_clk,
input i_reset_n,

input [word_size-1:0] i_Vpv,
input [word_size-1:0] i_Ipv,
input [word_size-1:0] i_Vout,

output o_MPC_switch
	);

reg [word_size-1:0] r_MPC_Vpv;
reg [word_size-1:0] r_MPC_Ipv;
reg [word_size-1:0] r_MPC_Vout;
reg [word_size-1:0] r_MPC_calc_DV;
reg [9:0] r_sample_cnt;

always@(posedge i_clk) 
begin
if(~i_reset_n) begin 
r_sample_cnt <= 0;
r_MPC_calc_DV <= 0;
r_MPC_Ipv <= 0;
r_MPC_Vout <= 0;
r_MPC_Vpv <= 0;
end
else
begin //375 25Mhz clock cycles for a 66.67kHz sample update
if(r_sample_cnt == 10'd375) 
begin
r_MPC_calc_DV <= 1;
r_sample_cnt <= 10'd0;
r_MPC_Vpv <= i_Vpv;
r_MPC_Vout <= i_Vout;
	if(i_Ipv[31:16] > 16'd10) //needs changed if word size changes
	begin r_MPC_Ipv[31:16] <= 16'd1; r_MPC_Ipv[15:0] <= i_Ipv[15:0]; end//prevents big current spikes from breaking the algo
	else r_MPC_Ipv <= i_Ipv;
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
//instantiate MPC module:
MPC mpc0(
.i_clk(i_clk),
.i_reset_n (i_reset_n),
.i_Vpv (r_MPC_Vpv),
.i_Ipv (r_MPC_Ipv),
.i_Vout (r_MPC_Vout),
.i_calc_DV(r_MPC_calc_DV),
.o_switch(o_MPC_switch)

	);
//instantiate Kalman filter module:

//instantate reinforcement learning module:

endmodule


module MPC#(
parameter word_size = 32,
parameter tsample = 32'd1,//$ceil(.000015)
parameter L = 32'd197, //$ceil(.003H * 2^16)   //define the bit size so that it doesn't default to 32bit signed
parameter Z = 32'd656 //$ceil(.01*2^16)
)
(
input i_clk,
input i_reset_n,

input [word_size-1:0] i_Vpv,
input [word_size-1:0] i_Ipv,
input [word_size-1:0] i_Vout,

input i_calc_DV, //goes high based on the sampling rate

output reg o_switch
);


reg [word_size-1:0] r_Vpv;
reg [word_size-1:0] r_Ipv;
reg [word_size-1:0] r_Vout;
reg [word_size-1:0] r_Vpv_t_1; //previous Vpv Sample
reg [word_size-1:0] r_Ipv_t_1; //previous Ipv Sample
reg [word_size-1:0] r_Dpv_t_1; //Iref from 2 timesteps ago
reg [word_size-1:0] r_Dpv; //Previous Iref Calc
reg signed[word_size-1:0] r_delta_V;
reg signed [word_size-1:0] r_delta_I;
reg signed[word_size-1:0] r_delta_D;

reg [4:0] r_state;

reg [word_size-1:0] r_Ipred_1;
reg	[word_size-1:0] r_Ipred_0;
reg [word_size-1:0] r_Iref;
reg signed[word_size-1:0] r_G_1;
reg signed[word_size-1:0] r_G_0;

reg r_begin;

always@(posedge i_clk)
begin
if(~i_reset_n)
begin
r_Vpv <= 0;
r_Ipv <= 0;
r_Vout <= 0;
r_Vpv_t_1 <= 0;
r_Ipv_t_1 <= 0;
r_Dpv_t_1 <= 0;
r_Dpv <= 0;
r_delta_V <= 0;
r_delta_I <= 0;
r_delta_D <= 0;
r_state <= 0;
r_begin <= 0;
end
else
begin
if(i_calc_DV) begin 
r_begin <= 1;
r_Ipv <= i_Ipv;
r_Vpv <= i_Vpv;
r_Vout <= i_Vout;
end
else begin
r_begin <= 0;
r_Ipv <= r_Ipv;
r_Vpv <= r_Vpv;
r_Vout <= r_Vout;
end
end
//MPC Algo//
case(r_state)
0:begin
if(r_begin) r_state <= 1;
else r_state <= r_state;
end
1: begin //update state info, perform MPC predictions:
r_delta_V <= r_Vpv - r_Vpv_t_1;
r_delta_I <= r_Ipv - r_Ipv_t_1;
r_delta_D <= r_Dpv - r_Dpv_t_1;

r_Ipred_1 <= r_Ipv + 2 * (tsample / L) * r_Vpv;
r_Ipred_0 <= r_Ipv + tsample / (2 * L) * (r_Vpv - r_Vout);
r_state <= 2;
end
2: begin //Run IncCond for Iref calc - Flow chart assignment moves right to left 0 to 5
	r_state <= 3;
	if     (r_delta_V == 0 && r_delta_I == 0) r_Iref <= r_Dpv; //0
	else if(r_delta_V == 0 && r_delta_I > 0) r_Iref <= r_Dpv - Z; //2
	else if(r_delta_V == 0 && r_delta_I < 0) r_Iref <= r_Dpv + Z; // 1
	else if(r_Ipv + (r_delta_I / r_delta_V) * r_Vpv == 0) r_Iref <= r_Dpv; //5
	else if(r_Ipv + (r_delta_I / r_delta_V) * r_Vpv > 0) r_Iref <= r_Dpv - Z; //3
	else r_Iref <= r_Dpv + Z; //4
end
3: begin //Apply reference signal to predictions
r_G_1 <= r_Iref - r_Ipred_1;
r_G_0 <= r_Iref - r_Ipred_0;
r_state <= 4;
end
4: begin //take abs value
r_G_1[word_size-1] = 0;
r_G_0[word_size-1] = 0;
r_state <= 5;
end
5: begin
r_state <= 0;
if(r_G_1 < r_G_0) o_switch <= 1;
else o_switch <= 0;
end
endcase 
end

endmodule