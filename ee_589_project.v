//EE 589 hardware in the loop model predictive control system for DC-DC converter
//assume 32 bit input with 16 bit fixed point decimal
//assume I have an analog stream of data (for FIL purposes) that I am sampling every 15e-6 seconds
module ee_589_project#(
parameter word_size = 32
parameter tsample = .000015
parameter tclk =.000000040) //25mhz
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
reg 
always@(posedge i_clk) //need a counter that counts up unitl it reaches the tsample time, then take input data and assign to MPC, set DV hi
begin
if(~i_reset_n) //

else
begin
	if()
end


end

//instantiate MPC module:
.MPC mpc0(
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
parameter word_size = 32
parameter tsample = .000015
parameter L = .003
)
(
input i_clk,
input i_reset_n,

input [word_size-1:0] i_Vpv,
input [word_size-1:0] i_Ipv,
input [word_size-1:0] i_Vout,

input i_calc_DV, //goes high based on the sampling rate

output o_switch
);

reg [word_size-1:0] r_Vpv_t_1; //previous Vpv Sample
reg [word_size-1:0] r_Ipv_t_1; //previous Ipv Sample
reg [word_size-1:0] r_Dpv_t_1; //Iref from 2 timesteps ago
reg [word_size-1:0] r_Dpv; //Previous Iref Calc
reg [word_size-1:0] r_delta_V;
reg [word_size-1:0] r_delta_I;
reg [word_size-1:0] r_delta_D;
reg [4:0] r_state;

localparam Z = .01;


always@(posedge i_clk)
begin
if(~i_reset_n) //
else
begin
if(i_calc_DV)
	begin r_state <= 1;
	if(i_Ipv[31:16] > 16'd10) //needs changed if word size changes
	i_Ipv[31:16] <= 16'd1; //prevents big current spikes from breaking the algo
	end
end
case(r_state)
0:begin r_state <= r_state;end


1: begin

end

endcase 

end



endmodule