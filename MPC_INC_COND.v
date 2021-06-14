module MPC_INC_COND(
input i_clk,
input i_rst_n,

input signed [31:0] i_Vpv,
input signed [31:0] i_Ipv,
input signed [31:0] i_Vout,

input i_calc_DV,
output reg signed [31:0] o_DC_control
);
reg signed [31:0] r_Vpv_t_1;
reg signed [31:0] r_Ipv_t_1;
reg signed [31:0] r_Iref_t_1; 

reg [7:0] r_SM = 0;

wire signed [63:0] mult_out[0:3];
reg  signed [63:0] mult_a[0:3];
reg  signed [63:0] mult_b[0:3];

assign mult_out[0] = mult_a[0] * mult_b[0];
assign mult_out[1] = mult_a[1] * mult_b[1];
assign mult_out[2] = mult_a[2] * mult_b[2];
assign mult_out[3] = mult_a[3] * mult_b[3];

always@(posedge i_clk)
begin
//follow the algorithm in matlab to rewrite the design using the mult. blocks above
end


endmodule



//old algorithm:
module MPC_INC_COND(
input i_clk,
input i_reset_n,

input [31:0] i_Vpv,
input [31:0] i_Ipv,
input [31:0] i_Vout,

input i_calc_DV, //goes high based on the sampling rate

output reg o_switch,
output [31:0] debug,
output [31:0] debug1
);

reg signed [word_size-1:0] debug_reg;
reg signed [word_size-1:0] debug_reg2;
assign debug = debug_reg;
assign debug1 = debug_reg2;

parameter word_size = 32;
parameter tsample = 32'd1;//$ceil(15^-6*2^16)     //conversion to 16 bit fixed point fractional
parameter L = 32'd197; //$ceil(.003H * 2^16)   //define the bit size so that it doesn't default to 32bit signed
parameter Z = 32'd656; //$ceil(.01*2^16)

reg signed [47:0] r_Vpv;
reg signed [47:0] r_Ipv;
reg signed [47:0] r_Vout;
reg signed [word_size-1:0] r_Vpv_t_1; //previous Vpv Sample
reg signed [word_size-1:0] r_Ipv_t_1; //previous Ipv Sample
reg signed [word_size-1:0] r_Dpv_t_1; //Iref from 2 timesteps ago
reg signed [word_size-1:0] r_Dpv; //Previous Iref Calc
reg signed [word_size-1:0] r_delta_V;
reg signed [word_size-1:0] r_delta_I;
reg signed [word_size-1:0] r_delta_D;

reg [4:0] r_state;

reg signed [word_size-1:0] r_Ipred_1;
reg	signed [word_size-1:0] r_Ipred_0;
reg signed [word_size-1:0] r_Iref;
reg signed [word_size-1:0] r_G_1;
reg signed [word_size-1:0] r_G_0;

reg r_begin;
reg signed [47:0] pred_const1 = 48'b00000000_00000000_0000_0000_0000_0000_0000_0010_1000_1111; 
reg signed [47:0] pred_const0 = 48'b00000000_00000000_0000_0000_0000_0000_0000_0000_1010_0100;


reg signed [47:0] hold;
reg signed [47:0] hold1 = 48'b00000000_00000000_00001000_00101001_11001000_00000000;
reg signed [47:0] hold2 = 48'b00000000_00000000_00010000_11010010_10010000_00000000;

reg signed [47:0] r_delta_I_hold;
reg signed [47:0] r_delta_V_hold;
reg signed [47:0] r_delta_I_delta_V;
reg signed [47:0] result_0;

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

 debug_reg2 <=r_Vout; //(hold1 << 16) / hold2;
 debug_reg <= r_delta_I_delta_V; //r_delta_I_delta_V;

r_Ipred_1 <= r_Ipv + ((pred_const1 * r_Vpv)>>16); //.01
r_Ipred_0 <= (r_Ipv) + ((pred_const0 * (r_Vpv - r_Vout))>>16); //.0025 
//r_Ipred_1 <= r_Ipv + ((2 * (tsample / L)) * r_Vpv); 
//r_Ipred_0 <= r_Ipv + ((tsample / (2 * L)) * (r_Vpv - r_Vout));
r_state <= 6;
end
2: begin //Run IncCond for Iref calc - Flow chart assignment moves right to left 0 to 5
	r_state <= 9;
	result_0 <= r_Ipv + (r_delta_I_delta_V * r_Vpv)>>16;
end
3: begin //Apply reference signal to predictions
r_G_1 <= r_Iref - r_Ipred_1;
r_G_0 <= r_Iref - r_Ipred_0;
r_state <= 4;
end
4: begin //take abs value
if(r_G_1[31] == 1'b1) r_G_1 <= -r_G_1;
else r_G_1 <= r_G_1;
if(r_G_0[31] == 1'b1) r_G_0 <= -r_G_0;
else r_G_0 <= r_G_0;
//r_G_1[word_size-1] <= 0;
//r_G_0[word_size-1] <= 0;
r_state <= 5;
end
5: begin //output switch and update static variables for next computation
r_state <= 0;
r_Vpv_t_1 <= r_Vpv;
r_Ipv_t_1 <= r_Ipv;
r_Dpv_t_1 <= r_Dpv;
r_Dpv <= r_Iref;
if(r_G_1 < r_G_0) o_switch <= 1;
else o_switch <= 0;
end

6: begin //adjustments for division
r_delta_V_hold[30:0] <= r_delta_V[30:0];
r_delta_I_hold[30:0] <= r_delta_I[30:0];
r_delta_V_hold[47:31] <= 0;
r_delta_I_hold[47:31] <= 0;
if(r_delta_I[31] == 1 && r_delta_V[31] == 1) r_delta_I_delta_V[31] <= 0;
else if (r_delta_I[31] == 1 && r_delta_V[31] == 0) r_delta_I_delta_V[31] <= 1;
else if (r_delta_I[31] == 0 && r_delta_V[31] == 1) r_delta_I_delta_V[31] <= 1;
else r_delta_I_delta_V[31] <= 0;

r_state <= 7;
end
7: begin
r_delta_I_hold <= r_delta_I_hold << 16;

r_state <= 8;
end
8: begin
r_delta_I_delta_V[30:0] <= r_delta_I_hold / r_delta_V_hold;
r_state <= 2;
end

9: begin
r_state <= 3;
	//debug_reg <= (r_delta_I / r_delta_V) ;
	if     (r_delta_V == 0 && r_delta_I == 0) begin r_Iref <= r_Dpv; end//debug_reg2[31:16] <= 0; end//0
	else if(r_delta_V == 0 && r_delta_I > 0) begin r_Iref <= r_Dpv - Z; end//debug_reg2[31:16] <= 2; end//2
	else if(r_delta_V == 0 && r_delta_I < 0) begin  r_Iref <= r_Dpv + Z; end//debug_reg2[31:16] <= 1; end// 1
	else if(r_Ipv + (result_0) == 0) begin r_Iref <= r_Dpv; end//debug_reg2[31:16] <= 5; end//5
	else if(r_Ipv + (result_0) > 0) begin r_Iref <= r_Dpv - Z; end//debug_reg2 [31:16] <= 3; end//3
	else begin r_Iref <= r_Dpv + Z; end//debug_reg2 <= 4; end//4
end
endcase 
end

endmodule


endmodule