module MPC_INC_COND(
input i_clk,
input i_rst_n,

input signed [31:0] i_Vpv,
input signed [31:0] i_Ipv,
input signed [31:0] i_Vout,
input signed [31:0] i_Ipv_plus,
input signed [31:0] i_Ipv_minus,
input i_calc_DV,

output reg signed [31:0] o_DC_control,
output reg o_DV
);

reg signed [31:0] r_Vpv;
reg signed [31:0] r_Ipv;
reg signed [31:0] r_Vout;
reg signed [31:0] r_Vpv_t_1;
reg signed [31:0] r_Ipv_t_1;
reg signed [31:0] r_Iref_t_1; 
reg signed [31:0] r_Iref;
reg signed [31:0] r_delta_V;
reg signed [31:0] r_delta_I;
reg signed [31:0] r_Ipv_plus;
reg signed [31:0] r_Ipv_minus;
reg signed [31:0] r_cond;
reg signed [47:0] r_numerator;
reg signed [31:0] r_equation_result;
reg signed [31:0] r_Z = 32'b00000000_00000000_00000000_01000010; //.001 for incremental step of Iref
reg signed [31:0] r_DC_Z = 32'b00000000_00000000_00000000_00000111;  //.0001 for incremental step of DC
reg [7:0] r_SM = 0;
reg signed [31:0] r_G0; 
reg signed [31:0] r_G1;
wire signed [63:0] mult_out;
reg r_G0_flag;
reg r_G1_flag;
integer i = 0;
always@(posedge i_clk)
begin
case(r_SM)
0:begin
o_DV <= 0;
if(i_calc_DV) begin //update variables
r_Ipv_plus <= i_Ipv_plus;
r_Ipv_minus <= i_Ipv_minus;
r_Vpv <= i_Vpv;
r_Ipv <= i_Ipv;
r_Vout <= i_Vout;
r_Vpv_t_1 <= r_Vpv;
r_Ipv_t_1 <= r_Ipv;
r_Iref_t_1 <= r_Iref;
r_SM <= 1; 
end
else r_SM <= r_SM;
end
1: begin //compute delta values and compute needed values
r_delta_V <= r_Vpv - r_Vpv_t_1;
r_delta_I <= r_Ipv - r_Ipv_t_1;
r_SM <= 2;
end
2: begin
r_numerator[31:0] <= r_delta_I;
r_SM <= 3;
end
3: begin
r_cond <= (r_numerator << 16) / r_delta_V;	
r_SM <= 4;
end
4: begin
mult_out <= r_cond * r_Vpv;
r_SM <= 5;
end
5: begin
r_equation_result <= (mult_out >> 16) + r_Ipv;
r_SM <= 6;
end
6: begin //begin logical statement to find new value of Iref
if      (r_delta_V == 0 && r_delta_I == 0) r_Iref <= r_Iref;
else if (r_delta_V == 0 && r_delta_I > 0)  r_Iref <= r_Iref - r_Z;
else if (r_delta_V == 0 && r_delta_I < 0)  r_Iref <= r_Iref + r_Z;
else if (r_delta_V != 0 && r_equation_result == 0) r_Iref <= r_Iref;
else if (r_delta_V != 0 && r_equation_result >  0) r_Iref <= r_Iref - r_Z;
else if (r_delta_V != 0 && r_equation_result <  0) r_Iref <= r_Iref + r_Z;
r_SM <= 7;
end
7: begin //take difference between Iref and Ipreds
r_G0 <= r_Iref - r_Ipv_minus;
r_G1 <= r_Iref - r_Ipv_plus;
end
8: begin //if MSB == 1, need to take the twos comp to find positive equivalent, if not, keep how it is
if(r_G0[31] == 1'b1)
begin
r_GO_flag <= 1;
for(i = 0; i < 32; i = i+1) r_G0[i] <= ~r_G0[i];
end
if(r_G1[31] == 1'b1)
begin
r_G1_flag <= 1;
for(i=0; i < 32; i = i+1) r_G1[i] <= ~r_G1[i];
end
r_SM <= 9;
end
9: begin //add 1 to complete the 2's comp computation
if(r_G0_flag)
r_G0 <= r_G0 + 1'b1;
if(r_G1_flag)
r_G1 <= r_G1 + 1'b1;
r_G0 <= 0;
r_G1 <= 0;
r_SM <= 10;
end
10: begin //apply duty cycle adjustment
if (r_G1 < r_G0) o_DC_control <= o_DC_control + r_DC_Z;
else o_DC_control <= o_DC_control - r_DC_Z;
r_SM <= 11;
end
11: begin //make sure that the DC is between 0 and 1 and end send a DV out; end of algo
if (o_DC_control > 32'b00000000_00000001_00000000_00000000)
o_DC_control <= 32'b00000000_00000001_00000000_00000000;
if (o_DC_control < 0)
o_DC_control <= 0;
o_DV <= 1;
r_SM <= 0;
end
endcase
end

endmodule