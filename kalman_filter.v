//kalman filter for estimating states/filtering noise of Boost Converter to be fed into MPC-MPPT algorithm
module kalman_filter(
input i_clk,
input i_rst_n,

input [31:0] i_u,
input [31:0] i_y,
input i_begin, //goes HI based on sampling rate

output reg [31:0] o_state0, //VCpv
output reg [31:0] o_state1, //IL1
output reg [31:0] o_state2, //IL2
output reg [31:0] o_state3 //VCout

	);

reg [31:0] r_A [0:3][0:3]; //A matrix
reg [31:0] r_B [0:3]; //B matrix
reg [31:0] r_X [0:3]; //X - state variables
reg [31:0] r_P [0:3][0:3] //P - Covariance Matrix 

reg r_DV; //toggle HI when a new state estimation is made

always@(posedge i_clk)//update output when new estimate data available
begin
if(r_DV)begin
o_state0 <= r_X[0];
o_state1 <= r_X[1];
o_state2 <= r_X[2];
o_state3 <= r_X[3];
end
else begin
o_state0 <= o_state0;
o_state1 <= o_state1;
o_state2 <= o_state2;
o_state3 <= o_state3;
end
end

reg r_init_system; //initialzed to 0, gets set to and stays at 1 after all data/memory is initialized
reg [3:0] r_SM;
always@(posedge i_clk)
begin
if(~r_init_system) begin
r_A [0][0] <= 0; //init to 0 for now
r_A [0][1] <= 0;
r_A [0][2] <= 0;
r_A [0][3] <= 0;
r_A [1][0] <= 0;
r_A [1][1] <= 0;
r_A [1][2] <= 0;
r_A [1][3] <= 0;
r_A [2][0] <= 0;
r_A [2][1] <= 0;
r_A [2][2] <= 0;
r_A [2][3] <= 0;
r_A [3][0] <= 0;
r_A [3][1] <= 0;
r_A [3][2] <= 0;
r_A [3][3] <= 0;
r_B [0] <= 0;   //init to 0 for now
r_B [1] <= 0;
r_B [2] <= 0;
r_B [3] <= 0;
r_P [0][0] <= 0; //init to .01 for now
r_P [0][1] <= 0;
r_P [0][2] <= 0;
r_P [0][3] <= 0;
r_P [1][0] <= 0;
r_P [1][1] <= 0;
r_P [1][2] <= 0;
r_P [1][3] <= 0;
r_P [2][0] <= 0;
r_P [2][1] <= 0;
r_P [2][2] <= 0;
r_P [2][3] <= 0;
r_P [3][0] <= 0;
r_P [3][1] <= 0;
r_P [3][2] <= 0;
r_P [3][3] <= 0;


r_init_system <= 1;
end
else begin
case(r_SM)
0: if(i_begin) r_SM <= 1;
else r_SM <= r_SM;
1:begin
r_X[0] <= r_A[0][0]*r_X[0] + r_A[0][1]*r_X[1] + r_A[0][2]*r_X[2] + r_A[0][3]*r_X[3] + r_B[0]*i_u;
r_X[1] <= r_A[1][0]*r_X[0] + r_A[1][1]*r_X[1] + r_A[1][2]*r_X[2] + r_A[1][3]*r_X[3] + r_B[1]*i_u;
r_X[2] <= r_A[2][0]*r_X[0] + r_A[2][1]*r_X[1] + r_A[2][2]*r_X[2] + r_A[2][3]*r_X[3] + r_B[2]*i_u;
r_X[3] <= r_A[3][0]*r_X[0] + r_A[3][1]*r_X[1] + r_A[3][2]*r_X[2] + r_A[3][3]*r_X[3] + r_B[3]*i_u;
end
endcase
end
end

endmodule