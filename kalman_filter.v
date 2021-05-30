//kalman filter for estimating states/filtering noise of Boost Converter to be fed into MPC-MPPT algorithm
//assumes 32 bit fixed point with 16 bit fractional
module kalman_filter(
input i_clk,
input i_rst_n,

input [47:0] i_u,
input [47:0] i_y,
input i_begin, //goes HI based on sampling rate

output reg [31:0] o_state0, //VCpv
output reg [31:0] o_state1, //IL1
output reg [31:0] o_state2, //IL2
output reg [31:0] o_state3 //VCout
	);

reg [47:0] r_A [0:3][0:3]; //A matrix
reg [47:0] r_AT[0:3][0:3]; //AT matrix
reg [47:0] r_B [0:3]; //B matrix
reg [47:0] r_X [0:3]; //X - state variables
reg [47:0] r_P [0:3][0:3]; //P - Covariance Matrix 
reg [47:0] r_Q [0:3][0:3]; // Q - Process Noise Matrix
reg [47:0] r_C [0:3]; //C matrix
reg [47:0] r_C_hold [0:3]; //C hold matrix (for multiplication)
reg [47:0] ybar; //measurement residual
reg [47:0] r_S; //innovation covariance
reg [47:0] r_R; //observation noise covariance

reg r_DV; //toggle HI when a new state estimation is made
integer i = 0;
integer j = 0;
integer k = 0;
integer l = 0;
integer m = 0;
reg r_init_system; //initialzed to 0, gets set to and stays at 1 after all data/memory is initialized
reg [3:0] r_SM;

wire signed [47:0] mult_out[0:3][0:3];
reg signed  [47:0] mult_a[0:3][0:3];
reg signed  [47:0] mult_b[0:3][0:3];
reg signed [47:0] add_out[0:3];

//general purpose blocks
assign mult_out[0][0] = mult_a[0][0]*mult_b[0];
assign mult_out[0][1] = mult_a[0][1]*mult_b[1];
assign mult_out[0][2] = mult_a[0][2]*mult_b[2];
assign mult_out[0][3] = mult_a[0][3]*mult_b[3];
assign mult_out[1][0] = mult_a[1][0]*mult_b[0];
assign mult_out[1][1] = mult_a[1][1]*mult_b[1];
assign mult_out[1][2] = mult_a[1][2]*mult_b[2];
assign mult_out[1][3] = mult_a[1][3]*mult_b[3];
assign mult_out[2][0] = mult_a[2][0]*mult_b[0];
assign mult_out[2][1] = mult_a[2][1]*mult_b[1];
assign mult_out[2][2] = mult_a[2][2]*mult_b[2];
assign mult_out[2][3] = mult_a[2][3]*mult_b[3];
assign mult_out[3][0] = mult_a[3][0]*mult_b[0];
assign mult_out[3][1] = mult_a[3][1]*mult_b[1];
assign mult_out[3][2] = mult_a[3][2]*mult_b[2];
assign mult_out[3][3] = mult_a[3][3]*mult_b[3];

//4X1 output, for 4x4 * 4x4, apply 4 times over 4 clock cycles and update the B matrix each time
assign add_out[0] = (mult_out[0][0] >>16) + (mult_out[0][1] >>16) + (mult_out[0][2] >>16) + (mult_out[0][3] >>16);
assign add_out[1] = (mult_out[1][0] >>16) + (mult_out[1][1] >>16) + (mult_out[1][2] >>16) + (mult_out[1][3] >>16);
assign add_out[2] = (mult_out[2][0] >>16) + (mult_out[2][1] >>16) + (mult_out[2][2] >>16) + (mult_out[2][3] >>16);
assign add_out[3] = (mult_out[3][0] >>16) + (mult_out[3][1] >>16) + (mult_out[3][2] >>16) + (mult_out[3][3] >>16);

always@(posedge i_clk) //keep the AT updated as a transpose of A
begin
for (l = 0; l < 4; l = l + 1)
	for (m = 0; m <4; m = m + 1)
r_AT[l][m] <= r_A[m][l];
end

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
r_C [0] <= 0;   //init to 0 for now
r_C [1] <= 0;
r_C [2] <= 0;
r_C [3] <= 0;
r_P [0][0] <= 0; //init to 0 for now
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
r_Q [0][0] <= 0; //init to 0 for now
r_Q [0][1] <= 0;
r_Q [0][2] <= 0;
r_Q [0][3] <= 0;
r_Q [1][0] <= 0;
r_Q [1][1] <= 0;
r_Q [1][2] <= 0;
r_Q [1][3] <= 0;
r_Q [2][0] <= 0;
r_Q [2][1] <= 0;
r_Q [2][2] <= 0;
r_Q [2][3] <= 0;
r_Q [3][0] <= 0;
r_Q [3][1] <= 0;
r_Q [3][2] <= 0;
r_Q [3][3] <= 0;
r_S <= 0; 
r_R <= 0; 
r_init_system <= 1;


end
else begin
case(r_SM)
0: begin
r_DV <= 0;
if(i_begin) r_SM <= 1;
else r_SM <= r_SM;
end
1:begin //State estimation/prediction -- X = AX + BU (this is the AX part)

for(i = 0; i < 4; i = i + 1) begin
for(j = 0; j < 4; j = j + 1) begin
mult_a[i][j] <= r_A[i][j];
mult_b[i] <= r_X[i];
//r_X[i] <= ((r_A[i][0]*r_X[0])>>16) + ((r_A[i][1]*r_X[1])>>16) + ((r_A[i][2]*r_X[2])>>16) + ((r_A[i][3]*r_X[3])>>16) + ((r_B[i]*i_u)>>16);
end
end
r_SM <= 2;
end
2:begin //State estimation/prediction -- X = AX + BU (this is the AX and BU part)
for (i=0; i < 4; i = i + 1) begin
r_X[i] <= add_out[i];
end
for(i = 0; i < 4; i = i + 1) begin
mult_a[0][i] <= r_B[i];
mult_b[i] <= i_u;
end
r_SM <= 3;
end
3:begin//State estimation/prediction -- X = AX + BU (this is the AX + BU part)
for(i = 0; i < 4; i = i + 1)begin
r_X[i] <= r_X[i] + add_out[0];
end
r_SM <= 4;
end
4:begin

end


//old code starts here
2:begin //Covariance estimation/prediction -- P = A*P*AT + Q (This is the A*P Part)
for (i=0; i<4; i= i+1)
	for (j= 0; j<4; j=j+1)
r_P[i][j] <= ((r_A[i][0]*r_P[0][j])>>16) + ((r_A[i][1]*r_P[1][j])>>16) + ((r_A[i][2]*r_P[2][j])>>16) + ((r_A[i][3]*r_P[3][j])>>16);
//r_P[0][0] = r_A[0][0]*r_P[0][0] + r_A[0][1]*r_P[1][0] + r_A[0][2]*r_P[2][0]+r_A[0][3]*r_P[3][0];
//r_P[0][1] = r_A[0][0]*r_P[0][1] + r_A[0][1]*r_P[1][1] + r_A[0][2]*r_P[2][1]+r_A[0][3]*r_P[3][1];
//r_P[0][2] = r_A[0][0]*r_P[0][2] + r_A[0][1]*r_P[1][2] + r_A[0][2]*r_P[2][2]+r_A[0][3]*r_P[3][2];
//r_P[0][3] = r_A[0][0]*r_P[0][3] + r_A[0][1]*r_P[1][3] + r_A[0][2]*r_P[2][3]+r_A[0][3]*r_P[3][3];
r_SM <= 3;
end
3:begin //Covariance estimation/prediction -- P = A*P*AT + Q (This is the *AT Part)
for (i=0; i<4; i= i+1)
	for (j= 0; j<4; j=j+1)
r_P[i][j] <= ((r_P[i][0]*r_AT[0][j])>>16) + ((r_P[i][1]*r_AT[1][j])>>16) + ((r_P[i][2]*r_AT[2][j])>>16) + ((r_P[i][3]*r_AT[3][j])>>16);
r_SM <=4;
end
4:begin //Covariance estimation/prediction -- P = A*P*AT + Q (This is the +Q Part)
for (i=0; i <4; i=i+1)
	for(j=0; j <4; j = j+1)
r_P[i][j] <= r_P[i][j] + r_Q[i][j];
r_SM <= 5;
end
5:begin //Update stage - prefit innovation -- ybar = y - C*X
ybar <= i_y - (((r_C[0]*r_X[0])>>16) + ((r_C[1]*r_X[1])>>16) + ((r_C[2]*r_X[2])>>16) + ((r_C[3]*r_X[3])>>16));
r_SM <= 6;
end
6:begin //Update stage - Innovation Covariance -- S = C*P*CT+R (This is the C*P Part)
for(i=0; i<4; i=i+1)
r_C_hold[i] <= ((r_C[0]*r_P[0][i])>>16) + ((r_C[1]*r_P[1][i])>>16) + ((r_C[2]*r_P[2][i])>>16) + ((r_C[3]*r_P[3][i])>>16);
r_SM <= 7;
end
7:begin //Update stage - Innovation Covariance -- S = C*P*CT+R (This is the *CT+R Part)
S = r_C_hold[0]*r_C[0] + r_C_hold[1]*r_C[1] + r_C_hold[2]*r_C[2] + r_C_hold[3]*r_C[3] + r_R;
r_SM <= 8;
end
8:begin //Update stage - Optimal Kalman Gain

end
endcase
end
end

endmodule