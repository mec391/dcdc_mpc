//notes: 32' * 32' = 64'; Q16 causes 64' >> 16 = 48'; 48' + 48' = 49'; 49' + 48' = 50'; 50' + 48' = 51'?
//need to go through testbench and make sure all results are properly displaying to make sure the error in the result is a resolution problem and not a truncate or math problem
//try to fix the division operator if time is permitting
//redo the top module for feeding data into Kalaman Filter

//state1 and state2 need commented out throughout the code to meet pinout for module simulation

`timescale 1ns/1ns
module kalman_filter_TB();
reg i_clk;
reg i_rst_n;
reg signed [31:0] i_u; //Vpv
reg signed [31:0] i_y; //Vout
reg signed [31:0] i_DC;
reg i_begin;

wire signed [31:0] o_state0;
wire signed [31:0] o_state1;
wire signed [31:0] o_state2;
wire signed [31:0] o_state3;
wire signed [31:0] o_IPV;
wire signed [31:0] o_IPV_minus;
wire signed [31:0] o_IPV_plus;
wire o_DV;

wire [7:0] debug_SM = UUT.r_SM;
wire signed [50:0] r_X0 = UUT.r_X[0];
wire signed [50:0] r_X1 = UUT.r_X[1];
wire signed [50:0] r_X2 = UUT.r_X[2];
wire signed [50:0] r_X3 = UUT.r_X[3];

wire signed [63:0] mult_out0 = UUT.mult_out[0][0];
wire signed [63:0] mult_out1 = UUT.mult_out[0][1];
wire signed [63:0] mult_out2 = UUT.mult_out[0][2];
wire signed [63:0] mult_out3 = UUT.mult_out[0][3];
wire signed [31:0] r_A00 = UUT.r_A[0][0];
wire signed [31:0] r_A01 = UUT.r_A[0][1];
wire signed [31:0] r_A02 = UUT.r_A[0][2];
wire signed [31:0] r_A03 = UUT.r_A[0][3];
wire signed [31:0] r_DC_use = UUT.r_DC_use;

kalman_filter UUT(
.i_clk (i_clk),
.i_rst_n (i_rst_n),
.i_u (i_u),
.i_y (i_y),
.i_DC(i_DC),
.i_begin (i_begin),
.o_state0 (o_state0),
.o_state1 (o_state1),
.o_state2 (o_state2),
.o_state3 (o_state3),
.o_IPV (o_IPV),
.o_IPV_plus (o_IPV_plus),
.o_IPV_minus (o_IPV_minus),
.o_DV (o_DV)
	);

initial begin
i_clk = 1;
i_u = 0;
i_y = 0;
i_rst_n = 1;


#2
i_u =  32'b0000000000100011_0000000000000000; //35.0V
i_y =  32'b0000000001010000_0000000000000000; //80.0V
i_DC = 32'b0000000000000000_1000000000000000; //.5 DC
#4
i_begin = 1;
#2
i_begin = 0;
end

always begin
#1 i_clk = !i_clk;
end

endmodule

//kalman filter for estimating states/filtering noise of Boost Converter to be fed into MPC-MPPT algorithm
//assumes 32 bit signed 2's compliment fixed point with 16 bit fractional, msb signed
module kalman_filter(
input i_clk,
input i_rst_n,

input signed [31:0] i_u,
input signed [31:0] i_y,
input signed [31:0] i_DC,
input i_begin, //goes HI based on sampling rate

output reg signed [31:0] o_state0, //VCpv
output reg signed [31:0] o_state1, //IL1
output reg signed [31:0] o_state2, //IL2
output reg signed [31:0] o_state3, //VCout
output reg signed [31:0] o_IPV,
output reg signed [31:0] o_IPV_plus,
output reg signed [31:0] o_IPV_minus,
output reg o_DV
	);

reg signed [31:0] r_A [0:3][0:3]; //A matrix
reg signed [31:0] r_AT[0:3][0:3]; //AT matrix
reg signed [31:0] r_B [0:3]; //B matrix
reg signed [31:0] r_X [0:3]; //X - state variables
reg signed [31:0] r_P [0:3][0:3]; //P - Estimate Covariance Matrix 
reg signed [31:0] r_Q [0:3][0:3]; // Q - Process Noise Matrix
reg signed [31:0] r_C [0:3]; //C matrix
reg signed [31:0] r_C_hold [0:3]; //C hold matrix (for multiplication)
reg signed [31:0] y_update; //measurement residual
reg signed [31:0] r_S; //innovation covariance
reg signed [31:0] r_R; //observation noise covariance
reg signed [31:0] r_K[0:3]; //kalman gain
reg signed [31:0] r_X_DC_pred[0:3]; //xhat values for future predictions

//this is 32' fixed point number 1.0 shifted up 16 bits for division numerator purposes (taking inverse of S)
reg signed [47:0] r_one = 48'b00000000_00000001_00000000_00000000_00000000_00000000; 

//this is 32' fixed point number 1.0 used for identity matrix subtraction (no shifting here)
reg signed [31:0] r_one_ID = 32'b00000000_00000001_00000000_00000000;

//this is 32' fixed point number .0001 used for incrementing up or down the duty cycle for predictions
reg signed [31:0] r_delta_DC = 32'b00000000_00000000_00000000_00000111;

//this is 32' fixed point number -1.0 for multiplying numbers by -1
reg signed[31:0] r_neg_one = 32'b11111111_11111111_00000000_00000000;

reg signed [31:0] r_DC_plus;
reg signed [31:0] r_DC_minus;
reg signed [31:0] r_DC_use;  //this gets assigned DC, DC_plus, or DC_minus for A matrix computation
integer i = 0;
integer j = 0;
reg [7:0] r_SM_hold; //used for the A matrix updater to know what state to go back to
reg r_init_system = 0; //initialzed to 0, gets set to and stays at 1 after all data/memory is initialized
reg [7:0] r_SM = 0;

wire signed [63:0] mult_out[0:3][0:3];
reg  signed [31:0] mult_a[0:3][0:3];
reg  signed [31:0] mult_b[0:3];
wire signed [50:0] add_out[0:3];

//general purpose multiply accumulate blocks
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

reg [3:0] init_SM = 0;
always@(posedge i_clk)
begin
if(~r_init_system) begin //initialize my arrays with state space values and initial conditions (A Matrix Assumes DC of .5)
case(init_SM)
0: begin
r_A [0][0] <= 32'b0000000000000000_1111000100111011; r_A [0][1] <= 32'b1111111111111111_1111000100111011; r_A [0][2] <= 32'b1111111111111111_1111100010011110; r_A [0][3] <= 0;
r_A [1][0] <= 32'b0000000000000000_0000000011110110; r_A [1][1] <= 32'b0000000000000000_1111111100001010; r_A [1][2] <= 32'b1111111111111111_1111111110101110; r_A [1][3] <= 32'b1111111111111111_1111111110101110;
r_A [2][0] <= 32'b0000000000000000_0000000011110110; r_A [2][1] <= 32'b1111111111111111_1111111110101110; r_A [2][2] <= 32'b0000000000000000_1111111100001010; r_A [2][3] <= 32'b1111111111111111_1111111110101110;
r_A [3][0] <= 0; 									 r_A [3][1] <= 0;									  r_A [3][2] <= 32'b0000000000000000_0000011101100010; r_A [3][3] <= 32'b0000000000000000_1111111111011010;

r_B [0] <= 32'b0000000000000000_0000111011000101;    r_B [1] <= 0;     r_B [2] <= 0;   r_B [3] <= 0;

r_C [0] <= 0;    r_C [1] <= 0;     r_C [2] <= 0;   r_C [3] <= 32'b0000000000000001_0000000000000000;

r_P [0][0] <= 32'b0000000000000000_0001100110011010; r_P [0][1] <= 32'b0000000000000000_0001100110011010; r_P [0][2] <= 32'b0000000000000000_0001100110011010; r_P [0][3] <= 32'b0000000000000000_0001100110011010;
r_P [1][0] <= 32'b0000000000000000_0001100110011010; r_P [1][1] <= 32'b0000000000000000_0001100110011010; r_P [1][2] <= 32'b0000000000000000_0001100110011010; r_P [1][3] <= 32'b0000000000000000_0001100110011010; 
r_P [2][0] <= 32'b0000000000000000_0001100110011010; r_P [2][1] <= 32'b0000000000000000_0001100110011010; r_P [2][2] <= 32'b0000000000000000_0001100110011010; r_P [2][3] <= 32'b0000000000000000_0001100110011010; 
r_P [3][0] <= 32'b0000000000000000_0001100110011010; r_P [3][1] <= 32'b0000000000000000_0001100110011010; r_P [3][2] <= 32'b0000000000000000_0001100110011010; r_P [3][3] <= 32'b0000000000000000_0001100110011010;

r_Q [0][0] <= 32'b0000000000000000_0000000000000111; r_Q [0][1] <= 32'b0000000000000000_0000000000000111; r_Q [0][2] <= 32'b0000000000000000_0000000000000111; r_Q [0][3] <= 32'b0000000000000000_0000000000000111;
r_Q [1][0] <= 32'b0000000000000000_0000000000000111; r_Q [1][1] <= 32'b0000000000000000_0000000000000111; r_Q [1][2] <= 32'b0000000000000000_0000000000000111; r_Q [1][3] <= 32'b0000000000000000_0000000000000111;
r_Q [2][0] <= 32'b0000000000000000_0000000000000111; r_Q [2][1] <= 32'b0000000000000000_0000000000000111; r_Q [2][2] <= 32'b0000000000000000_0000000000000111; r_Q [2][3] <= 32'b0000000000000000_0000000000000111;
r_Q [3][0] <= 32'b0000000000000000_0000000000000111; r_Q [3][1] <= 32'b0000000000000000_0000000000000111; r_Q [3][2] <= 32'b0000000000000000_0000000000000111; r_Q [3][3] <= 32'b0000000000000000_0000000000000111;

r_S <= 0; 

r_R <= 32'b0000000000000000_0000001010001111; 

r_X[0] <= 32'b0000000000000000_0001100110011010; r_X[1] <= 32'b0000000000000000_0001100110011010; r_X[2] <= 32'b0000000000000000_0001100110011010; r_X[3] <= 32'b0000000000000000_0001100110011010;

init_SM <= 1;
end
1: begin //update A_Transpose from A matrix
for (i = 0; i < 4; i = i + 1)begin
for (j = 0; j <4; j = j + 1)begin
r_AT[i][j] <= r_A[j][i];	
end
end
r_init_system <= 1;
end
endcase 
end
else begin //begining of kalman filter SM
case(r_SM)
0: begin
o_DV <= 0;
if(i_begin)begin r_SM <= 100; r_DC_use <= i_DC; r_SM_hold <= 1; end
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
r_X[i] <= r_X[i] + (mult_out[0][i] >> 16) ;
end
r_SM <= 4;
end
4:begin //state estimation/prediction -- P_pred = A*P*AT + Q (this is the A*P part over 4 clock cycles - column 0)
for (i = 0; i < 4; i = i + 1)begin
for (j = 0; j < 4; j = j + 1) begin
mult_a[i][j] <= r_A[i][j];
mult_b[i] <= r_P[i][0];
end
end
r_SM <= 5;
end
5: begin //state estimation/prediction -- P_pred = A*P*AT + Q (this is the A*P part over 4 clock cycles - column 1)
for (i = 0; i < 4; i = i + 1)begin	
mult_b[i] <= r_P[i][1];
r_P[i][0] <= add_out[i];
end
r_SM <= 6;
end
6: begin // ... 
for (i = 0; i < 4; i = i + 1)begin	
mult_b[i] <= r_P[i][2];
r_P[i][1] <= add_out[i];
end
r_SM <= 7;
end
7:begin // ... 
for (i = 0; i < 4; i = i + 1)begin	
mult_b[i] <= r_P[i][3];
r_P[i][2] <= add_out[i];
end
r_SM <= 8;
end
8: begin // ...
for (i = 0; i < 4; i = i + 1)
r_P[i][3] <= add_out[i];	
r_SM <= 9;
end
9: begin ///state estimation/prediction -- P_pred = A*P*AT + Q (this is the (AP)*AT + Q part over 4 clock cycles - column 1)
for (i = 0; i < 4; i = i + 1)begin
for (j = 0; j < 4; j = j + 1) begin
mult_a[i][j] <= r_P[i][j];
mult_b[i] <= r_AT[i][0];
end
end
r_SM <= 10;
end
10: begin // ... 
for (i = 0; i < 4; i = i + 1)begin	
mult_b[i] <= r_AT[i][1];
r_P[i][0] <= add_out[i] + r_Q[i][0];
end
r_SM <= 11;
end
11: begin // ...
for (i = 0; i < 4; i = i + 1)begin
mult_b[i] <= r_AT[i][2];
r_P[i][1] <= add_out[i] + r_Q[i][1];
end
r_SM <= 12;
end
12: begin // ... 
for (i = 0; i < 4; i = i + 1)begin
mult_b[i] <= r_AT[i][3];
r_P[i][2] <= add_out[i] + r_Q[i][2];
end
r_SM <= 13;
end
13: begin // ...
for (i = 0; i < 4; i = i + 1)
r_P[i][3] <= add_out[i] + r_Q[i][3];	
r_SM <= 14;
end
14: begin //update stage -- y_update = y - C*x_pred (this is the CX part)
for(i=0; i < 4; i = i + 1)begin
mult_a[0][i] <= r_C[i];
mult_b[i] <= r_X[i];
end
r_SM <= 15;
end
15: begin //update stage -- y_update = y - C*x_pred (this is the subtraction part) -- S_update = C*P*CT + R (this is the C*P part)
y_update <= i_y - add_out[0];
for(i=0; i < 4; i = i+1)begin
for(j=0; j < 4; j = j+1)begin
mult_a[i][j] <= r_P[i][j];
mult_b[i] <= r_C[i];
end
end
r_SM <= 16;
end
16: begin // -- S_update = C*P*CT + R (this is the (CP)*CT + R part)
for(i=0; i < 4; i = i + 1)begin
mult_a[0][i] <= add_out[i];
mult_b[i] <= r_C[i];
end
r_SM <= 17;
end
17: begin // ...
r_S <= add_out[0] + r_R;
r_SM <= 18;
end
18:begin //take inverse of S by taking 1/S (numerator is already shifted up 16 bits) //K = P*CT*Sinv (this is P*CT)
r_S <= r_one / r_S; 	//This single division causes Fmax to drop from ~47MHZ to ~7.5 MHz, with +1K LE; look into replacing with something else
for(i= 0; i < 4; i = i + 1)begin
for(j = 0; j < 4; j = j +1)begin
mult_a[i][j] <= r_P[i][j];
mult_b[i] <= r_C[i];
end
end
r_SM <= 19;
end
19:begin //... (this is (PCT)*Sinv part)
for(i = 0; i < 4; i = i + 1)begin
mult_a[0][i] <= add_out[i];
mult_b[i] <= r_S;
end
r_SM <= 20;
end
20: begin // ...
for(i = 0; i < 4; i = i +1)
r_K[i] <= (mult_out[0][i] >> 16) ;
r_SM <= 21;
end
21: begin //X = X + K*Y (this is K*Y)
for (i = 0; i < 4; i = i+1)begin
mult_a[0][i] <= r_K[i];
mult_b[i] <= y_update;
end
r_SM <= 22;
end
22: begin //X = X + K*Y      -- P = (I - K*C)*P (this is K*C Part)
for(i =0; i < 4; i = i+1)begin
r_X[i] <= r_X[i] + (mult_out[0][i] >> 16) ;

mult_a[0][i] <= r_K[0];
mult_a[1][i] <= r_K[1];
mult_a[2][i] <= r_K[2];
mult_a[3][i] <= r_K[3];
mult_b[i] <= r_C[i];
end
r_SM <= 23;
end
23: begin // P = (I - K*C)*P (this is the (I - (KC)*P part)
for (i = 0; i < 4; i = i+1)begin
for (j = 0; j < 4; j = j+1)begin
if(i == j)
mult_a[i][j] <= r_one_ID - add_out[i][j];
else mult_a[i][j] <= add_out[i][j];
mult_b[i] <= r_P[i][0];
end
end
r_SM <= 24;
end
24: begin // ...
for(i = 0; i < 4; i = i + 1)begin
mult_b[i] <= r_P[i][1];
r_P[i][0] <= add_out[0];
end
r_SM <= 25;
end
25: begin // ...
for(i = 0; i < 4; i = i + 1)begin
mult_b[i] <= r_P[i][2];
r_P[i][1] <= add_out[0];
end
r_SM <= 26;
end
26: begin // ...
for(i = 0; i < 4; i = i + 1)begin
mult_b[i] <= r_P[i][3];
r_P[i][2] <= add_out[0];
end
r_SM <= 27;
end
27: begin // ...
for(i = 0; i < 4; i = i + 1)begin
r_P[i][3] <= add_out[0];
end
r_SM <= 28;
end
28: //route new state estimations to ouput, start prediction process 
begin
o_state0 <= r_X[0];
o_state1 <= r_X[1];
o_state2 <= r_X[2];
o_state3 <= r_X[3];
o_IPV <= i_u - r_X[0];
r_DC_plus  <= r_DC_use + r_delta_DC;
r_DC_minus <= r_DC_use - r_delta_DC;
r_SM <= 29;
end
29: begin //update necessary A matrix values with the r_DC_plus values
r_DC_use <= r_DC_plus;
r_SM <= 100;
r_SM_hold <= 30;
end
30: begin //estimate xhat again:
for(i = 0; i < 4; i = i + 1) begin
for(j = 0; j < 4; j = j + 1) begin
mult_a[i][j] <= r_A[i][j];
mult_b[i] <= r_X[i];
end
end
r_SM <= 31;
end
31: begin //State estimation/prediction -- X = AX + BU (this is the AX and BU part)
for (i=0; i < 4; i = i + 1) begin
r_X_DC_pred[i] <= add_out[i];
end
for(i = 0; i < 4; i = i + 1) begin
mult_a[0][i] <= r_B[i];
mult_b[i] <= i_u;
end
r_SM <= 32;
end
32:begin//State estimation/prediction -- X = AX + BU (this is the AX + BU part)
for(i = 0; i < 4; i = i + 1)begin
r_X_DC_pred[i] <= r_X_DC_pred[i] + (mult_out[0][i] >> 16) ;
end
r_SM <= 33;
end
33:begin //find IPV+ and start process again for IPV-
o_IPV_plus <= i_u - r_X_DC_pred[0]; 
r_DC_use <= r_DC_minus;
r_SM <= 100;
r_SM_hold <= 34;
end
34: begin //estimate xhat again:
for(i = 0; i < 4; i = i + 1) begin
for(j = 0; j < 4; j = j + 1) begin
mult_a[i][j] <= r_A[i][j];
mult_b[i] <= r_X[i];
end
end
r_SM <= 35;
end
35: begin //State estimation/prediction -- X = AX + BU (this is the AX and BU part)
for (i=0; i < 4; i = i + 1) begin
r_X_DC_pred[i] <= add_out[i];
end
for(i = 0; i < 4; i = i + 1) begin
mult_a[0][i] <= r_B[i];
mult_b[i] <= i_u;
end
r_SM <= 36;
end
36:begin//State estimation/prediction -- X = AX + BU (this is the AX + BU part)
for(i = 0; i < 4; i = i + 1)begin
r_X_DC_pred[i] <= r_X_DC_pred[i] + (mult_out[0][i] >> 16) ;
end
r_SM <= 37;
end
37:begin //find IPV- and end the SM
o_IPV_minus <= i_u - r_X_DC_pred[0]; 
r_SM <= 0;
o_DV <= 1;
end

//this is for updating the A Matrix with the existing value of the Duty Cycle DC (A(0,2), A(1,0), A(1,1), A(1,2), A(1,3), A(2,0), A(2,1), A(2,2), A(2,3), A(3, 2) (used at begining of SM and when making future predictions)
100:begin //make sure DC is between 0 and 1
if(r_DC_use < 0) r_DC_use <= 0;
else if(r_DC_use > 32'b00000000_00000001_00000000_00000000) r_DC_use <= 32'b00000000_00000001_00000000_00000000;
else r_DC_use <= r_DC_use;
r_SM <= 101;
end                 
101:begin           
mult_a[0][0] <= 32'b11110000_11111001_11011000_10011110; mult_b[0] <= r_DC_use;//A(0,2) (DC*-1/CPV) 
mult_a[0][1] <= 32'b00000000_10100110_10101010_10101011; mult_b[1] <= r_DC_use;//A(1,0) DC*L2/(L1*(L1+L2))) NEEDS TO BE ADDED TO 1/(L1+L2) THEN
mult_a[0][2] <= 32'b11111111_01011001_01010101_01010101; mult_b[2] <= r_DC_use;//A(1,1) DC*-RL1*L2/(L1*(L1+L2)) NEEDS TO BE ADDED TO -RL1/(L1+L2)
mult_a[0][3] <= 32'b00000000_10100110_10101010_10101011; mult_b[3] <= r_DC_use;//A(1,2) RL2*DC/(L1+L2) NEEDS TO BE ADDED TO -RL2/(L1+L2) THEN
mult_a[1][0] <= 32'b00000000_10100110_10101010_10101011; 					   //A(1,3) DC/(L1+L2) NEEDS ADDED TO -1/(L1+L2)
mult_a[1][1] <= 32'b00000000_10100110_10101010_10101011; 					   //A(2,0) L1*DC/(L2*(L1+L2) NEEDS ADDED TO 1/(L1+L2)
mult_a[1][2] <= 32'b00000000_10100110_10101010_10101011;                       //A(2,1) RL1*DC/(L1+L2) NEEDS ADDED TO -RL1/(L1+L2)
mult_a[1][3] <= 32'b11111111_01011001_01010101_01010101;                       //A(2,2) -RL2*L1*DC/(L2*(L1+L2) NEEDS ADDED TO -RL2/(L1+L2)
mult_a[2][0] <= 32'b00000000_10100110_10101010_10101011;                       //A(2,3) DC/(L1+L2) NEEDS ADDED TO -1/(L1+L2)
mult_a[2][1] <= 32'b11110000_11111001_11011000_10011110;                       //A(3,2) -DC/(Cout) NEEDS ADDED TO 1/(Cout)
r_SM <= 102;
end
102:begin //get results, add to other values, apply to proper A matrix position
r_A[0][2] <= (mult_out[0][0] >> 16);
r_A[1][0] <= (mult_out[0][1] >> 16) + 32'b00000000_10100110_10101010_10101011;
r_A[1][1] <= (mult_out[0][2] >> 16) + 32'b11111111_01011001_01010101_01010101;
r_A[1][2] <= (mult_out[0][3] >> 16) + 32'b11111111_01011001_01010101_01010101;
r_A[1][3] <= (mult_out[1][0] >> 16) + 32'b11111111_01011001_01010101_01010101;
r_A[2][0] <= (mult_out[1][1] >> 16) + 32'b00000000_10100110_10101010_10101011;
r_A[2][1] <= (mult_out[1][2] >> 16) + 32'b11111111_01011001_01010101_01010101;
r_A[2][2] <= (mult_out[1][3] >> 16) + 32'b11111111_01011001_01010101_01010101;
r_A[2][3] <= (mult_out[2][0] >> 16) + 32'b11111111_01011001_01010101_01010101;
r_A[3][2] <= (mult_out[2][1] >> 16) + 32'b00001111_00000110_00100111_01100010;
r_SM <= 103;
end
103: begin //discretize the updated values (add I + r_A*TS)
mult_a[0][0] <= r_A[0][2]; mult_b[0] <= 32'b00000000_00000000_00000000_00000001;
mult_a[0][1] <= r_A[1][0]; mult_b[1] <= 32'b00000000_00000000_00000000_00000001;
mult_a[0][2] <= r_A[1][1]; mult_b[2] <= 32'b00000000_00000000_00000000_00000001;
mult_a[0][3] <= r_A[1][2]; mult_b[3] <= 32'b00000000_00000000_00000000_00000001;
mult_a[1][0] <= r_A[1][3];
mult_a[1][1] <= r_A[2][0];
mult_a[1][2] <= r_A[2][1];
mult_a[1][3] <= r_A[2][2];
mult_a[2][0] <= r_A[2][3];
mult_a[2][1] <= r_A[3][2];
r_SM <= 104;
end
104:begin //add I and go back to the core SM
r_A[0][2] <= (mult_out[0][0] >> 16);
r_A[1][0] <= (mult_out[0][1] >> 16);
r_A[1][1] <= (mult_out[0][2] >> 16) + r_one_ID;
r_A[1][2] <= (mult_out[0][3] >> 16);
r_A[1][3] <= (mult_out[1][0] >> 16);
r_A[2][0] <= (mult_out[1][1] >> 16);
r_A[2][1] <= (mult_out[1][2] >> 16);
r_A[2][2] <= (mult_out[1][3] >> 16) + r_one_ID;
r_A[2][3] <= (mult_out[2][0] >> 16);
r_A[3][2] <= (mult_out[2][1] >> 16);
r_SM <= r_SM_hold;
end
endcase
end
end

endmodule

