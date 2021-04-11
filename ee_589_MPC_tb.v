`timescale 1ns/1ns
module ee_589_MPC_tb();
parameter word_size = 32;
reg i_clk;
reg i_reset_n;

reg [word_size-1:0] i_Vpv;
reg [word_size-1:0] i_Ipv;
reg [word_size-1:0] i_Vout;

reg i_calc_DV; //goes high based on the sampling rate

wire o_switch;

MPC UUT(
.i_clk (i_clk),
.i_reset_n (i_reset_n),
.i_Vpv (i_Vpv),
.i_Ipv (i_Ipv),
.i_Vout (i_Vout),
.i_calc_DV (i_calc_DV),
.o_switch (o_switch)
	);

//part 1
initial begin
i_clk = 1;
i_reset_n = 1;

#2 i_reset_n = 0; //toggle reset
#4 i_reset_n = 1;

i_Vpv[31:16] = 20; i_Vpv[15:0] = 0;
i_Ipv[31:16] = 3; i_Ipv[15:0] = 0;
i_Vout[31:16] = 100; i_Vout[15:0] = 0;
i_calc_DV = 1;
#2 i_calc_DV = 0;

#20 
i_Vpv[31:16] = 20; i_Vpv[15:0] = 0;
i_Ipv[31:16] = 4; i_Ipv[15:0] = 0;
i_Vout[31:16] = 100; i_Vout[15:0] = 0;
i_calc_DV = 1;
#2 i_calc_DV = 0;


#20 
i_Vpv[31:16] = 20; i_Vpv[15:0] = 0;
i_Ipv[31:16] = 2; i_Ipv[15:0] = 0;
i_Vout[31:16] = 100; i_Vout[15:0] = 0;
i_calc_DV = 1;
#2 i_calc_DV = 0;
end

always begin 
#1 i_clk = !i_clk;
end

endmodule