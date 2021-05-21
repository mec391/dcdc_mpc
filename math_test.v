`timescale 1ns/1ns
module math_test_TB();
reg i_clk;
reg signed [31:0] i_data0;
reg signed [31:0] i_data1;
wire [31:0] o_answer_div0;
wire [31:0] o_answer_mul0;
wire [31:0] o_answer_div1;
wire [31:0] o_answer_mul1;

math_test UUT(
.i_clk (i_clk),
.i_data0(i_data0),
.i_data1(i_data1),
.o_answer_div0(o_answer_div0),
.o_answer_mul0(o_answer_mul0),
.o_answer_div1(o_answer_div1),
.o_answer_mul1(o_answer_mul1)
	);

initial begin
i_clk = 1;
i_data0 = 0;
i_data1 = 0;

# 2
i_data0 = 32'b0000_0000_0000_0001_1000_0000_0000_0000; //1.5
i_data1 = 32'b0000_0000_0000_0011_1000_0000_0000_0000; //3.5
/*
1.5 * 3.5 = 5.25     1.5 / 3.5 = .42857
3.5 * 1.5 = 5.25     3.5 / 1.5 = 2.3333
*/
#10 
i_data1 = 32'b0000_0000_0000_0001_1000_0000_0000_0000; //1.5
i_data0 = 32'b0000_0000_0000_0011_1000_0000_0000_0000; //3.5

end

always begin
#1 i_clk = !i_clk;
end

endmodule

module math_test(
input i_clk,
input signed [31:0] i_data0,
input signed [31:0] i_data1,
output reg signed[31:0] o_answer_div0,
output reg signed[31:0] o_answer_mul0,
output reg signed[31:0] o_answer_div1,
output reg signed[31:0] o_answer_mul1
);


reg [47:0] hold0;
reg [47:0] hold1;
reg [3:0] sm = 0;
always@(posedge i_clk)
begin
o_answer_div0 <= (i_data0 << 16) / i_data1;
o_answer_mul0 <= (i_data0 * i_data1) >> 16;

case(sm)
0: begin hold0 <= i_data0; hold1 <= i_data1; sm <= 1; end
1: begin o_answer_div1 <= (hold0 << 16) / hold1; o_answer_mul1 <= (hold0 * hold1) >> 16; sm <= 0; end
endcase


end

endmodule 
