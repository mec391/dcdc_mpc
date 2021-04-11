module FIL_TEST
(
input i_clk,
input i_reset_n,

input [31:0] i_Vpv,
input [31:0] i_Ipv,
input [31:0] i_Vout,

output [31:0] o_Vout,

output reg o_MPC_switch

	);

reg [12:0] counter;
assign o_Vout = i_Vout;

always@(posedge i_clk)
begin
if(~i_reset_n) begin counter <= 0; o_MPC_switch <= 0; end
else begin
if(counter == 12'b1111_1111_1111) begin counter <= 0; o_MPC_switch <= 1; end
else begin counter <= counter + 1; o_MPC_switch <= 0; end
end
end


endmodule