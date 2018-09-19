`timescale 1ns/1ns
module multiplier3
  #( parameter nb =8
  )
(
  input clk,
  input start,
  input [nb -1 :0] A,
  input [nb -1 :0] B,
  output reg signed [2 * nb - 1:0] Product,
  output ready
  );
  
  
  reg [nb - 1:0] counter;
  
  
  wire [nb:0] s;
  wire Cout;
  wire [nb:0] mux;
  
  reg [nb - 1 :0] BB;
  
  assign mux = Product[0] ? {BB[nb - 1],BB} : {(nb + 1){1'b0}};
  
  assign s = (counter != nb - 1) ? ({Product[2*nb - 1],Product[2*nb - 1:nb]} + mux) : ({Product[2 * nb - 1],Product[2 * nb - 1: nb]} + ~ mux + 1)  ;
  
  assign ready = (counter == nb);
  
  
  always@(posedge clk)
    if(start) begin
      counter <= {nb{1'b0}};
      BB <= B;
      Product[nb - 1:0] <= A;

      Product [2 * nb - 1:nb] <= {nb{1'b0}};
    end
  
    else if (! ready ) begin
      counter <= counter + 1;
      Product [nb - 2:0] <= Product [nb - 1:1];
      Product [2 * nb - 1: nb - 1] <= s ;
    end
endmodule
  
  

  





