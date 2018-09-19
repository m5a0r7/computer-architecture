`timescale 1ns/1ns
module multiplier2(
  input clk,
  input start,
  input [7:0] A,
  input [7:0] B,
  output reg [15:0] Product,
  output ready
  );
  
  
  reg [3:0] counter;
  reg [7:0] BB;
  
  
  
  wire [7:0] s;
  wire Cout;
  wire [7:0] mux;
  
  
  assign mux = Product[0] ? BB : 8'b0000_0000;
  
  assign {Cout,s} = {1'b0,Product[15:8]} + {1'b0,mux} ;
  
  assign ready = counter[3];
  
  
  always@(posedge clk)
    if(start) begin
      counter <= 4'h0;
      Product[7:0] <= A;
      BB <= B;
      Product [15:8] <= 8'h00;
    end
  
    else if (! ready) begin
      counter <= counter + 1;
      Product [6:0] <= Product [7:1];
      Product [15:7] <= {Cout,s};
    end
endmodule
  
  

  

