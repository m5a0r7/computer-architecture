
`timescale 1ns/1ns

module fp_adder
  (
  input [31 : 0] a,
  input [31 : 0] b,
  output [31 : 0] s
  );
  

wire [7 : 0] E_A ;
wire [7 : 0] E_B ;
 
wire S_A ;
wire S_B ;

wire [22 : 0] F_A ;
wire [22 : 0] F_B ;

wire [7: 0 ] a_ex ;
wire [7 : 0] b_ex ;

wire [25 : 0] small_frac ;
wire [25 : 0] big_frac ;

wire [25 : 0] A_frac ;
wire [25 : 0] B_frac ;


wire [28 : 0] asb ;
wire small_sign ;
wire big_sign ;

wire [7 : 0] ex_diff ;


wire borrow ;
wire sticky ;

wire [25 : 0] X;

wire [7 :0] E ;
wire [7 :0] E1 ;
wire [28 : 0] meymoon ;

wire [28 : 0] olagh ;


wire [ 28 : 0] tc_small ;
wire [28 : 0] tc_big ;

wire [26 : 0] Y , Z ;
wire [27 : 0] Y1 ,Z1, W , T;
wire [28 : 0] f_fract;


wire sub;

wire [28 : 0] gav ;

wire sign_final ;

wire [2 : 0] rgs ;

wire [23 : 0] amir1 ;
wire [22 : 0] amir ;
wire [23 : 0] amin ;



wire [27 : 0] FINAL ;

wire [22 : 0] final_fraction ;
wire [7 : 0] final_ex ;



assign S_A = a[31];
assign S_B = b[31];

assign E_A = a[30 : 23];
assign E_B = b[30 : 23];

assign F_A = a[22 : 0];
assign F_B = b[22 : 0];



wire R ;

wire G ;


assign a_ex =  (E_A == 0) ?  8'b00000001 : E_A ;
assign b_ex =  (E_B == 0) ?  8'b00000001 : E_B ;

assign A_frac = (E_A == 0) ?  {1'b0 , F_A , 2'b00} : {1'b1 , F_A , 2'b00};
assign B_frac = (E_B == 0) ?  {1'b0 , F_B , 2'b00} : {1'b1 , F_B , 2'b00};


assign borrow = (a_ex < b_ex) ? 1'b1 : 1'b0 ;

assign ex_diff = (borrow == 0) ? (a_ex - b_ex) : (b_ex - a_ex) ;



assign small_frac = (borrow == 0) ? B_frac : A_frac ;
assign big_frac = (borrow == 0) ? A_frac : B_frac ;


assign small_sign = (borrow == 0) ? S_B : S_A ;
assign big_sign = (borrow == 0) ? S_A : S_B ;

assign sticky = | (small_frac << (26 - ex_diff)  ) ;

assign X = small_frac >> ex_diff ;

assign E = (borrow == 0) ? a_ex : b_ex ;


assign Y = {X , sticky} ;
assign Z = {big_frac , 1'b0} ;
assign Y1 ={1'b0 , Y };
assign Z1 ={1'b0 , Z };

assign W = (small_sign == 0) ? Y1 : ~Y1 + 1 ;
assign T = (big_sign == 0) ? Z1 : ~Z1 + 1;
 
assign tc_small = {W[27] , W} ;
assign tc_big = {T[27] , T} ;

assign sub = S_A ^ S_B ;


assign gav = tc_small + tc_big ;

assign asb = (gav[28] == 0) ? gav  : ~ gav + 1;



assign sign_final = gav[28] ;




assign rgs = asb[2:0];


assign f_fract = asb ;


assign E1 = f_fract[28] ? E+2 
: f_fract[27] ? E+1 
: f_fract[26] ? E 
: f_fract[25] ? (E > 1) ? E-1 : 0 
: f_fract[24] ? (E > 2) ? E-2 : 0 
: f_fract[23] ? (E > 3) ? E-3 : 0 
: f_fract[22] ? (E > 4) ? E-4 : 0 
: f_fract[21] ? (E > 5) ? E-5 : 0 
: f_fract[20] ? (E > 6) ? E-6 : 0 
: f_fract[19] ? (E > 7) ? E-7 : 0 
: f_fract[18] ? (E > 8) ? E-8 : 0 
: f_fract[17] ? (E > 9) ? E-9 : 0 
: f_fract[16] ? (E > 10) ? E-10 : 0 
: f_fract[15] ? (E > 11) ? E-11 : 0 
: f_fract[14] ? (E > 12) ? E-12 : 0 
: f_fract[13] ? (E > 13) ? E-13 : 0 
: f_fract[12] ? (E > 14) ? E-14 : 0 
: f_fract[11] ? (E > 15) ? E-15 : 0 
: f_fract[10] ? (E > 16) ? E-16 : 0 
: f_fract[9] ? (E > 17) ? E-17 : 0 
: f_fract[8] ? (E > 18) ? E-18 : 0 
: f_fract[7] ? (E > 19) ? E-19 : 0 
: f_fract[6] ? (E > 20) ? E-20 : 0 
: f_fract[5] ? (E > 21) ? E-21 : 0 
: f_fract[4] ? (E > 22) ? E-22 : 0 
: f_fract[3] ? (E > 23) ? E-23 : 0 
: f_fract[2] ? (E > 24) ? E-24 : 0 
: f_fract[1] ? (E > 25) ? E-25 : 0
: f_fract[0] ? ((E > 26) ? E-26 : 0) : 0;





 
assign {amir1 , R , G} = (E1 != 0) ?( (3+E1)>=2 + E ? {f_fract[25 +(E1 - E)  -: 23 ] , f_fract[2 + E1 - E] , |(f_fract << (27 + E - E1))}
:((3+E1)==1+E) ? {f_fract[23 : 1], f_fract[0] , 1'b0}
: meymoon[28 : 4] )
:( (3-(- 1))>=2+E ? {f_fract[25 -( E  - 1 )  -: 23 ] , f_fract[2 - (E - 1)] , |(f_fract << (27 + E - 1 ))}
:((3 - (- 1 ) )==1+E) ? {f_fract[23: 1], f_fract[1] , 1'b0}
: meymoon[28 : 4] );
assign amir = amir1[22 : 0] ;

assign amin = ({ R , G } == 2'b11 )? amir + 1
: {R , G} == 2'b01 ? amir 
: {R , G} == 2'b00 ? amir : (amir + amir[0]) ;


assign meymoon = ( E1 != 0) ? ((3 + E1  < 1+E) ?  (f_fract << (3 + E - E1)) : {29{1'b0}})
:((2 < E) ? (f_fract << (2+E)) : {29{1'b0}} )  ;
assign olagh = f_fract << 1;


assign final_fraction = ((amin[23] == 1) ? {(23){1'b0}} : amin[22 : 0] );


assign final_ex = (amin[23] == 1) ? E1 + 1 : E1 ;


assign s = {sign_final , final_ex , final_fraction };





endmodule










