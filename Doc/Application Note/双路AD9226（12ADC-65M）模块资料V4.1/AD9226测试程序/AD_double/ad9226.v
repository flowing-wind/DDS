
module ad9226(date_in,date_out,clk);
input  [11:0]  date_in;
input   clk;
output [11:0] date_out;

reg   [11:0] date_out;


   always@( posedge clk )
    begin 

    date_out <= date_in;
    end


endmodule 