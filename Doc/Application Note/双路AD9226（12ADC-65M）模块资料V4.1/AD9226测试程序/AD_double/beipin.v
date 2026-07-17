module beipin (fin,fout);

input  fin;
output fout;
wire   temp;
reg    q;
always@(posedge fout)
begin 
q<=temp;
end
assign temp=~q;
assign fout=~(temp^fin);

endmodule