module fenpin (fin,fout);

input  fin;
output fout;

reg    fout;
reg    fout1;
reg    fout2;

always@(posedge fin)
begin 

fout1<=~fout1;
fout2<=fout1;
fout<=fout2;

end


endmodule