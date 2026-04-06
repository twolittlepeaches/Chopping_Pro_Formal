
module SPI_slave (
	//inputs
	SCLK,			// SPI clock
	RST_N,		// asynchronous negative reset
	MOSI,			// SPI master-output slave-input
	
	IN10,
	IN09,
	IN08,
	IN07,
	IN06,
	IN05,
	IN04,
	IN03,
	IN02,
	IN01,
	
	//outputs
	MISO,			// SPI master-input slave-output
	OUT40,
	OUT39,
	OUT38,
	OUT37,
	OUT36,
	OUT35,
	OUT34,
	OUT33,
	OUT32,
	OUT31,
	OUT30,
	OUT29,
	OUT28,
	OUT27,
	OUT26,
	OUT25,	
	OUT24,
	OUT23,
	OUT22,
	OUT21,
	OUT20,
	OUT19,
	OUT18,
	OUT17,
	OUT16,
	OUT15,
	OUT14,
	OUT13,
	OUT12,
	OUT11,
	OUT10,
	OUT09,
	OUT08,
	OUT07,
	OUT06,
	OUT05,
	OUT04,
	OUT03,
	OUT02,
	OUT01
	);

// Operation state definitions.
localparam  [1:0]   STATE_IDLE  = 2'b10;    // idle
localparam  [1:0]   STATE_CMD   = 2'b00;    // command input
localparam  [1:0]   STATE_DATA  = 2'b01;    // data access
parameter	    SPI_CMD_WRITE=1'b0;
parameter	    SPI_CMD_READ= 1'b1;

//inputs
input  		wire  SCLK;						// SPI clock
input  		wire  RST_N;   // async. reset negative
input  		wire  MOSI;      // SPI master-output slave-input
input  		[7:0] IN10;
input  		[7:0] IN09;
input  		[7:0] IN08;
input  		[7:0] IN07;
input  		[7:0] IN06;
input  		[7:0] IN05;
input  		[7:0] IN04;
input   	[7:0] IN03;
input  		[7:0] IN02;
input   	[7:0] IN01;

//outputs
output	 wire MISO;       // SPI master-input slave-output
output  [7:0] OUT40;
output  [7:0] OUT39;
output  [7:0] OUT38;
output  [7:0] OUT37;
output  [7:0] OUT36;
output  [7:0] OUT35;
output  [7:0] OUT34;
output  [7:0] OUT33;
output  [7:0] OUT32;
output  [7:0] OUT31;
output  [7:0] OUT30;
output  [7:0] OUT29;
output  [7:0] OUT28;
output  [7:0] OUT27;
output  [7:0] OUT26;
output  [7:0] OUT25;
output  [7:0] OUT24;
output  [7:0] OUT23;
output  [7:0] OUT22;
output  [7:0] OUT21;
output  [7:0] OUT20;
output  [7:0] OUT19;
output  [7:0] OUT18;
output  [7:0] OUT17;
output  [7:0] OUT16;
output  [7:0] OUT15;
output  [7:0] OUT14;
output  [7:0] OUT13;
output  [7:0] OUT12;
output  [7:0] OUT11;
output  [7:0] OUT10;
output  [7:0] OUT09;
output  [7:0] OUT08;
output  [7:0] OUT07;
output  [7:0] OUT06;
output  [7:0] OUT05;
output  [7:0] OUT04;
output  [7:0] OUT03;
output  [7:0] OUT02;
output  [7:0] OUT01;

//----------------------------------------------------------------------------
// Internal signal declarations
//----------------------------------------------------------------------------
reg   [7:0]   spcr;              // SPI command register, including r/w and address
reg   [7:0]   spdw;              // data write shift register
reg   [7:0]   spdr;              // data read shift register
reg   [3:0]   counter;           // clock positive edge counter
wire  [1:0]   state;             // SPI operating state    
wire 	      wr_en;          			// write enable
wire  	      rd_en;         				// read enable
reg   [7:0]   out40_reg;
reg   [7:0]   out39_reg;
reg   [7:0]   out38_reg;
reg   [7:0]   out37_reg;
reg   [7:0]   out36_reg;
reg   [7:0]   out35_reg;
reg   [7:0]   out34_reg;
reg   [7:0]   out33_reg;
reg   [7:0]   out32_reg;
reg   [7:0]   out31_reg;
reg   [7:0]   out30_reg;
reg   [7:0]   out29_reg;
reg   [7:0]   out28_reg;
reg   [7:0]   out27_reg;
reg   [7:0]   out26_reg;
reg   [7:0]   out25_reg;
reg   [7:0]   out24_reg;
reg   [7:0]   out23_reg;
reg   [7:0]   out22_reg;
reg   [7:0]   out21_reg;
reg   [7:0]   out20_reg;
reg   [7:0]   out19_reg;
reg   [7:0]   out18_reg;
reg   [7:0]   out17_reg;
reg   [7:0]   out16_reg;
reg   [7:0]   out15_reg;
reg   [7:0]   out14_reg;
reg   [7:0]   out13_reg;
reg   [7:0]   out12_reg;
reg   [7:0]   out11_reg;
reg   [7:0]   out10_reg;
reg   [7:0]   out09_reg;
reg   [7:0]   out08_reg;
reg   [7:0]   out07_reg;
reg   [7:0]   out06_reg;
reg   [7:0]   out05_reg;
reg   [7:0]   out04_reg;
reg   [7:0]   out03_reg;
reg   [7:0]   out02_reg;
reg   [7:0]   out01_reg;


//----------------------------------------------------------------------------
// Main codes
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
// Control signals generation
//----------------------------------------------------------------------------
// Clock positive edge counter.
always @ (posedge SCLK or negedge RST_N)
begin
    if (!RST_N)
        counter <= 4'h0;
    else 
        counter <= counter + 4'h1;
end
//comments: In the above block, counter 4 bits; from 0~7 commands(addr+w/r), from 8~15 data.

// SPI command shift in.
always @ (posedge SCLK or negedge RST_N)
begin
    if (!RST_N)
        spcr <= 8'h00;
    else if (state == STATE_CMD)//SCLK rising edge 0~7
        spcr <= {spcr[6:0], MOSI};//left shift, MSB first
end

// SPI operating state and write/read enable signals.
assign state = {1'b0, counter[3]};
assign wr_en = (state == STATE_DATA) && (spcr[7] == SPI_CMD_WRITE);
assign rd_en = (state == STATE_DATA) && (spcr[7] == SPI_CMD_READ );


//----------------------------------------------------------------------------
// SPI data write process
//----------------------------------------------------------------------------
// SPI write data shift in.
always @ (posedge SCLK or  negedge RST_N)
begin
    if (!RST_N)
        spdw <= 8'h00;
    else if (wr_en)
        spdw <= {spdw[6:0], MOSI};
end

//send the data in when counter=15 based on the addr.
always @ (posedge SCLK or negedge RST_N)
begin
	if (!RST_N) begin
                out40_reg<=8'h00;
                out39_reg<=8'h00;
                out38_reg<=8'h00;
                out37_reg<=8'h00;
                out36_reg<=8'h00;
                out35_reg<=8'h00;
                out34_reg<=8'h00;
                out33_reg<=8'h00;
		out32_reg<=8'h00;
		out31_reg<=8'h00;
		out30_reg<=8'h00;
		out29_reg<=8'h00;
		out28_reg<=8'h00;
		out27_reg<=8'h00;
		out26_reg<=8'h00;
		out25_reg<=8'h00;
		out24_reg<=8'h00;
		out23_reg<=8'h00;
		out22_reg<=8'h00;
		out21_reg<=8'h00;
		out20_reg<=8'h00;
		out19_reg<=8'h00;
		out18_reg<=8'h00;
		out17_reg<=8'h00;
		out16_reg<=8'h00;
		out15_reg<=8'h00;
		out14_reg<=8'h00;
		out13_reg<=8'h00;
		out12_reg<=8'h00;
		out11_reg<=8'h00;
		out10_reg<=8'h00;
		out09_reg<=8'h00;
		out08_reg<=8'h00;
		out07_reg<=8'h00;
		out06_reg<=8'h00;
		out05_reg<=8'h00;
		out04_reg<=8'h00;
		out03_reg<=8'h00;
		out02_reg<=8'h00;
		out01_reg<=8'b1011_0101;//reserved for test
	end
	else if (wr_en && counter == 4'hF)
		case (spcr[6:0])
			7'd40	 :	out40_reg <= {spdw[6:0], MOSI};
			7'd39	 :	out39_reg <= {spdw[6:0], MOSI};
			7'd38	 :	out38_reg <= {spdw[6:0], MOSI};
			7'd37	 :	out37_reg <= {spdw[6:0], MOSI};
			7'd36	 :	out36_reg <= {spdw[6:0], MOSI};
			7'd35	 :	out35_reg <= {spdw[6:0], MOSI};
			7'd34	 :	out34_reg <= {spdw[6:0], MOSI};
			7'd33	 :	out33_reg <= {spdw[6:0], MOSI};
			7'd32	 :	out32_reg <= {spdw[6:0], MOSI};
			7'd31	 :      out31_reg <= {spdw[6:0], MOSI};
			7'd30	 :	out30_reg <= {spdw[6:0], MOSI};
			7'd29	 :      out29_reg <= {spdw[6:0], MOSI};
			7'd28	 : 	out28_reg <= {spdw[6:0], MOSI};
			7'd27	 : 	out27_reg <= {spdw[6:0], MOSI};
			7'd26	 : 	out26_reg <= {spdw[6:0], MOSI};
			7'd25	 : 	out25_reg <= {spdw[6:0], MOSI};
			7'd24	 : 	out24_reg <= {spdw[6:0], MOSI};
			7'd23	 : 	out23_reg <= {spdw[6:0], MOSI};
			7'd22	 : 	out22_reg <= {spdw[6:0], MOSI};
			7'd21	 : 	out21_reg <= {spdw[6:0], MOSI};
			7'd20	 : 	out20_reg <= {spdw[6:0], MOSI};
			7'd19	 : 	out19_reg <= {spdw[6:0], MOSI};
			7'd18	 : 	out18_reg <= {spdw[6:0], MOSI};
			7'd17	 : 	out17_reg <= {spdw[6:0], MOSI};
			7'd16	 : 	out16_reg <= {spdw[6:0], MOSI};
			7'd15	 : 	out15_reg <= {spdw[6:0], MOSI};
			7'd14	 : 	out14_reg <= {spdw[6:0], MOSI};
			7'd13	 : 	out13_reg <= {spdw[6:0], MOSI};
			7'd12	 : 	out12_reg <= {spdw[6:0], MOSI};
			7'd11	 : 	out11_reg <= {spdw[6:0], MOSI};
			7'd10	 : 	out10_reg <= {spdw[6:0], MOSI};
			7'd09	 : 	out09_reg <= {spdw[6:0], MOSI};
			7'd08	 : 	out08_reg <= {spdw[6:0], MOSI};
			7'd07	 : 	out07_reg <= {spdw[6:0], MOSI};
			7'd06	 : 	out06_reg <= {spdw[6:0], MOSI};
			7'd05	 : 	out05_reg <= {spdw[6:0], MOSI};
			7'd04	 : 	out04_reg <= {spdw[6:0], MOSI};
			7'd03	 : 	out03_reg <= {spdw[6:0], MOSI};
			7'd02	 : 	out02_reg <= {spdw[6:0], MOSI};
			7'd01	 : 	out01_reg <= {spdw[6:0], MOSI};			
			default: 	out40_reg <= 8'h00;
	endcase
end

//read the data out at falling edge of SCLK, when counter=8 based on addr.
always @ (negedge SCLK or negedge RST_N)
begin
	if (!RST_N)
        spdr <= 8'h00;
  else if (rd_en && (counter == 4'h8))
		case (spcr[6:0])
			7'd50		: 	spdr <= IN10;
			7'd49		: 	spdr <= IN09;
			7'd48		: 	spdr <= IN08;
			7'd47		: 	spdr <= IN07;
			7'd46		: 	spdr <= IN06;
			7'd45		: 	spdr <= IN05;
			7'd44		: 	spdr <= IN04;
			7'd43		: 	spdr <= IN03;
			7'd42		: 	spdr <= IN02;
			7'd41		: 	spdr <= IN01;		
            7'd40    :      spdr <= out40_reg;
            7'd39    :      spdr <= out39_reg;
            7'd38    :      spdr <= out38_reg;
            7'd37    :      spdr <= out37_reg;
            7'd36    :      spdr <= out36_reg;
            7'd35    :      spdr <= out35_reg;
            7'd34    :      spdr <= out34_reg;
            7'd33    :      spdr <= out33_reg;
			7'd32	 : 	spdr <= out32_reg;
			7'd31	 : 	spdr <= out31_reg;
			7'd30	 : 	spdr <= out30_reg;
			7'd29	 : 	spdr <= out29_reg;
			7'd28	 : 	spdr <= out28_reg;
			7'd27	 : 	spdr <= out27_reg;
			7'd26	 : 	spdr <= out26_reg;
			7'd25	 : 	spdr <= out25_reg;
			7'd24	 :  spdr <= out24_reg;
			7'd23	 : 	spdr <= out23_reg;
			7'd22	 : 	spdr <= out22_reg;
			7'd21	 : 	spdr <= out21_reg;
			7'd20	 : 	spdr <= out20_reg;
			7'd19	 : 	spdr <= out19_reg;
			7'd18	 : 	spdr <= out18_reg;
			7'd17	 : 	spdr <= out17_reg;
			7'd16	 : 	spdr <= out16_reg;
			7'd15	 : 	spdr <= out15_reg;
			7'd14	 : 	spdr <= out14_reg;
			7'd13	 : 	spdr <= out13_reg;
			7'd12	 : 	spdr <= out12_reg;
			7'd11	 : 	spdr <= out11_reg;
			7'd10	 : 	spdr <= out10_reg;
			7'd09	 : 	spdr <= out09_reg;
			7'd08	 : 	spdr <= out08_reg;
			7'd07	 : 	spdr <= out07_reg;
			7'd06	 : 	spdr <= out06_reg;
			7'd05	 : 	spdr <= out05_reg;
			7'd04	 : 	spdr <= out04_reg;
			7'd03	 : 	spdr <= out03_reg;
			7'd02	 : 	spdr <= out02_reg;
			7'd01	 : 	spdr <= out01_reg;
			default: 	spdr <= 8'h00;
		endcase
	else
		spdr <= spdr << 1;//when counter=9~15, spdr left shift one by one.
end
		
// SPI MISO output.
assign MISO  = spdr[7];

//assign outputs
assign OUT40 = out40_reg;
assign OUT39 = out39_reg;
assign OUT38 = out38_reg;
assign OUT37 = out37_reg;
assign OUT36 = out36_reg;
assign OUT35 = out35_reg;
assign OUT34 = out34_reg;
assign OUT33 = out33_reg;
assign OUT32 = out32_reg;
assign OUT31 = out31_reg;
assign OUT30 = out30_reg;
assign OUT29 = out29_reg;
assign OUT28 = out28_reg;
assign OUT27 = out27_reg;
assign OUT26 = out26_reg;
assign OUT25 = out25_reg;
assign OUT24 = out24_reg;
assign OUT23 = out23_reg;
assign OUT22 = out22_reg;
assign OUT21 = out21_reg;
assign OUT20 = out20_reg;
assign OUT19 = out19_reg;
assign OUT18 = out18_reg;
assign OUT17 = out17_reg;
assign OUT16 = out16_reg;
assign OUT15 = out15_reg;
assign OUT14 = out14_reg;
assign OUT13 = out13_reg;
assign OUT12 = out12_reg;
assign OUT11 = out11_reg;
assign OUT10 = out10_reg;
assign OUT09 = out09_reg;
assign OUT08 = out08_reg;
assign OUT07 = out07_reg;
assign OUT06 = out06_reg;
assign OUT05 = out05_reg;
assign OUT04 = out04_reg;
assign OUT03 = out03_reg;
assign OUT02 = out02_reg;
assign OUT01 = out01_reg;

endmodule