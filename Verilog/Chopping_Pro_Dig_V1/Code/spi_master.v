// Updated SPI Master Testbench
// Compatible with SPI_slave module (40 outputs, 10 inputs)
`timescale 1ns/1ps

module SPI_master ();

// Clock and control signals
reg sclk;
reg reset_n;
reg mosi;
wire miso;

// Input signals to SPI slave (IN01-IN10)
reg [7:0] IN10, IN09, IN08, IN07, IN06, IN05, IN04, IN03, IN02, IN01;

// Output signals from SPI slave (OUT01-OUT40)
wire [7:0] OUT40, OUT39, OUT38, OUT37, OUT36, OUT35, OUT34, OUT33, OUT32, OUT31;
wire [7:0] OUT30, OUT29, OUT28, OUT27, OUT26, OUT25, OUT24, OUT23, OUT22, OUT21;
wire [7:0] OUT20, OUT19, OUT18, OUT17, OUT16, OUT15, OUT14, OUT13, OUT12, OUT11;
wire [7:0] OUT10, OUT09, OUT08, OUT07, OUT06, OUT05, OUT04, OUT03, OUT02, OUT01;

// SPI slave instance
SPI_slave SPI_slave_inst (
    // Inputs
    .SCLK(sclk),
    .RST_N(reset_n),
    .MOSI(mosi),
    .IN10(IN10),
    .IN09(IN09),
    .IN08(IN08),
    .IN07(IN07),
    .IN06(IN06),
    .IN05(IN05),
    .IN04(IN04),
    .IN03(IN03),
    .IN02(IN02),
    .IN01(IN01),
    // Outputs
    .MISO(miso),
    .OUT40(OUT40),
    .OUT39(OUT39),
    .OUT38(OUT38),
    .OUT37(OUT37),
    .OUT36(OUT36),
    .OUT35(OUT35),
    .OUT34(OUT34),
    .OUT33(OUT33),
    .OUT32(OUT32),
    .OUT31(OUT31),
    .OUT30(OUT30),
    .OUT29(OUT29),
    .OUT28(OUT28),
    .OUT27(OUT27),
    .OUT26(OUT26),
    .OUT25(OUT25),
    .OUT24(OUT24),
    .OUT23(OUT23),
    .OUT22(OUT22),
    .OUT21(OUT21),
    .OUT20(OUT20),
    .OUT19(OUT19),
    .OUT18(OUT18),
    .OUT17(OUT17),
    .OUT16(OUT16),
    .OUT15(OUT15),
    .OUT14(OUT14),
    .OUT13(OUT13),
    .OUT12(OUT12),
    .OUT11(OUT11),
    .OUT10(OUT10),
    .OUT09(OUT09),
    .OUT08(OUT08),
    .OUT07(OUT07),
    .OUT06(OUT06),
    .OUT05(OUT05),
    .OUT04(OUT04),
    .OUT03(OUT03),
    .OUT02(OUT02),
    .OUT01(OUT01)
);

// SPI parameters
reg [7:0] spi_unit_data;
parameter SPI_CMD_WRITE = 1'b0;
parameter SPI_CMD_READ  = 1'b1;
parameter SPI_PERIOD    = 5_000; // 5us period = 200kHz

// Initialize input signals
initial begin
    IN10 = 8'hAA;
    IN09 = 8'h99;
    IN08 = 8'h88;
    IN07 = 8'h77;
    IN06 = 8'h66;
    IN05 = 8'h55;
    IN04 = 8'h44;
    IN03 = 8'h33;
    IN02 = 8'h22;
    IN01 = 8'h11;
end

// Test sequence
initial begin
    // Initialize signals
    sclk = 1'b0;
    mosi = 1'b0;
    reset_n = 1'b1;
    
    // Display test start
    $display("===========================================");
    $display("SPI Master Testbench Started");
    $display("===========================================");
    
    // Reset sequence
    #6_000 reset_n = 1'b0;
    $display("Time %0t: Reset asserted", $time);
    #6_000 reset_n = 1'b1;
    $display("Time %0t: Reset released", $time);
    #10_000;
    
    // Test write operations to various addresses
    $display("\n--- Testing WRITE operations ---");
    spi_unit_write(7'd01, 8'hA5);
    $display("Time %0t: Written 0xA5 to address 01, OUT01 = 0x%h", $time, OUT01);
    
    spi_unit_write(7'd10, 8'h3C);
    $display("Time %0t: Written 0x3C to address 10, OUT10 = 0x%h", $time, OUT10);
    
    spi_unit_write(7'd20, 8'hF0);
    $display("Time %0t: Written 0xF0 to address 20, OUT20 = 0x%h", $time, OUT20);
    
    spi_unit_write(7'd40, 8'h5A);
    $display("Time %0t: Written 0x5A to address 40, OUT40 = 0x%h", $time, OUT40);
    
    // Test read operations from written addresses
    $display("\n--- Testing READ operations (reading back written data) ---");
    spi_unit_read(7'd01);
    $display("Time %0t: Read from address 01, data = 0x%h (expected 0xA5)", $time, spi_unit_data);
    
    spi_unit_read(7'd10);
    $display("Time %0t: Read from address 10, data = 0x%h (expected 0x3C)", $time, spi_unit_data);
    
    spi_unit_read(7'd20);
    $display("Time %0t: Read from address 20, data = 0x%h (expected 0xF0)", $time, spi_unit_data);
    
    spi_unit_read(7'd40);
    $display("Time %0t: Read from address 40, data = 0x%h (expected 0x5A)", $time, spi_unit_data);
    
    // Test read operations from input registers (addresses 41-50)
    $display("\n--- Testing READ operations (reading input registers) ---");
    spi_unit_read(7'd41);
    $display("Time %0t: Read from address 41 (IN01), data = 0x%h (expected 0x11)", $time, spi_unit_data);
    
    spi_unit_read(7'd45);
    $display("Time %0t: Read from address 45 (IN05), data = 0x%h (expected 0x55)", $time, spi_unit_data);
    
    spi_unit_read(7'd50);
    $display("Time %0t: Read from address 50 (IN10), data = 0x%h (expected 0xAA)", $time, spi_unit_data);
    
    // Additional write-read cycle
    $display("\n--- Testing additional WRITE-READ cycle ---");
    spi_unit_write(7'd15, 8'hCC);
    $display("Time %0t: Written 0xCC to address 15", $time);
    spi_unit_read(7'd15);
    $display("Time %0t: Read from address 15, data = 0x%h (expected 0xCC)", $time, spi_unit_data);
    
    #50_000_000;
    $display("\n===========================================");
    $display("SPI Master Testbench Completed");
    $display("===========================================");
    $finish;
end

// SPI write task - write 1 byte to SPI slave at specified address
task spi_unit_write;
    input [6:0] addr;
    input [7:0] data;
    integer i;
    reg [7:0] cmd;
    begin
        cmd = {SPI_CMD_WRITE, addr};
        
        // Send command byte (write bit + address)
        for (i=7; i>=0; i=i-1) begin
            mosi = cmd[i];
            #(SPI_PERIOD/2) sclk = 1'b1;
            #(SPI_PERIOD/2) sclk = 1'b0;
        end
        
        // Small delay between command and data
        #(SPI_PERIOD*2);
        
        // Send data byte
        for (i=7; i>=0; i=i-1) begin
            mosi = data[i];
            #(SPI_PERIOD/2) sclk = 1'b1;
            #(SPI_PERIOD/2) sclk = 1'b0;
        end
        
        // Inter-transaction delay
        #(SPI_PERIOD*2);
    end
endtask

// SPI read task - read 1 byte from SPI slave at specified address
task spi_unit_read;
    input [6:0] addr;
    integer i;
    reg [7:0] cmd;
    begin
        spi_unit_data = 8'd0;
        cmd = {SPI_CMD_READ, addr};
        
        // Send command byte (read bit + address)
        for (i=7; i>=0; i=i-1) begin
            mosi = cmd[i];
            #(SPI_PERIOD/2) sclk = 1'b1;
            #(SPI_PERIOD/2) sclk = 1'b0;
        end
        
        // Small delay between command and data
        #(SPI_PERIOD*2);
        
        // Receive data byte
        for (i=7; i>=0; i=i-1) begin
            #(SPI_PERIOD/2) sclk = 1'b1;
            #(SPI_PERIOD/2) sclk = 1'b0;
            spi_unit_data[i] = miso;
        end
        
        // Inter-transaction delay
        #(SPI_PERIOD*2);
    end
endtask

// Optional: Generate VCD file for waveform viewing
/*
initial begin
    $dumpfile("spi_tb.vcd");
    $dumpvars(0, SPI_master_tb);
end
*/

endmodule
