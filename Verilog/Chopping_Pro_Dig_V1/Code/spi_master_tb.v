`timescale 1ps/1fs

module spi_master_tb(
    output reg         SCLK,
    output reg         RST_N,
    output reg         MOSI,
    input  wire        MISO
);

    //========================================================================
    // SPI Configuration Parameters
    //========================================================================
    parameter SPI_PERIOD = 10_000;  // 100ns = 10MHz SPI clock
    
    
    // Frequency control
    reg [4:0]  FCW_I_tmp;
    reg [11:0] FCW_F_tmp;
    reg [7:0]  TDC_OFT_tmp;
    reg [11:0] INVKDTC_MANUAL_tmp;
    
    // DCO & tracking
    reg [7:0]  INVKDCO_tmp;
    reg [3:0]  ALPHA_tmp;
    reg [4:0]  RHO_tmp;
    reg        TUNE_I_EN_tmp;
    
    // Control enables
    reg        TDC_OUT2SPI_FREEZE_tmp;
    reg        DTCQ_SD_EN_tmp;
    reg [12:0] OTW_MANUAL_tmp;
    reg        OTW_MANUAL_EN_tmp;
    reg        BB_EN_tmp;
    
    // DCO/DTC modes
    reg        DCO_SD_EN_tmp;
    reg        MAIN_DTC_TEST_EN_tmp;
    reg        TDV2_EN_tmp;
    reg        CHOPPER_EN_tmp;
    reg        DCO_DEM_EN_tmp;
    
    // Calibration
    reg        CAL_INVKDTC_EN_tmp;
    reg        CAL_TDV2_EN_tmp;
    reg        CAL_OFTDTC_EN_tmp;
    reg        TDC_TEST_MODE_tmp;
    
    // DTC test
    reg [7:0]  DTC_TEST_P1_tmp;
    reg [7:0]  DTC_TEST_P2_tmp;
    reg [9:0]  DTC_TEST_DCW1_tmp;
    reg [9:0]  DTC_TEST_DCW2_tmp;
    reg        DTC_TEST_EDGE_SEL_tmp;
    
    // Analog block controls
    reg [9:0]  OFFSET_DTC_CTRL_manual_tmp;
    reg [9:0]  DTC_CTRL_manual_tmp;
    reg        RST_N_DIGMAIN_tmp;
    
    // Freeze controls
    reg        INVKDTC_OUT2SPI_FREEZE_tmp;
    reg        Tvd2_mis_OUT2SPI_FREEZE_tmp;
    reg        OFTDTC_OUT2SPI_FREEZE_tmp;
    
    // Analog control signals
    reg        DCO_BUFFER_ENi_tmp;
    reg        DCO_TEST_BUFFER_ENi_tmp;
    reg [5:0]  DCO_coarse_ENi_tmp;
    reg        TDC_EN_tmp;
    reg [4:0]  DCO_Tail_ENi_tmp;
    reg        MAIN_DTC_pre_clk_sel_tmp;
    reg        MAIN_DTC_pre_clk_test_tmp;
    reg [4:0]  MAIN_DTC_DL_cap_ctrl_tmp;
    reg        EN_MMD_MUX_tmp;
    reg        OFFSET_DTC_pre_clk_sel_tmp;
    reg        OFFSET_DTC_pre_clk_test_tmp;
    reg [4:0]  OFFSET_DTC_DL_cap_ctrl_tmp;
    
    //========================================================================
    // SPI Output Data Wires (packed format)
    //========================================================================
    wire [7:0] spi_out01, spi_out02, spi_out03, spi_out04;
    wire [7:0] spi_out05, spi_out06, spi_out07, spi_out08;
    wire [7:0] spi_out09, spi_out10, spi_out11, spi_out12;
    wire [7:0] spi_out13, spi_out14, spi_out15, spi_out16;
    wire [7:0] spi_out17, spi_out18, spi_out19, spi_out20;
    wire [7:0] spi_out21, spi_out22, spi_out23, spi_out24;
    wire [7:0] spi_out25, spi_out26, spi_out27, spi_out28;
    wire [7:0] spi_out29, spi_out30, spi_out31, spi_out32;
    wire [7:0] spi_out33, spi_out34, spi_out35;
    
    // Pack tmp registers into SPI output format
    assign spi_out01 = {3'b000, FCW_I_tmp};
    assign spi_out02 = FCW_F_tmp[7:0];
    assign spi_out03 = {4'b0000, FCW_F_tmp[11:8]};
    assign spi_out04 = TDC_OFT_tmp;
    assign spi_out05 = INVKDTC_MANUAL_tmp[7:0];
    assign spi_out06 = {4'b0000, INVKDTC_MANUAL_tmp[11:8]};
    assign spi_out07 = INVKDCO_tmp;
    assign spi_out08 = {4'b0000, ALPHA_tmp};
    assign spi_out09 = {3'b000, RHO_tmp};
    assign spi_out10 = {6'b000000, TUNE_I_EN_tmp, TDC_OUT2SPI_FREEZE_tmp};
    assign spi_out11 = {7'b0000000, DTCQ_SD_EN_tmp};
    assign spi_out12 = OTW_MANUAL_tmp[7:0];
    assign spi_out13 = {3'b000, OTW_MANUAL_tmp[12:8]};
    assign spi_out14 = {6'b000000, OTW_MANUAL_EN_tmp, BB_EN_tmp};
    assign spi_out15 = {6'b000000, DCO_SD_EN_tmp, MAIN_DTC_TEST_EN_tmp};
    assign spi_out16 = {5'b00000, TDV2_EN_tmp, CHOPPER_EN_tmp, DCO_DEM_EN_tmp};
    assign spi_out17 = {4'b0000, CAL_INVKDTC_EN_tmp, CAL_TDV2_EN_tmp, CAL_OFTDTC_EN_tmp, TDC_TEST_MODE_tmp};
    assign spi_out18 = DTC_TEST_P1_tmp;
    assign spi_out19 = DTC_TEST_P2_tmp;
    assign spi_out20 = DTC_TEST_DCW1_tmp[7:0];
    assign spi_out21 = {6'b000000, DTC_TEST_DCW1_tmp[9:8]};
    assign spi_out22 = DTC_TEST_DCW2_tmp[7:0];
    assign spi_out23 = {6'b000000, DTC_TEST_DCW2_tmp[9:8]};
    assign spi_out24 = {6'b000000, DTC_TEST_EDGE_SEL_tmp, TDC_TEST_MODE_tmp};
    assign spi_out25 = {5'b00000, INVKDTC_OUT2SPI_FREEZE_tmp, Tvd2_mis_OUT2SPI_FREEZE_tmp, OFTDTC_OUT2SPI_FREEZE_tmp};
    assign spi_out26 = OFFSET_DTC_CTRL_manual_tmp[7:0];
    assign spi_out27 = {6'b000000, OFFSET_DTC_CTRL_manual_tmp[9:8]};
    assign spi_out28 = DTC_CTRL_manual_tmp[7:0];
    assign spi_out29 = {RST_N_DIGMAIN_tmp, 5'b00000, DTC_CTRL_manual_tmp[9:8]};
    assign spi_out30 = {DCO_BUFFER_ENi_tmp, DCO_TEST_BUFFER_ENi_tmp, DCO_coarse_ENi_tmp};
    assign spi_out31 = {2'b00, TDC_EN_tmp, DCO_Tail_ENi_tmp};
    assign spi_out32 = {MAIN_DTC_TEST_EN_tmp, MAIN_DTC_pre_clk_sel_tmp, MAIN_DTC_pre_clk_test_tmp, MAIN_DTC_DL_cap_ctrl_tmp};
    assign spi_out33 = {EN_MMD_MUX_tmp, OFFSET_DTC_pre_clk_sel_tmp, OFFSET_DTC_pre_clk_test_tmp, OFFSET_DTC_DL_cap_ctrl_tmp};
    assign spi_out34 = 8'h00;
    assign spi_out35 = 8'h00;
    
    
    //========================================================================
    // SPI Tasks (copied from original testbench)
    //========================================================================
    
    task spi_unit_write;
        input [6:0] addr;
        input [7:0] data;
        integer i;
        reg [7:0] cmd;
        begin
            cmd = {1'b0, addr};  // Write command
            
            // Send command byte
            for (i=7; i>=0; i=i-1) begin
                MOSI = cmd[i];
                #(SPI_PERIOD/2) SCLK = 1'b1;
                #(SPI_PERIOD/2) SCLK = 1'b0;
            end
            
            #(SPI_PERIOD*2);
            
            // Send data byte
            for (i=7; i>=0; i=i-1) begin
                MOSI = data[i];
                #(SPI_PERIOD/2) SCLK = 1'b1;
                #(SPI_PERIOD/2) SCLK = 1'b0;
            end
            
            #(SPI_PERIOD*2);
        end
    endtask
    
    task spi_unit_read;
        input [6:0] addr;
        output [7:0] data;
        integer i;
        reg [7:0] cmd;
        begin
            data = 8'd0;
            cmd = {1'b1, addr};  // Read command
            
            // Send command byte
            for (i=7; i>=0; i=i-1) begin
                MOSI = cmd[i];
                #(SPI_PERIOD/2) SCLK = 1'b1;
                #(SPI_PERIOD/2) SCLK = 1'b0;
            end
            
            #(SPI_PERIOD*2);
            
            // Receive data byte
            for (i=7; i>=0; i=i-1) begin
                #(SPI_PERIOD/2) SCLK = 1'b1;
                #(SPI_PERIOD/2) SCLK = 1'b0;
                data[i] = MISO;
            end
            
            #(SPI_PERIOD*2);
        end
    endtask
    
    //========================================================================
    // Testbench Initialization & Test Sequence
    //========================================================================
    
    initial begin
        // Initialize
        SCLK = 1'b0;
        MOSI = 1'b0;
        RST_N = 1'b0;


  
        // ====================================================================
        // Initialize Configuration Parameters
        // ====================================================================
        FCW_I_tmp = 5'd10;
        FCW_F_tmp = 12'h840;
        TDC_OFT_tmp = 8'h80;
        INVKDTC_MANUAL_tmp = 12'd1781;
        INVKDCO_tmp = 8'd120;
        ALPHA_tmp = 4'd3;
        RHO_tmp = 5'd8;
        
        TUNE_I_EN_tmp = 1'b1;
        DTCQ_SD_EN_tmp = 1'b0;
        OTW_MANUAL_tmp = 13'd0;
        OTW_MANUAL_EN_tmp = 1'b0;
        BB_EN_tmp = 1'b1;
        
        MAIN_DTC_TEST_EN_tmp = 1'b0;
        DCO_DEM_EN_tmp = 1'b0;
        TDC_TEST_MODE_tmp = 1'b0;
        
        DTC_TEST_P1_tmp = 8'd20;
        DTC_TEST_P2_tmp = 8'd20;
        DTC_TEST_DCW1_tmp = 10'd2;
        DTC_TEST_DCW2_tmp = 10'd3;
        DTC_TEST_EDGE_SEL_tmp = 1'b0;
        
        INVKDTC_OUT2SPI_FREEZE_tmp = 1'b1;
        Tvd2_mis_OUT2SPI_FREEZE_tmp = 1'b1;
        OFTDTC_OUT2SPI_FREEZE_tmp = 1'b1;
        
        OFFSET_DTC_CTRL_manual_tmp = 10'd100;
        DTC_CTRL_manual_tmp = 10'd50;
        RST_N_DIGMAIN_tmp = 1'b1;
        
        DCO_BUFFER_ENi_tmp = 1'b0;
        DCO_TEST_BUFFER_ENi_tmp = 1'b0;
        DCO_coarse_ENi_tmp = 6'd20;
        TDC_EN_tmp = 1'b1;
        DCO_Tail_ENi_tmp = 5'd0;
        MAIN_DTC_pre_clk_sel_tmp = 1'b0;
        MAIN_DTC_pre_clk_test_tmp = 1'b1;
        MAIN_DTC_DL_cap_ctrl_tmp = 5'd0;
        EN_MMD_MUX_tmp = 1'b1;
        OFFSET_DTC_pre_clk_sel_tmp = 1'b0;
        OFFSET_DTC_pre_clk_test_tmp = 1'b0;
        OFFSET_DTC_DL_cap_ctrl_tmp = 5'd0;
        
        DCO_SD_EN_tmp = 1'b0;
        CHOPPER_EN_tmp = 1'b0;
        CAL_INVKDTC_EN_tmp = 1'b0;
        TDV2_EN_tmp = 1'b0;
        CAL_TDV2_EN_tmp = 1'b0;
        CAL_OFTDTC_EN_tmp = 1'b0;
        TDC_OUT2SPI_FREEZE_tmp = 1'b1;
        
        // ====================================================================
        // Reset Sequence
        // ====================================================================
        #1000;
        RST_N = 1'b1;
        #1000;
        
        // ====================================================================
        // SPI Configuration Phase (Write all 35 registers)
        // ====================================================================
        
        spi_unit_write(7'd01, spi_out01);
        spi_unit_write(7'd02, spi_out02);
        spi_unit_write(7'd03, spi_out03);
        spi_unit_write(7'd04, spi_out04);
        spi_unit_write(7'd05, spi_out05);
        spi_unit_write(7'd06, spi_out06);
        spi_unit_write(7'd07, spi_out07);
        spi_unit_write(7'd08, spi_out08);
        spi_unit_write(7'd09, spi_out09);
        spi_unit_write(7'd10, spi_out10);
        spi_unit_write(7'd11, spi_out11);
        spi_unit_write(7'd12, spi_out12);
        spi_unit_write(7'd13, spi_out13);
        spi_unit_write(7'd14, spi_out14);
        spi_unit_write(7'd15, spi_out15);
        spi_unit_write(7'd16, spi_out16);
        spi_unit_write(7'd17, spi_out17);
        spi_unit_write(7'd18, spi_out18);
        spi_unit_write(7'd19, spi_out19);
        spi_unit_write(7'd20, spi_out20);
        spi_unit_write(7'd21, spi_out21);
        spi_unit_write(7'd22, spi_out22);
        spi_unit_write(7'd23, spi_out23);
        spi_unit_write(7'd24, spi_out24);
        spi_unit_write(7'd25, spi_out25);
        spi_unit_write(7'd26, spi_out26);
        spi_unit_write(7'd27, spi_out27);
        spi_unit_write(7'd28, spi_out28);
        spi_unit_write(7'd29, spi_out29);
        spi_unit_write(7'd30, spi_out30);
        spi_unit_write(7'd31, spi_out31);
        spi_unit_write(7'd32, spi_out32);
        spi_unit_write(7'd33, spi_out33);
        spi_unit_write(7'd34, spi_out34);
        spi_unit_write(7'd35, spi_out35);
        
        #130_0;  // 130us
    
        TDV2_EN_tmp = 1'b0;//swap
        CHOPPER_EN_tmp = 1'b0;
        CAL_INVKDTC_EN_tmp = 1'b0;
        CAL_TDV2_EN_tmp = 1'b0;//(if cal tdv2 and oftdtc,must en chopper,else not lock)&&chopper can not be en alone!!!
        CAL_OFTDTC_EN_tmp = 1'b0;
    
        #1
        spi_unit_write(7'd16, spi_out16);
        #1
        spi_unit_write(7'd17, spi_out17);

    end

endmodule
