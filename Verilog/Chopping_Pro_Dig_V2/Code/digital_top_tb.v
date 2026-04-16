
`timescale 1ps/1fs

module digital_top_tb();

////define for digital top to analog blocks
//========================================================================
// SPI Interface signals
reg         SCLK;
reg         RST_N;
reg         MOSI;
wire        MISO;

// Clock signals input for digital top
wire        CKR;
wire        CLK_SD;

// TDC input
wire [7:0]  TDC_Q;
// Test inputs
reg  [7:0]  Reserved_IN1, Reserved_IN2;

// DCO tuning words
wire [62:0] OTW_T_H;
wire [126:0] OTW_T_L;
wire [7:0]  DCO_sdm;

// DTC control signals
wire [3:0]  DTC_P_top_4b;
wire [62:0] DTC_P_top_6u;
wire [14:0] DTC_P_bot_4u;
wire [3:0]  DTC_N_top_4b;
wire [62:0] DTC_N_top_6u;
wire [14:0] DTC_N_bot_4u;

// Other control signals
wire        Chopping_EN;
wire [4:0]  MMD_RN;

// Test outputs for analog blocks
wire [7:0]  Reserved_OUT1, Reserved_OUT2, Reserved_OUT3;
wire [7:0]  Reserved_OUT4, Reserved_OUT5, Reserved_OUT6;
//========================================================================



////define for analog to analog blocks    for tb
//========================================================================
// temp Internal signals for analog blocks
wire        ckvd;
wire        MAIN_DTC_clk_ref;
wire        MAIN_DTC_clk_delay;
wire        OFFSET_DTC_clk_ref;
wire        OFFSET_DTC_clk_delay;
wire        OFFSET_DTC_in;
wire        MAIN_DTC_clk_delayi;
wire        MAIN_DTC_DEMUX_TEST;
reg         MAIN_DTC_DIV_OUT;
wire        MAIN_DTC_TEST_OUTPUT;

wire        DTC_TEST_MODE;
wire        VD, VS;
wire        ckv;
//========================================================================



////define for reference clock from external generator
//========================================================================
wire        FREF;
//========================================================================


//// extract signals from digital top to ana SPI/dig input
//========================================================================
// Extract signals from Reserved_OUT1 (8-bit)
// Reserved_OUT1 = {DCO_BUFFER_ENi, DCO_TEST_BUFFER_ENi, DCO_coarse_ENi[5:0]}
wire        DCO_BUFFER_ENi;
wire        DCO_TEST_BUFFER_ENi;
wire [5:0]  DCO_coarse_ENi; // OTW_PVT

assign DCO_BUFFER_ENi       = Reserved_OUT1[7];
assign DCO_TEST_BUFFER_ENi  = Reserved_OUT1[6];
assign DCO_coarse_ENi       = Reserved_OUT1[5:0];

// Extract signals from Reserved_OUT2
// Reserved_OUT2 = {2'b00, TDC_EN, DCO_Tail_ENi[4:0]}
wire        TDC_EN;
wire [4:0]  DCO_Tail_ENi;

assign TDC_EN       = Reserved_OUT2[5];  // Always set to 1'b1
assign DCO_Tail_ENi = Reserved_OUT2[4:0];

// Extract signals from Reserved_OUT3
// Reserved_OUT3 = {MAIN_DTC_TEST_EN, MAIN_DTC_pre_clk_sel, MAIN_DTC_pre_clk_test, MAIN_DTC_DL_cap_ctrl[4:0]}
wire        MAIN_DTC_TEST_EN;
wire        MAIN_DTC_pre_clk_sel;
wire        MAIN_DTC_pre_clk_test;
wire [4:0]  MAIN_DTC_DL_cap_ctrl;

assign MAIN_DTC_TEST_EN      = Reserved_OUT3[7];  // Should match tmp_SPI
assign MAIN_DTC_pre_clk_sel  = Reserved_OUT3[6];  // VS
assign MAIN_DTC_pre_clk_test = Reserved_OUT3[5];  // VD
assign MAIN_DTC_DL_cap_ctrl  = Reserved_OUT3[4:0];  // 5'd0

// Extract signals from Reserved_OUT4
// Reserved_OUT4 = {EN_MMD_MUX, OFFSET_DTC_pre_clk_sel, OFFSET_DTC_pre_clk_test, OFFSET_DTC_DL_cap_ctrl[4:0]}
wire        EN_MMD_MUX;
wire        OFFSET_DTC_pre_clk_sel;
wire        OFFSET_DTC_pre_clk_test;
wire [4:0]  OFFSET_DTC_DL_cap_ctrl;

assign EN_MMD_MUX               = Reserved_OUT4[7];  // Always 1'b1
assign OFFSET_DTC_pre_clk_sel   = Reserved_OUT4[6];  // 1'b0
assign OFFSET_DTC_pre_clk_test  = Reserved_OUT4[5];  // 1'b0
assign OFFSET_DTC_DL_cap_ctrl   = Reserved_OUT4[4:0]; // 5'd0
//========================================================================



//// SPI Parameters
//========================================================================
parameter SPI_PERIOD = 100_000; // 100ns = 10MHz
//========================================================================

//// SPI configuration and test sequence
//====================================================================
// STEP 1. Declaration of intermediate variables (before initial block)
//====================================================================
// SPI output registers
wire [7:0] spi_out01, spi_out02, spi_out03, spi_out04;
wire [7:0] spi_out05, spi_out06, spi_out07, spi_out08;
wire [7:0] spi_out09, spi_out10, spi_out11, spi_out12;
wire [7:0] spi_out13, spi_out14, spi_out15, spi_out16;
wire [7:0] spi_out17, spi_out18, spi_out19, spi_out20;
wire [7:0] spi_out21, spi_out22, spi_out23, spi_out24;
wire [7:0] spi_out25, spi_out26, spi_out27, spi_out28;
wire [7:0] spi_out29, spi_out30, spi_out31, spi_out32;
wire [7:0] spi_out33, spi_out34, spi_out35;

// Frequency control parameters
reg [4:0]  FCW_I_tmp;              // -> OUT01
reg [11:0] FCW_F_tmp;              // -> OUT02,OUT03
reg [7:0]  TDC_OFT_tmp;           // -> OUT04
reg [11:0] INVKDTC_MANUAL_tmp;     // -> OUT05,OUT06
reg [7:0]  INVKDCO_tmp;           // -> OUT07
reg [3:0]  ALPHA_tmp;             // -> OUT08
reg [4:0]  RHO_tmp;               // -> OUT09

// Enable and control signals
reg        TUNE_I_EN_tmp;          // -> OUT10
reg        TDC_OUT2SPI_FREEZE_tmp; // -> OUT10
reg        DTCQ_SD_EN_tmp;         // -> OUT11
reg [12:0] OTW_MANUAL_tmp;         // -> OUT12,OUT13
reg        OTW_MANUAL_EN_tmp;      // -> OUT14
reg        BB_EN_tmp;              // -> OUT14

// DCO and DTC control
reg        DCO_SD_EN_tmp;          // -> OUT15
reg        MAIN_DTC_TEST_EN_tmp;   // -> OUT15
reg        CHOPPER_EN_tmp;         // -> OUT16
reg        DCO_DEM_EN_tmp;         // -> OUT16

// Calibration and test modes
reg        CAL_INVKDTC_EN_tmp;     // -> OUT17
reg        CAL_OFTDTC_EN_tmp;      // -> OUT17
reg        TDC_TEST_MODE_tmp;      // -> OUT17[0]
reg        MASH_SEL_tmp;           // -> OUT17[4]  0=MASH1, 1=MASH1-1
reg        DIFF_EN_tmp;            // -> OUT17[5]  0=SE,    1=DIFF
reg        CAL_POLY_EN_tmp;        // -> OUT17[6]  0=off,   1=on

// DTC test parameters
reg [7:0]  DTC_TEST_P1_tmp;       // -> OUT18
reg [7:0]  DTC_TEST_P2_tmp;       // -> OUT19
reg [9:0]  DTC_TEST_DCW1_tmp;     // -> OUT20,OUT21
reg [9:0]  DTC_TEST_DCW2_tmp;     // -> OUT22,OUT23
reg        DTC_TEST_EDGE_SEL_tmp; // -> OUT24
reg        DTC_TEST_MODE_tmp;     // -> OUT24

// DTC control and freeze signals
reg [9:0]  OFFSET_DTC_CTRL_manual_tmp; // -> OUT26,OUT27
reg [9:0]  DTC_CTRL_manual_tmp;        // -> OUT28,OUT29
reg        RST_N_DIGMAIN_tmp;          // -> OUT29
reg        INVKDTC_OUT2SPI_FREEZE_tmp; // -> OUT25
reg        OFTDTC_OUT2SPI_FREEZE_tmp;  // -> OUT25

//for extract signals from digital top to ana SPI/dig input
// -> OUT30
reg        DCO_BUFFER_ENi_tmp;
reg        DCO_TEST_BUFFER_ENi_tmp;
reg [5:0]  DCO_coarse_ENi_tmp;
// -> OUT31
reg        TDC_EN_tmp;
reg [4:0]  DCO_Tail_ENi_tmp;
// -> OUT32
reg        MAIN_DTC_pre_clk_sel_tmp;
reg        MAIN_DTC_pre_clk_test_tmp;
reg [4:0]  MAIN_DTC_DL_cap_ctrl_tmp;
// -> OUT33
reg        EN_MMD_MUX_tmp;
reg        OFFSET_DTC_pre_clk_sel_tmp;
reg        OFFSET_DTC_pre_clk_test_tmp;
reg [4:0]  OFFSET_DTC_DL_cap_ctrl_tmp;


//====================================================================
//====================================================================
// STEP 3. Prepare SPI output data from tmp variables
//====================================================================
// Prepare frequency control registers
assign spi_out01 = {3'b000, FCW_I_tmp};
assign spi_out02 = FCW_F_tmp[7:0];
assign spi_out03 = {4'b0000, FCW_F_tmp[11:8]};
assign spi_out04 = TDC_OFT_tmp;

// Prepare INVKDTC and other parameters
assign spi_out05 = INVKDTC_MANUAL_tmp[7:0];
assign spi_out06 = {4'b0000, INVKDTC_MANUAL_tmp[11:8]};
assign spi_out07 = INVKDCO_tmp;
assign spi_out08 = {4'b0000, ALPHA_tmp};
assign spi_out09 = {3'b000, RHO_tmp};

// Prepare control enables
assign spi_out10 = {6'b000000, TUNE_I_EN_tmp, TDC_OUT2SPI_FREEZE_tmp};
assign spi_out11 = {7'b0000000, DTCQ_SD_EN_tmp};

// Prepare OTW configuration
assign spi_out12 = OTW_MANUAL_tmp[7:0];
assign spi_out13 = {3'b000, OTW_MANUAL_tmp[12:8]};
assign spi_out14 = {6'b000000, OTW_MANUAL_EN_tmp, BB_EN_tmp};

// Prepare DCO and DTC controls
assign spi_out15 = {6'b000000, DCO_SD_EN_tmp, MAIN_DTC_TEST_EN_tmp};
assign spi_out16 = {6'b000000, CHOPPER_EN_tmp, DCO_DEM_EN_tmp};
// OUT17 bit 分配:
// [0] TDC_TEST_MODE  [1] CAL_OFTDTC_EN  [2] reserved(0)  [3] CAL_INVKDTC_EN
// [4] MASH_SEL       [5] DIFF_EN         [6] CAL_POLY_EN  [7] reserved
assign spi_out17 = {1'b0, CAL_POLY_EN_tmp, DIFF_EN_tmp, MASH_SEL_tmp,
                   CAL_INVKDTC_EN_tmp, 1'b0, CAL_OFTDTC_EN_tmp, TDC_TEST_MODE_tmp};

// Prepare DTC test parameters
assign spi_out18 = DTC_TEST_P1_tmp;
assign spi_out19 = DTC_TEST_P2_tmp;
assign spi_out20 = DTC_TEST_DCW1_tmp[7:0];
assign spi_out21 = {6'b000000, DTC_TEST_DCW1_tmp[9:8]};
assign spi_out22 = DTC_TEST_DCW2_tmp[7:0];
assign spi_out23 = {6'b000000, DTC_TEST_DCW2_tmp[9:8]};
assign spi_out24 = {6'b000000, DTC_TEST_EDGE_SEL_tmp, DTC_TEST_MODE_tmp};

// Prepare freeze signals
assign spi_out25 = {6'b000000, INVKDTC_OUT2SPI_FREEZE_tmp, OFTDTC_OUT2SPI_FREEZE_tmp};

// Prepare DTC controls
assign spi_out26 = OFFSET_DTC_CTRL_manual_tmp[7:0];
assign spi_out27 = {6'b000000, OFFSET_DTC_CTRL_manual_tmp[9:8]};
assign spi_out28 = DTC_CTRL_manual_tmp[7:0];
assign spi_out29 = {RST_N_DIGMAIN_tmp, 5'b00000, DTC_CTRL_manual_tmp[9:8]};


assign spi_out30 = {DCO_BUFFER_ENi_tmp, DCO_TEST_BUFFER_ENi_tmp, DCO_coarse_ENi_tmp};
assign spi_out31 = {2'b00, TDC_EN_tmp, DCO_Tail_ENi_tmp};
assign spi_out32 = {MAIN_DTC_TEST_EN_tmp, MAIN_DTC_pre_clk_sel_tmp, MAIN_DTC_pre_clk_test_tmp, MAIN_DTC_DL_cap_ctrl_tmp};
assign spi_out33 = {EN_MMD_MUX_tmp, OFFSET_DTC_pre_clk_sel_tmp, OFFSET_DTC_pre_clk_test_tmp, OFFSET_DTC_DL_cap_ctrl_tmp};
assign spi_out34 = 8'h00; // Unused
assign spi_out35 = 8'h00; // Unused

assign DTC_TEST_MODE = spi_out24[0];
//========================================================================
// Reference clock generator (100MHz)
//========================================================================
FREF_generator #(.Fr(100e6)) FREF_generator_inst(
    .fref(FREF),
    .rst_n()
);

//========================================================================
// Digital Top (DUT)
//========================================================================
digital_top digital_top_inst (
    // SPI Interface
    .SCLK           (SCLK),
    .RST_N          (RST_N),
    .MOSI           (MOSI),
    .MISO           (MISO),
    
    // Clock inputs
    .CKR            (CKR),
    .CLK_SD         (CLK_SD),
    
    // TDC input
    .TDC_Q          (TDC_Q),
    
    // Test inputs
    .Reserved_IN1    (Reserved_IN1),
    .Reserved_IN2    (Reserved_IN2),
    
    // DCO tuning words (no DCO_coarse_ENi port in digital_top)
    .OTW_T_H        (OTW_T_H),
    .OTW_T_L        (OTW_T_L),
    .DCO_sdm        (DCO_sdm),
    
    // DTC control outputs
    .DTC_P_top_4b     (DTC_P_top_4b),
    .DTC_P_top_6u     (DTC_P_top_6u),
    .DTC_P_bot_4u     (DTC_P_bot_4u),
    .DTC_N_top_4b  (DTC_N_top_4b),
    .DTC_N_top_6u  (DTC_N_top_6u),
    .DTC_N_bot_4u  (DTC_N_bot_4u),
    
    // Other control outputs
    .Chopping_EN        (Chopping_EN),
    .MMD_RN         (MMD_RN),
    
    // Test outputs for analog blocks
    .Reserved_OUT1  (Reserved_OUT1),
    .Reserved_OUT2  (Reserved_OUT2),
    .Reserved_OUT3  (Reserved_OUT3),
    .Reserved_OUT4  (Reserved_OUT4),
    .Reserved_OUT5  (Reserved_OUT5),
    .Reserved_OUT6  (Reserved_OUT6)
);

//========================================================================
// DCO instantiation
//========================================================================
DCO DCO_inst (
    .OTW_C      (DCO_coarse_ENi),
    .OTW_M1     ({1'b0, OTW_T_H[62:48]}),
    .OTW_M2     (OTW_T_H[47:32]),
    .OTW_M3     (OTW_T_H[31:16]),
    .OTW_M4     (OTW_T_H[15:0]),
    .OTW_F1     ({1'b0, OTW_T_L[126:96]}),
    .OTW_F2     (OTW_T_L[95:64]),
    .OTW_F3     (OTW_T_L[63:32]),
    .OTW_F4     (OTW_T_L[31:0]),
    .OTW_sd     (DCO_sdm),
    .ckv        (ckv),
    .VD         (VD),
    .VS         (VS)
);

//========================================================================
// module divider_8:CLK_SD generation (divide ckv by 8)
//========================================================================
reg [2:0] cnt_ckv;
initial begin
    cnt_ckv <= 3'd0;
end
always @(posedge ckv) begin
    cnt_ckv <= cnt_ckv + 1'b1;
end

assign CLK_SD = cnt_ckv[2];

//========================================================================
// MMD instantiation
//========================================================================
mmd #(
    .WD(10)
) mmd_inst (
    .MMD_MUX_OUT    (ckvd),
    .CLK            (ckv),
    .CLKN           (),
    .EN_MMD_MUX     (EN_MMD_MUX),//1'b1
    .Rn             (MMD_RN),
    .VDD_0p9        (VD),
    .VSS_0p9        (VS)
);

//========================================================================
// Chopper instantiation
//========================================================================
chopper #(
    .DELAY(50.0)
) chopper_inst (
    .VDD_0V9        (VD),
    .VSS_0V9        (VS),
    .IN0            (FREF),
    .IN1            (ckvd),
    .CHOPPER_SEL    (Chopping_EN),
    .OUT0           (MAIN_DTC_clk_ref),
    .OUT1           (OFFSET_DTC_clk_ref)
);

//========================================================================
// CKR selection
//========================================================================
assign CKR = DTC_TEST_MODE ? FREF : ckvd;

//========================================================================
// Main DTC instantiation
//========================================================================
DTC_analog_10b #(
    .sigma_DNL      (1),        // 1ps DNL
    .reso_binary    (0.5),      // 0.5ps per binary LSB
    .reso_unary     (8),        // 8ps per unary bit
    .base_delay     (339),      // 339ps base delay
    .INL_MISMATCH   (0.0)
) DTC_analog_10b_inst (
    .vdd_ref        (VD),
    .vdd            (VD),
    .gnd            (VS),
    .DTC_DL_cap_ctrl            (MAIN_DTC_DL_cap_ctrl),//5'd0
    .DTC_Iref                   (1'b1),//1'b1
    .DTC_vtop_tran              (1'b1),//1'b1
    .DTC_clk_ref                (MAIN_DTC_clk_ref),
    .DTC_top_cap_ctrl_4bit_binary   (DTC_P_top_4b),
    .DTC_top_cap_ctrl_6bit_unary    (DTC_P_top_6u),
    .DTC_bot_cap_ctrl_4bit_unary    (DTC_P_bot_4u),
    .DTC_pre_clk_test           (MAIN_DTC_pre_clk_test),//VD
    .DTC_pre_clk_sel            (MAIN_DTC_pre_clk_sel),//VS
    .OUT                        (MAIN_DTC_clk_delay)
);

//========================================================================
// Offset DTC instantiation
//========================================================================
assign OFFSET_DTC_in = DTC_TEST_MODE ? MAIN_DTC_clk_ref : OFFSET_DTC_clk_ref;

DTC_analog_10b #(
    .sigma_DNL      (1),
    .reso_binary    (0.5),
    .reso_unary     (8),
    .base_delay     (339),
    .INL_MISMATCH   (0.1)
) Offset_DTC_analog_10b_inst (
    .vdd_ref        (VD),
    .vdd            (VD),
    .gnd            (VS),
    .DTC_DL_cap_ctrl            (OFFSET_DTC_DL_cap_ctrl),//5'd0
    .DTC_Iref                   (1'b1),
    .DTC_vtop_tran              (1'b1),
    .DTC_clk_ref                (OFFSET_DTC_in),
    .DTC_top_cap_ctrl_4bit_binary   (DTC_N_top_4b),
    .DTC_top_cap_ctrl_6bit_unary    (DTC_N_top_6u),
    .DTC_bot_cap_ctrl_4bit_unary    (DTC_N_bot_4u),
    .DTC_pre_clk_test           (1'b0),
    .DTC_pre_clk_sel            (1'b0),
    .OUT                        (OFFSET_DTC_clk_delay)
);

//========================================================================
// TDC instantiation
//========================================================================
assign TDC_EN = 1'b1;  // Always enabled for this testbench
/*
TDC #(
    .TDC_BITS(8)
) TDC_inst (
    .fref           (MAIN_DTC_clk_delayi),
    .fdiv           (OFFSET_DTC_clk_delay),
    .TDC_EN         (TDC_EN),
    .TDC            (TDC_Q)
);*/

TDC #(
    .TDC_BITS(8)
) TDC_inst (
    .fref           (OFFSET_DTC_clk_delay),
    .fdiv           (MAIN_DTC_clk_delayi),
    .TDC_EN         (TDC_EN),
    .TDC            (TDC_Q)
);

//========================================================================
// module DTC DEMUX for test mode
//========================================================================
assign MAIN_DTC_clk_delayi = MAIN_DTC_TEST_EN ? 1'b0 : MAIN_DTC_clk_delay;
assign MAIN_DTC_DEMUX_TEST = MAIN_DTC_TEST_EN ? MAIN_DTC_clk_delay : 1'b0;

initial begin
    MAIN_DTC_DIV_OUT <= 1'b0;
end

always @(posedge MAIN_DTC_DEMUX_TEST) begin
    MAIN_DTC_DIV_OUT <= ~MAIN_DTC_DIV_OUT;
end

assign MAIN_DTC_TEST_OUTPUT = MAIN_DTC_DIV_OUT;

//========================================================================
// SPI Master Tasks
//========================================================================

// SPI write task - write 1 byte to specified address
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

// SPI read task - read 1 byte from specified address
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
// Test Sequence
//========================================================================
reg [7:0] read_data;

initial begin
    // Initialize signals
    SCLK = 1'b0;
    MOSI = 1'b0;
    RST_N = 1'b0;
    Reserved_IN1 = 8'h00;
    Reserved_IN2 = 8'h00;

    //====================================================================
    // STEP 2. Initialize configuration variables (in initial block)
    //====================================================================
    // Basic control parameters
    FCW_I_tmp = 5'd10;
    FCW_F_tmp = 12'b0000_0100_0000;//1000_0100_0000,test1: 4'b0000_0100_0000 for 0.25GHz
    INVKDTC_MANUAL_tmp = 12'd1881;//1881
    TDC_OFT_tmp = 8'b1000_0000;
    INVKDCO_tmp = 8'd120;
    ALPHA_tmp = 4'd3;
    RHO_tmp = 5'd8;

    // Enable controls
    TUNE_I_EN_tmp = 1'b1;
    DTCQ_SD_EN_tmp = 1'b0;
    OTW_MANUAL_tmp = 13'd0;
    OTW_MANUAL_EN_tmp = 1'b0;
    BB_EN_tmp = 1'b1;

    // DCO and DTC controls
    MAIN_DTC_TEST_EN_tmp = 1'b0;
    DCO_DEM_EN_tmp = 1'b0;

    // Calibration and test modes
    TDC_TEST_MODE_tmp = 1'b0;

    // DTC test parameters
    DTC_TEST_P1_tmp = 8'd20;
    DTC_TEST_P2_tmp = 8'd20;
    DTC_TEST_DCW1_tmp = 10'd2;
    DTC_TEST_DCW2_tmp = 10'd3;
    DTC_TEST_EDGE_SEL_tmp = 1'b0;
    DTC_TEST_MODE_tmp = 1'b0;

    // Control freeze signals
    INVKDTC_OUT2SPI_FREEZE_tmp = 1'b1;
    OFTDTC_OUT2SPI_FREEZE_tmp = 1'b1;

    // DTC control values
    OFFSET_DTC_CTRL_manual_tmp = 10'd100;
    DTC_CTRL_manual_tmp = 10'd50;
    RST_N_DIGMAIN_tmp = 1'b1;


    // Signals for extract signals from digital top to ana SPI/dig input
    DCO_BUFFER_ENi_tmp = 1'b0;
    DCO_TEST_BUFFER_ENi_tmp = 1'b0;
    //DCO_coarse_ENi_tmp = 6'd20;
        DCO_coarse_ENi_tmp = 6'b101011;

    TDC_EN_tmp = 1'b1;
    DCO_Tail_ENi_tmp = 5'd0;

    MAIN_DTC_TEST_EN_tmp = 1'b0;
    MAIN_DTC_pre_clk_sel_tmp = VS;
    MAIN_DTC_pre_clk_test_tmp = VD;
    MAIN_DTC_DL_cap_ctrl_tmp = 5'd0;

    EN_MMD_MUX_tmp = 1'b1;
    OFFSET_DTC_pre_clk_sel_tmp = 1'b0;
    OFFSET_DTC_pre_clk_test_tmp = 1'b0;
    OFFSET_DTC_DL_cap_ctrl_tmp = 5'd0;


    DCO_SD_EN_tmp = 1'b0;
	CHOPPER_EN_tmp = 1'b0;
    CAL_INVKDTC_EN_tmp = 1'b0;
    CAL_OFTDTC_EN_tmp = 1'b0;
    MASH_SEL_tmp    = 1'b0;  // 0=MASH1, 1=MASH1-1
    DIFF_EN_tmp     = 1'b1;  // 1=DIFF mode
    CAL_POLY_EN_tmp = 1'b1;  // off by default


    TDC_OUT2SPI_FREEZE_tmp = 1'b1;

    
    $display("===========================================");
    $display("Digital Top Testbench Started");
    $display("Time: %0t", $time);
    $display("===========================================");
    
    // Reset sequence
    #10_000_000;
    RST_N = 1'b1;
    $display("Time %0t: SPI Reset released", $time);
    #10_000_000;
//
    //====================================================================
    // SPI Configuration Phase
    //====================================================================
    $display("\n--- SPI Configuration Phase Started ---");
    // Configure FCW_I (OUT01)
    spi_unit_write(7'd01, spi_out01);
    $display("Time %0t: Written FCW_I = %0d", $time, spi_out01);
    
    // Configure FCW_F (OUT02, OUT03)
    spi_unit_write(7'd02, spi_out02);
    $display("Time %0t: Written FCW_F[7:0] = 0x%0h", $time, spi_out02);

    // Configure FCW_F[11:8] (OUT03)
    spi_unit_write(7'd03, spi_out03);
    $display("Time %0t: Written FCW_F[11:8] = 0x%0h", $time, spi_out03);

    // Configure FCW_F[11:8] (OUT04)
    spi_unit_write(7'd04, spi_out04);
    $display("Time %0t: Written TDC_OFT = 0x%0h", $time, spi_out04);   

    // Configure INVKDTC_MANUAL (OUT05, OUT06)
    spi_unit_write(7'd05, spi_out05);
    $display("Time %0t: Written INVKDTC_MANUAL = 0x%0h", $time, spi_out05);  

    // Configure INVKDTC_MANUAL[12:8] (OUT06)
    spi_unit_write(7'd06, spi_out06);
    $display("Time %0t: Written INVKDTC_MANUAL[12:8] = 0x%0h", $time, spi_out06);

    // Configure INVKDCO (OUT07)
    spi_unit_write(7'd07, spi_out07);
    $display("Time %0t: Written INVKDCO = 0x%0h", $time, spi_out07);

    // Configure ALPHA (OUT08)
    spi_unit_write(7'd08, spi_out08);
    $display("Time %0t: Written ALPHA = 0x%0h", $time, spi_out08);

    // Configure RHO (OUT09)
    spi_unit_write(7'd09, spi_out09);
    $display("Time %0t: Written RHO = 0x%0h", $time, spi_out09);

    // Configure TUNE_I_EN and TDC_OUT2SPI_FREEZE (OUT10)
    spi_unit_write(7'd10, spi_out10);
    $display("Time %0t: Written OUT10 = 0x%0h", $time, spi_out10);

    // Configure DTCQ_SD_EN (OUT11)
    spi_unit_write(7'd11, spi_out11);
    $display("Time %0t: Written OUT11 = 0x%0h", $time, spi_out11);

    // Configure OTW_MANUAL[7:0] (OUT12)
    spi_unit_write(7'd12, spi_out12);
    $display("Time %0t: Written OTW_MANUAL[7:0] = 0x%0h", $time, spi_out12);

    // Configure OTW_MANUAL[12:8] (OUT13)
    spi_unit_write(7'd13, spi_out13);
    $display("Time %0t: Written OTW_MANUAL[12:8] = 0x%0h", $time, spi_out13);

    // Configure OTW_MANUAL_EN, BB_EN (OUT14)
    spi_unit_write(7'd14, spi_out14);
    $display("Time %0t: Written OTW_MANUAL_EN & BB_EN = 0x%0h", $time, spi_out14);

    // Configure DCO_SD_EN, MAIN_DTC_TEST_EN (OUT15)
    spi_unit_write(7'd15, spi_out15);
    $display("Time %0t: Written DCO_SD_EN & MAIN_DTC_TEST_EN = 0x%0h", $time, spi_out15);

    // Configure CHOPPER_EN, DCO_DEM_EN (OUT16)
    spi_unit_write(7'd16, spi_out16);
    $display("Time %0t: Written CHOPPER_EN, DCO_DEM_EN = 0x%0h", $time, spi_out16);

    // Configure Calibration & TDC_TEST_MODE (OUT17)
    spi_unit_write(7'd17, spi_out17);
    $display("Time %0t: Written CAL_xxx & TDC_TEST_MODE = 0x%0h", $time, spi_out17);

    // Configure DTC_TEST_P1 (OUT18)
    spi_unit_write(7'd18, spi_out18);
    $display("Time %0t: Written DTC_TEST_P1 = 0x%0h", $time, spi_out18);

    // Configure DTC_TEST_P2 (OUT19)
    spi_unit_write(7'd19, spi_out19);
    $display("Time %0t: Written DTC_TEST_P2 = 0x%0h", $time, spi_out19);

    // Configure DTC_TEST_DCW1[7:0] (OUT20)
    spi_unit_write(7'd20, spi_out20);
    $display("Time %0t: Written DTC_TEST_DCW1[7:0] = 0x%0h", $time, spi_out20);

    // Configure DTC_TEST_DCW1[9:8] (OUT21)
    spi_unit_write(7'd21, spi_out21);
    $display("Time %0t: Written DTC_TEST_DCW1[9:8] = 0x%0h", $time, spi_out21);

    // Configure DTC_TEST_DCW2[7:0] (OUT22)
    spi_unit_write(7'd22, spi_out22);
    $display("Time %0t: Written DTC_TEST_DCW2[7:0] = 0x%0h", $time, spi_out22);

    // Configure DTC_TEST_DCW2[9:8] (OUT23)
    spi_unit_write(7'd23, spi_out23);
    $display("Time %0t: Written DTC_TEST_DCW2[9:8] = 0x%0h", $time, spi_out23);

    // Configure DTC_TEST_EDGE_SEL, DTC_TEST_MODE (OUT24)
    spi_unit_write(7'd24, spi_out24);
    $display("Time %0t: Written DTC_TEST_EDGE_SEL & DTC_TEST_MODE = 0x%0h", $time, spi_out24);

    // Configure Freeze signals (OUT25)
    spi_unit_write(7'd25, spi_out25);
    $display("Time %0t: Written Freeze Control = 0x%0h", $time, spi_out25);

    // Configure OFFSET_DTC_CTRL_manual[7:0] (OUT26)
    spi_unit_write(7'd26, spi_out26);
    $display("Time %0t: Written OFFSET_DTC_CTRL_manual[7:0] = 0x%0h", $time, spi_out26);

    // Configure OFFSET_DTC_CTRL_manual[9:8] (OUT27)
    spi_unit_write(7'd27, spi_out27);
    $display("Time %0t: Written OFFSET_DTC_CTRL_manual[9:8] = 0x%0h", $time, spi_out27);
    

    // Configure DTC_CTRL_manual[7:0] (OUT28)
    spi_unit_write(7'd28, spi_out28);
    $display("Time %0t: Written DTC_CTRL_manual[7:0] = 0x%0h", $time, spi_out28);

    // Configure DTC_CTRL_manual[9:8], RST_N_DIGMAIN (OUT29)
    spi_unit_write(7'd29, spi_out29);
    $display("Time %0t: Written DTC_CTRL_manual[9:8] & RST_N_DIGMAIN = 0x%0h", $time, spi_out29);

    // Configure Reserved_OUT1 (OUT30)
    spi_unit_write(7'd30, spi_out30);
    $display("Time %0t: Written Reserved_OUT1 = 0x%0h", $time, spi_out30);

    // Configure Reserved_OUT2 (OUT31)
    spi_unit_write(7'd31, spi_out31);
    $display("Time %0t: Written Reserved_OUT2 = 0x%0h", $time, spi_out31);

    // Configure Reserved_OUT3 (OUT32)
    spi_unit_write(7'd32, spi_out32);
    $display("Time %0t: Written Reserved_OUT3 = 0x%0h", $time, spi_out32);

    // Configure Reserved_OUT4 (OUT33)
    spi_unit_write(7'd33, spi_out33);
    $display("Time %0t: Written Reserved_OUT4 = 0x%0h", $time, spi_out33);

    // Configure Reserved_OUT5 (OUT34)
    spi_unit_write(7'd34, spi_out34);
    $display("Time %0t: Written Reserved_OUT5 = 0x%0h", $time, spi_out34);

    // Configure Reserved_OUT6 (OUT35)
    spi_unit_write(7'd35, spi_out35);
    $display("Time %0t: Written Reserved_OUT6 = 0x%0h", $time, spi_out35);

    $display("--- All 35 SPI registers configured successfully ---");
    #10_000_000;
   
    //====================================================================
    // Digital Main Startup Sequence
    //====================================================================
    $display("\n--- Digital Main Startup Sequence ---");
    
    // Pull down RST_N_DIGMAIN (reset digital_main)
    spi_unit_write(7'd29, 8'h00);  // {1'b0, 5'b00000, 2'b00}
    $display("Time %0t: RST_N_DIGMAIN pulled LOW (reset asserted)", $time);
    #10_000_000;
    
    // Release RST_N_DIGMAIN (start digital_main)
    spi_unit_write(7'd29, 8'h80);  // {1'b1, 5'b00000, 2'b00}
    $display("Time %0t: RST_N_DIGMAIN pulled HIGH (reset released, digital_main started)", $time);
    $display("===========================================");
    $display("Digital Main is now running...");
    $display("===========================================\n");
    
    //====================================================================
    // Run for 130us then reconfigure
    //====================================================================
    #130_000_000;  // 130us
    
    $display("\n===========================================");
    $display("Time %0t: Dynamic Reconfiguration Started", $time);
    $display("===========================================");
//
    // Reconfigure calibration enables (OUT17)

    CHOPPER_EN_tmp     = 1'b1;
    CAL_OFTDTC_EN_tmp  = 1'b1;//IF Chopping = 1，must en cal_oftdtc


    CAL_INVKDTC_EN_tmp = 1'b1;

    
    // Configure spi_out16 and display bit-by-bit (CHOPPER_EN, DCO_DEM_EN)

#1

    spi_unit_write(7'd16, spi_out16);
    $display("Time %0t: Written spi_out16 (address 7'd16) = 0x%0h", $time, spi_out16);
    $display("         spi_out16 bit-level breakdown:");
    $display("         - Bit1 (CHOPPER_EN_tmp): 1'b%b", spi_out16[1]);
    $display("         - Bit0 (DCO_DEM_EN_tmp): 1'b%b", spi_out16[0]);

#1
    // Configure spi_out17 and display bit-by-bit (corresponds to CAL signals and TDC_TEST_MODE)
    spi_unit_write(7'd17, spi_out17);
    $display("\nTime %0t: Written spi_out17 (address 7'd17) = 0x%0h", $time, spi_out17);
    $display("         spi_out17 bit-level breakdown:");
    $display("         - Bit3 (CAL_INVKDTC_EN_tmp)  : 1'b%b (INVKDTC Calibration Enable)", spi_out17[3]);
    $display("         - Bit1 (CAL_OFTDTC_EN_tmp)   : 1'b%b (OFTDTC Calibration Enable)", spi_out17[1]);
    $display("         - Bit0 (TDC_TEST_MODE_tmp)   : 1'b%b (TDC Test Mode Switch)", spi_out17[0]); 
    
    $display("===========================================");
    $display("Dynamic Reconfiguration Completed");
    $display("===========================================\n");
    
    //====================================================================
    // Continue simulation
    //====================================================================
    #500_000_000;  // 
    
    $display("\n===========================================");
    $display("Testbench Completed");
    $display("Total simulation time: %0t", $time);
    $display("===========================================");
    $finish;
end

endmodule