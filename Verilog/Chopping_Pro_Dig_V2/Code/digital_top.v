
`timescale 1ps/1fs

module digital_top (
    // SPI Interface
    input  wire        SCLK,           // SPI clock
    input  wire        RST_N,          // Asynchronous negative reset
    input  wire        MOSI,           // SPI master-output slave-input
    output wire        MISO,           // SPI master-input slave-output
    
    // Clock inputs
    input  wire        CKR,            // Reference clock for digital_main (100MHz)
    input  wire        CLK_SD,         // Clock for sd_mod (600MHz) from DCO
    
    // TDC input from analog
    input  wire [7:0]  TDC_Q,          // TDC quantization result

    input  wire [7:0]  Reserved_IN1,
    input  wire [7:0]  Reserved_IN2,
    
    // DCO tuning words output
    output wire [62:0]  OTW_T_H,       // DCO tuning word - high (thermometer)
    output wire [126:0] OTW_T_L,       // DCO tuning word - low (thermometer)
    output wire [7:0]   DCO_sdm,       // DCO sigma-delta output
    
    // Main DTC control output
    output wire [3:0]   DTC_P_top_4b,    // Top caps 4-bit binary
    output wire [62:0]  DTC_P_top_6u,    // Top caps 6-bit unary
    output wire [14:0]  DTC_P_bot_4u,    // Bottom caps 4-bit unary
    
    // Offset DTC control output
    output wire [3:0]   DTC_N_top_4b,
    output wire [62:0]  DTC_N_top_6u,
    output wire [14:0]  DTC_N_bot_4u,
    
    // Other control outputs
    output wire         Chopping_EN,       // Chopper swap enable
    output wire [4:0]   MMD_RN,        // MMD division ratio

    output wire [7:0]  Reserved_OUT1,
    output wire [7:0]  Reserved_OUT2,
    output wire [7:0]  Reserved_OUT3,
    output wire [7:0]  Reserved_OUT4,
    output wire [7:0]  Reserved_OUT5,
    output wire [7:0]  Reserved_OUT6
);

    //========================================================================
    // Internal signal declarations
    //========================================================================
    
    // Signals from SPI_slave to digital_main (configuration parameters)
    wire [4:0]  FCW_I;
    wire [11:0] FCW_F;
    wire [7:0]  TDC_OFT;
    wire        DTCQ_SD_EN;
    wire        BB_EN;
    wire [11:0] INVKDTC_MANUAL;
    wire [7:0]  INVKDCO;
    wire [3:0]  ALPHA;
    wire [4:0]  RHO;
    wire        TUNE_I_EN;
    wire [12:0] OTW_MANUAL;
    wire        OTW_MANUAL_EN;
    wire        DCO_SD_EN;
    wire        MAIN_DTC_TEST_EN;
    wire        TDC_TEST_MODE;
    wire        CHOPPER_EN;
    wire        DCO_DEM_EN;
    wire        CAL_INVKDTC_EN;
    wire        CAL_OFTDTC_EN;
    wire [7:0]  DTC_TEST_P1;
    wire [7:0]  DTC_TEST_P2;
    wire [9:0]  DTC_TEST_DCW1;
    wire [9:0]  DTC_TEST_DCW2;
    wire        DTC_TEST_EDGE_SEL;
    wire        DTC_TEST_MODE;
    wire [9:0]  OFFSET_DTC_CTRL_manual;
    wire [9:0]  DTC_CTRL_manual;
    wire        RST_N_DIGMAIN;
    wire        TDC_OUT2SPI_FREEZE;
    wire        INVKDTC_OUT2SPI_FREEZE;
    wire        OFTDTC_OUT2SPI_FREEZE;
    
    // Signals from digital_main to SPI_slave (status feedback)
    wire [7:0]  TDC_OUT2SPI;
    wire [11:0] INVKDTC_OUT2SPI;
    wire [11:0] OFTDTC_OUT2SPI;
    
    // Signals between digital_main and sd_mod
    wire [4:0]  OTW_SD;
    wire        DCO_SD_EN_Q;

    // Unconnected wires for unused or partially used SPI output ports
    wire [7:0]  spi_out01_full;
    wire [7:0]  spi_out03_full;
    wire [7:0]  spi_out06_full;
    wire [7:0]  spi_out08_full;
    wire [7:0]  spi_out09_full;
    wire [7:0]  spi_out10_full;
    wire [7:0]  spi_out11_full;
    wire [7:0]  spi_out13_full;
    wire [7:0]  spi_out14_full;
    wire [7:0]  spi_out15_full;
    wire [7:0]  spi_out16_full;
    wire [7:0]  spi_out17_full;
    wire [7:0]  spi_out21_full;
    wire [7:0]  spi_out23_full;
    wire [7:0]  spi_out24_full;
    wire [7:0]  spi_out25_full;
    wire [7:0]  spi_out27_full;
    wire [7:0]  spi_out29_full;
    wire [7:0]  spi_unconnected_36;
    wire [7:0]  spi_unconnected_37;
    wire [7:0]  spi_unconnected_38;
    wire [7:0]  spi_unconnected_39;
    wire [7:0]  spi_unconnected_40;

    // Extract used bits from full output wires
    assign FCW_I = spi_out01_full[4:0];
    assign FCW_F[11:8] = spi_out03_full[3:0];
    assign INVKDTC_MANUAL[11:8] = spi_out06_full[3:0];
    assign ALPHA = spi_out08_full[3:0];
    assign RHO = spi_out09_full[4:0];
    assign {TUNE_I_EN, TDC_OUT2SPI_FREEZE} = spi_out10_full[1:0];
    assign DTCQ_SD_EN = spi_out11_full[0];
    assign OTW_MANUAL[12:8] = spi_out13_full[4:0];
    assign {OTW_MANUAL_EN, BB_EN} = spi_out14_full[1:0];
    assign {DCO_SD_EN, MAIN_DTC_TEST_EN} = spi_out15_full[1:0];
    assign {CHOPPER_EN, DCO_DEM_EN} = spi_out16_full[1:0];
    assign {CAL_INVKDTC_EN, CAL_OFTDTC_EN, TDC_TEST_MODE} = {spi_out17_full[3], spi_out17_full[1], spi_out17_full[0]};
    assign DTC_TEST_DCW1[9:8] = spi_out21_full[1:0];
    assign DTC_TEST_DCW2[9:8] = spi_out23_full[1:0];
    assign {DTC_TEST_EDGE_SEL, DTC_TEST_MODE} = spi_out24_full[1:0];
    assign {INVKDTC_OUT2SPI_FREEZE, OFTDTC_OUT2SPI_FREEZE} = spi_out25_full[1:0];
    assign OFFSET_DTC_CTRL_manual[9:8] = spi_out27_full[1:0];
    assign {RST_N_DIGMAIN, DTC_CTRL_manual[9:8]} = {spi_out29_full[7], spi_out29_full[1:0]};

    //========================================================================
    // SPI Slave instantiation
    //========================================================================
    SPI_slave SPI_slave_inst (
        // SPI interface
        .SCLK       (SCLK),
        .RST_N      (RST_N),
        .MOSI       (MOSI),
        .MISO       (MISO),
        
        // Input ports (status feedback from digital_main)
        .IN01       (TDC_OUT2SPI),
        .IN02       (INVKDTC_OUT2SPI[7:0]),
        .IN03       ({4'b0000, INVKDTC_OUT2SPI[11:8]}),
        .IN04       (8'h00),
        .IN05       (8'h00),
        .IN06       (OFTDTC_OUT2SPI[7:0]),
        .IN07       ({4'b0000, OFTDTC_OUT2SPI[11:8]}),
        .IN08       (Reserved_IN1),
        .IN09       (Reserved_IN2),
        .IN10       (8'b01010011),
        
        // Output ports (configuration to digital_main)
        // Note: All outputs must connect to wires, not constants
        .OUT01      (spi_out01_full),          // [4:0] used for FCW_I
        .OUT02      (FCW_F[7:0]),              // Fully used
        .OUT03      (spi_out03_full),          // [3:0] used for FCW_F[11:8]
        .OUT04      (TDC_OFT),                 // Fully used
        .OUT05      (INVKDTC_MANUAL[7:0]),     // Fully used
        .OUT06      (spi_out06_full),          // [3:0] used for INVKDTC_MANUAL[11:8]
        .OUT07      (INVKDCO),                 // Fully used
        .OUT08      (spi_out08_full),          // [3:0] used for ALPHA
        .OUT09      (spi_out09_full),          // [4:0] used for RHO
        .OUT10      (spi_out10_full),          // [1:0] used for TUNE_I_EN, TDC_OUT2SPI_FREEZE
        .OUT11      (spi_out11_full),          // [0] used for DTCQ_SD_EN
        .OUT12      (OTW_MANUAL[7:0]),         // Fully used
        .OUT13      (spi_out13_full),          // [4:0] used for OTW_MANUAL[12:8]
        .OUT14      (spi_out14_full),          // [1:0] used for OTW_MANUAL_EN, BB_EN
        .OUT15      (spi_out15_full),          // [1:0] used for DCO_SD_EN, MAIN_DTC_TEST_EN
        .OUT16      (spi_out16_full),          // [1:0] used for CHOPPER_EN, DCO_DEM_EN
        .OUT17      (spi_out17_full),          // [3:0] used for calibration enables
        .OUT18      (DTC_TEST_P1),             // Fully used
        .OUT19      (DTC_TEST_P2),             // Fully used
        .OUT20      (DTC_TEST_DCW1[7:0]),      // Fully used
        .OUT21      (spi_out21_full),          // [1:0] used for DTC_TEST_DCW1[9:8]
        .OUT22      (DTC_TEST_DCW2[7:0]),      // Fully used
        .OUT23      (spi_out23_full),          // [1:0] used for DTC_TEST_DCW2[9:8]
        .OUT24      (spi_out24_full),          // [1:0] used for edge_sel and test_mode
        .OUT25      (spi_out25_full),          // [2:0] used for freeze signals
        .OUT26      (OFFSET_DTC_CTRL_manual[7:0]), // Fully used
        .OUT27      (spi_out27_full),          // [1:0] used for OFFSET_DTC_CTRL_manual[9:8]
        .OUT28      (DTC_CTRL_manual[7:0]),    // Fully used
        .OUT29      (spi_out29_full),          // [7] for RST_N, [1:0] for DTC_CTRL_manual[9:8]
        .OUT30      (Reserved_OUT1),           // Reserved for future use
        .OUT31      (Reserved_OUT2),           // Reserved for future use
        .OUT32      (Reserved_OUT3),           // Reserved for future use
        .OUT33      (Reserved_OUT4),           // Reserved for future use
        .OUT34      (Reserved_OUT5),           // Reserved for future use
        .OUT35      (Reserved_OUT6),           // Reserved for future use
        .OUT36      (spi_unconnected_36),      // Unconnected
        .OUT37      (spi_unconnected_37),      // Unconnected
        .OUT38      (spi_unconnected_38),      // Unconnected
        .OUT39      (spi_unconnected_39),      // Unconnected
        .OUT40      (spi_unconnected_40)       // Unconnected
    );
    
    wire MASH_SEL;
    wire DIFF_EN;
    wire CAL_POLY_EN;

    // spi_out17_full bit 分配：
    //   [0] TDC_TEST_MODE
    //   [1] CAL_OFTDTC_EN
    
    //   [3] CAL_INVKDTC_EN
    //   [4] MASH_SEL      (0=MASH1, 1=MASH1-1)
    //   [5] DIFF_EN        (0=SE,    1=DIFF)
    //   [6] CAL_POLY_EN   (0=off,   1=on)
    //   [7] reserved
    assign MASH_SEL    = spi_out17_full[4];
    assign DIFF_EN     = spi_out17_full[5];
    assign CAL_POLY_EN = spi_out17_full[6];
    //========================================================================
    // Digital Main instantiation
    //========================================================================
    digital_main digital_main_inst (
        .CKR                        (CKR),
        .RST_N_SPI                  (RST_N_DIGMAIN),
        .FCW_I                      (FCW_I),
        .FCW_F                      (FCW_F),
        .TDC_Q                      (TDC_Q),
        .TDC_OFT                    (TDC_OFT),
        .DTCQ_SD_EN                 (DTCQ_SD_EN),
        .BB_EN                      (BB_EN),
        .INVKDTC_MANUAL             (INVKDTC_MANUAL),
        .TDC_OUT2SPI                (TDC_OUT2SPI),
        .TDC_OUT2SPI_FREEZE         (TDC_OUT2SPI_FREEZE),
        .INVKDCO                    (INVKDCO),
        .ALPHA                      (ALPHA),
        .RHO                        (RHO),
        .TUNE_I_EN                  (TUNE_I_EN),
        .OTW_T_H                    (OTW_T_H),
        .OTW_T_L                    (OTW_T_L),
        .OTW_MANUAL_EN              (OTW_MANUAL_EN),
        .OTW_MANUAL                 (OTW_MANUAL),
        .OTW_SD                     (OTW_SD),
        .DCO_SD_EN                  (DCO_SD_EN),
        .DCO_SD_EN_Q                (DCO_SD_EN_Q),
        .DCO_DEM_EN                 (DCO_DEM_EN),
        .DTC_P_top_4b                 (DTC_P_top_4b),
        .DTC_P_top_6u                 (DTC_P_top_6u),
        .DTC_P_bot_4u                 (DTC_P_bot_4u),
        .DTC_N_top_4b          (DTC_N_top_4b),
        .DTC_N_top_6u          (DTC_N_top_6u),
        .DTC_N_bot_4u          (DTC_N_bot_4u),
        .MAIN_DTC_TEST_EN           (MAIN_DTC_TEST_EN),
        .Chopping_EN                    (Chopping_EN),
        .CHOPPER_EN                 (CHOPPER_EN),
        .MASH_SEL                    (MASH_SEL),
        .DIFF_EN                     (DIFF_EN),
        .CAL_INVKDTC_EN             (CAL_INVKDTC_EN),
        .CAL_OFTDTC_EN              (CAL_OFTDTC_EN),
        .CAL_POLY_EN                (CAL_POLY_EN),
        .MMD_RN                     (MMD_RN),
        .DTC_TEST_P1                (DTC_TEST_P1),
        .DTC_TEST_P2                (DTC_TEST_P2),
        .DTC_TEST_DCW1              (DTC_TEST_DCW1),
        .DTC_TEST_DCW2              (DTC_TEST_DCW2),
        .DTC_TEST_EDGE_SEL          (DTC_TEST_EDGE_SEL),
        .DTC_TEST_MODE              (DTC_TEST_MODE),
        .INVKDTC_OUT2SPI_FREEZE     (INVKDTC_OUT2SPI_FREEZE),
        .OFTDTC_OUT2SPI_FREEZE      (OFTDTC_OUT2SPI_FREEZE),
        .INVKDTC_OUT2SPI            (INVKDTC_OUT2SPI),
        .OFTDTC_OUT2SPI             (OFTDTC_OUT2SPI),
        .TDC_TEST_MODE              (TDC_TEST_MODE),
        .OFFSET_DTC_CTRL_manual     (OFFSET_DTC_CTRL_manual),
        .DTC_CTRL_manual            (DTC_CTRL_manual),
        .RSV0                       (8'h00),
        .RSV1                       (8'h00)
    );
    
    //========================================================================
    // SD Modulator instantiation
    //========================================================================
    sd_mod sd_mod_inst (
        .clk        (CLK_SD),
        .rst_n      (DCO_SD_EN_Q),
        .sdin       (OTW_SD),
        .div_therm  (DCO_sdm)
    );

endmodule