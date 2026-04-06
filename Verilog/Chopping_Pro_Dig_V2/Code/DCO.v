`timescale 1ps/1fs 
  
module DCO (
    // Coarse Bank - 6-bit thermometer
    OTW_C,
    // Medium Bank - 4x16-bit thermometer  
    OTW_M1, OTW_M2, OTW_M3, OTW_M4,
    // Fine Bank - 4x32-bit thermometer
    OTW_F1, OTW_F2, OTW_F3, OTW_F4,
    // SD Bank - 3-bit binary
    OTW_sd,
    // Output
    ckv,
    // Power
    VD, VS
);

// Port declarations
input [5:0] OTW_C;              // Coarse bank - thermometer
input [15:0] OTW_M1, OTW_M2, OTW_M3, OTW_M4;  // Medium banks - thermometer
input [31:0] OTW_F1, OTW_F2, OTW_F3, OTW_F4;  // Fine banks - thermometer  
input [7:0] OTW_sd;             // SD bank - thermometer
//input [2:0] OTW_sd;             // SD bank - binary
output ckv;
inout VD, VS;

reg ckv;

// Period control parameters  
real Tc = 222.222;              // Center period corresponding to 4.5GHz (ps)
parameter real base_period = 215;       // Base period corresponding to 5GHz (ps)
parameter real freq_coarse = 172e6;     // 172MHz per coarse unit
parameter real freq_medium = 4e6;       // 4MHz per medium unit  
parameter real freq_fine = 40.7e3;      // 40.7kHz per fine unit
parameter real freq_sd = 40.7e3;        // 40.7kHz per SD unit

// Jitter parameters
parameter real J_Tv = 9e-3;     // Jitter in ps

// Internal variables
real fv, Tv;
real jitter;
integer i;
integer ccw_coarse;
integer ccw_medium; 
integer ccw_fine;
integer ccw_sd;
integer fp1;
integer seed = 11;
parameter integer td = 1000;

// Count thermometer-coded coarse bank
always @(OTW_C) begin
    ccw_coarse = 0;
    for (i = 0; i <= 5; i = i + 1) begin
        if (~OTW_C[i]) ccw_coarse = ccw_coarse + 1;
    end
end

// Count thermometer-coded medium banks
always @(OTW_M1, OTW_M2, OTW_M3, OTW_M4) begin
    ccw_medium = 0;
    // Count OTW_M1
    for (i = 0; i <= 15; i = i + 1) begin
        if (~OTW_M1[i]) ccw_medium = ccw_medium + 1;
    end
    // Count OTW_M2
    for (i = 0; i <= 15; i = i + 1) begin
        if (~OTW_M2[i]) ccw_medium = ccw_medium + 1;
    end
    // Count OTW_M3  
    for (i = 0; i <= 15; i = i + 1) begin
        if (~OTW_M3[i]) ccw_medium = ccw_medium + 1;
    end
    // Count OTW_M4
    for (i = 0; i <= 15; i = i + 1) begin
        if (~OTW_M4[i]) ccw_medium = ccw_medium + 1;
    end
end

// Count thermometer-coded fine banks
always @(OTW_F1, OTW_F2, OTW_F3, OTW_F4) begin
    ccw_fine = 0;
    // Count OTW_F1
    for (i = 0; i <= 31; i = i + 1) begin
        if (~OTW_F1[i]) ccw_fine = ccw_fine + 1;
    end
    // Count OTW_F2
    for (i = 0; i <= 31; i = i + 1) begin
        if (~OTW_F2[i]) ccw_fine = ccw_fine + 1;
    end
    // Count OTW_F3
    for (i = 0; i <= 31; i = i + 1) begin
        if (~OTW_F3[i]) ccw_fine = ccw_fine + 1;
    end
    // Count OTW_F4
    for (i = 0; i <= 31; i = i + 1) begin
        if (~OTW_F4[i]) ccw_fine = ccw_fine + 1;
    end
end

// Decode binary SD bank

always @(OTW_sd) begin
    ccw_sd = 0;
    for (i = 0; i <= 7; i = i + 1) begin
        if (OTW_sd[i]) ccw_sd = ccw_sd + 1;
    end
end

// Calculate period using Delta_t = Delta_f/f^2 relationship
always @(ccw_coarse, ccw_medium, ccw_fine, ccw_sd) begin
    jitter = J_Tv * (0.01 * $dist_uniform(seed, -50, 50));
    
    // Calculate period using original formula structure:
    // Tv = base_period + period_increments_from_each_bank
    // Using Delta_t = Delta_f/f^2, where coefficient = (Tc^2/1e6)
    Tv = base_period + ccw_coarse * (Tc*Tc/1e6) * (freq_coarse/1e6) + 
         ccw_medium * (Tc*Tc/1e6) * (freq_medium/1e6) + 
         ccw_fine * (Tc*Tc/1e6) * (freq_fine/1e6) + 
         ccw_sd * (Tc*Tc/1e6) * (freq_sd/1e6);
    
    // Apply minimum period constraint  
    if (Tv < 150) Tv = 150;  // Minimum period constraint
    
    // Calculate corresponding frequency for reference
    fv = 1e12 / Tv;
end

// Clock generation
initial begin
    ckv = 0;
    ccw_coarse = 0;
    ccw_medium = 0; 
    ccw_fine = 0;
    ccw_sd = 0;
    Tv = Tc;
    fp1 = $fopen("./ckv.txt", "w");
    #td forever begin 
        #(Tv/2 + J_Tv * (0.01 * $dist_uniform(seed, -50, 50))) ckv = ~ckv; 
    end
end

// Log positive edges

always @(posedge ckv) begin
    $fwrite(fp1, "%3.13e \n", $realtime);
end


endmodule
/*
`timescale 1ps/1fs

module DCO (
    // Coarse Bank - 6-bit thermometer
    OTW_C,
    // Medium Bank - 4x16-bit thermometer
    OTW_M1, OTW_M2, OTW_M3, OTW_M4,
    // Fine Bank - 4x32-bit thermometer
    OTW_F1, OTW_F2, OTW_F3, OTW_F4,
    // SD Bank - 3-bit binary
    OTW_sd,
    // Output
    ckv, 
    // Power
    VD, VS
);

// Port declarations
input [5:0] OTW_C;                  // Coarse bank - thermometer
input [15:0] OTW_M1, OTW_M2, OTW_M3, OTW_M4; // Medium banks - thermometer
input [31:0] OTW_F1, OTW_F2, OTW_F3, OTW_F4; // Fine banks - thermometer
input [7:0] OTW_sd;                 // SD bank - thermometer
//input [2:0] OTW_sd;               // SD bank - binary
output ckv;
inout VD, VS;

reg ckv, ckv_n;
reg DCO_OUTPUT_TEST_P, DCO_OUTPUT_TEST_N;

// Period control parameters
real Tc = 222.222;                  // Center period corresponding to 4.5GHz (ps)
parameter real base_period = 215;   // Base period corresponding to 56GHz (ps)
parameter real freq_coarse = 172e6; // 172MHz per coarse unit
parameter real freq_medium = 4e6;   // 4MHz per medium unit
parameter real freq_fine = 40.7e3;  // 40.7kHz per fine unit
parameter real freq_sd = 40.7e3;    // 40.7kHz per SD unit
// Jitter parameters
parameter real J_Tv = 9e-3;         // Jitter in ps

// Internal variables
real current_period;                //
real half_period_with_jitter;       //
integer ccw_coarse, ccw_medium, ccw_fine, ccw_sd;
integer seed = 11;
parameter integer td = 1000;

//------------------------------------------------
function integer count_thermometer_6;
    input [5:0] therm;
    integer i, cnt;
begin
    cnt = 0;
    for (i = 0; i < 6; i = i + 1)
        if (therm[i]) cnt = cnt + 1;
    count_thermometer_6 = cnt;
end
endfunction

//------------------------------------------------
function integer count_thermometer_16;
    input [15:0] therm;
    integer i, cnt;
begin
    cnt = 0;
    for (i = 0; i < 16; i = i + 1)
        if (therm[i]) cnt = cnt + 1;
    count_thermometer_16 = cnt;
end
endfunction

//------------------------------------------------
function integer count_thermometer_32;
    input [31:0] therm;
    integer i, cnt;
begin
    cnt = 0;
    for (i = 0; i < 32; i = i + 1)
        if (therm[i]) cnt = cnt + 1;
    count_thermometer_32 = cnt;
end
endfunction

//------------------------------------------------
function integer count_thermometer_8;
    input [7:0] therm;
    integer i, cnt;
begin
    cnt = 0;
    for (i = 0; i < 8; i = i + 1)
        if (therm[i]) cnt = cnt + 1;
    count_thermometer_8 = cnt;
end
endfunction

//------------------------------------------------
function real calculate_period;
    input integer c_coarse, c_medium, c_fine, c_sd;
    real period;
begin
    period = base_period +
             c_coarse * (Tc*Tc/1e6) * (freq_coarse/1e6) +
             c_medium * (Tc*Tc/1e6) * (freq_medium/1e6) +
             c_fine  * (Tc*Tc/1e6) * (freq_fine/1e6) +
             c_sd    * (Tc*Tc/1e6) * (freq_sd/1e6);

    //
    if (period < 150) period = 150;
    calculate_period = period;
end
endfunction

//------------------------------------------------
function real get_jittered_half_period;
    input real period;
begin
    get_jittered_half_period = (period / 2.0) + (J_Tv * 0.01 * $dist_uniform(seed, -50, 50));
end
endfunction

//------------------------------------------------
// Event-driven current_period
always @(OTW_C or OTW_M1 or OTW_M2 or OTW_M3 or OTW_M4 or
         OTW_F1 or OTW_F2 or OTW_F3 or OTW_F4 or OTW_sd) begin
    //
    ccw_coarse = count_thermometer_6(OTW_C);

    ccw_medium = count_thermometer_16(OTW_M1) +
                 count_thermometer_16(OTW_M2) +
                 count_thermometer_16(OTW_M3) +
                 count_thermometer_16(OTW_M4);

    ccw_fine = count_thermometer_32(OTW_F1) +
               count_thermometer_32(OTW_F2) +
               count_thermometer_32(OTW_F3) +
               count_thermometer_32(OTW_F4);

    ccw_sd = count_thermometer_8(OTW_sd);

    //
    current_period = calculate_period(ccw_coarse, ccw_medium, ccw_fine, ccw_sd);
end

//------------------------------------------------
// Event-driven
initial begin
    ckv = 0;
    ckv_n = 1;
    DCO_OUTPUT_TEST_P = 0;
    DCO_OUTPUT_TEST_N = 0;
    current_period = Tc;
    ccw_coarse = 0;
    ccw_medium = 0;
    ccw_fine = 0;
    ccw_sd = 0;

    // td;
    //#td;

    // fork-join
    fork
        // ckv
        forever begin
            half_period_with_jitter = get_jittered_half_period(current_period);
            #half_period_with_jitter;
            ckv = ~ckv;
            DCO_OUTPUT_TEST_P = ckv;
        end

        // ckv_n
        forever begin
            half_period_with_jitter = get_jittered_half_period(current_period);
            #half_period_with_jitter;
            ckv_n = ~ckv_n;
            DCO_OUTPUT_TEST_N = ckv_n;
        end
    join
end

endmodule*/