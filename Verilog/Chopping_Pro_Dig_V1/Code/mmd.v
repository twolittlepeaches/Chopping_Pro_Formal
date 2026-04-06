
`timescale 1ps/1fs

module mmd #(parameter WD=20)
(
    output MMD_MUX_OUT,
    input CLK,
    input CLKN,
    input EN_MMD_MUX,//MMD ENABLE
    input [4:0] Rn,
    input VDD_0p9,
    input VSS_0p9
);

wire ckv = CLK;
wire [9:0] N_in = Rn + 6'd32;
reg [WD-1:0] cnt;
reg [WD-1:0] N_sync;
reg [WD-1:0] N_latched; // 模拟实际电路的下降沿锁存器
reg rstn;

initial begin 
    rstn <= 1'b1; #15000 rstn <= 1'b0; #30000 rstn <= 1'b1;
end

// 1. 建模实际电路：下降沿锁存
// 注意：为了避免仿真竞争，这里加入极小的延时或者确保 Rn 是稳定的
always @(negedge MMD_MUX_OUT, negedge rstn) begin
    if (~rstn) begin
        N_latched <= 52;
    end else begin
        // 这里直接采样 Rn，模拟下降沿触发的 DFF
        N_latched <= N_in; 
    end
end

// 2. 核心计数逻辑
// 必须在 ckv 的上升沿判断 cnt，并在此处装载锁存好的 N_latched
always @(posedge ckv, negedge rstn) begin
    if (~rstn) begin
        cnt <= 0;
        N_sync <= 32;
    end else begin
        if (cnt < (N_sync - 1'b1)) begin
            cnt <= #(100) cnt + 1'b1;
        end else begin
            cnt <= #(100) 0;
            // 关键：在这一时刻（第 N 拍起点），从锁存器读入数值
            N_sync <= #(100) N_latched; 
        end
    end
end

// 3. 输出生成
wire ckvd_i = (cnt < {1'b0, N_sync[WD-1:1]});
reg ckvd_inv;
always @(negedge ckv, negedge rstn) begin
    if (~rstn) ckvd_inv <= 0;
    else       ckvd_inv <= #(71) ckvd_i;
end

assign MMD_MUX_OUT = ckvd_i;

endmodule
