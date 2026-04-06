module    sd_mod    (
                    clk ,
                    rst_n ,
                    sdin ,
                    div_therm 
                    ) ;
input               clk ;
input               rst_n ;
input[4:0]         sdin ;
output[7:0]         div_therm ;

reg [4:0]           acc_1 ;
wire[5:0]          acc_1_sum ;
wire[4:0]          acc_1_new ;
wire                acc_1_ov ;
reg [4:0]           acc_2 ;
wire[5:0]          acc_2_sum ;
wire[4:0]          acc_2_new ;
wire                acc_2_ov ;
reg                 acc_1_ov_d1 ;
reg                 acc_1_ov_d2 ;
reg                 acc_2_ov_d1 ;
reg                 acc_2_ov_d2 ;
reg[7:0]            div_therm ;
wire[2:0]           div_temp ;
wire[2:0]           sum1 ;
wire[2:0]           sum2 ;


always@(posedge clk or negedge rst_n)begin
    if(rst_n == 1'b0)
        acc_1 <= 5'd1 ;
    else
        acc_1 <= acc_1_new ;
end
assign  acc_1_sum = {1'b0 , acc_1} + {1'b0 , sdin} ;
assign  acc_1_new = acc_1_sum[4:0] ;
assign  acc_1_ov = acc_1_sum[5] ;

always@(posedge clk or negedge rst_n)begin
    if(rst_n == 1'b0)
        acc_2 <= 5'd0 ;
    else
        acc_2 <= acc_2_new ;
end
assign  acc_2_sum = {1'b0 , acc_2} + {1'b0 , acc_1} ;
assign  acc_2_new = acc_2_sum[4:0] ;
assign  acc_2_ov = acc_2_sum[5] ;


always@(posedge clk or negedge rst_n)begin
    if(rst_n == 1'b0)begin
        acc_1_ov_d1 <= 1'b0 ;
        acc_1_ov_d2 <= 1'b0 ;
        acc_2_ov_d1 <= 1'b0 ;
        acc_2_ov_d2 <= 1'b0 ;
    end
    else begin
        acc_1_ov_d1 <= acc_1_ov ;
        acc_1_ov_d2 <= acc_1_ov_d1 ;
        acc_2_ov_d1 <= acc_2_ov ;
        acc_2_ov_d2 <= acc_2_ov_d1 ;
    end
end

assign  sum1 = ({1'd0 , acc_1_ov_d2} + {1'd0 , acc_2_ov_d1}) ;//2bit
assign  sum2 = {1'd0 , acc_2_ov_d2} ;//2bit
assign div_temp = {acc_1_ov_d2, acc_2_ov_d1, ~acc_2_ov_d2};
always@(posedge clk or negedge rst_n)
    begin
        if(rst_n == 1'b0)
        begin
            div_therm <= 8'd0 ;//
        end
        else begin
            div_therm <= 8'd255>>(4'd8-div_temp) ;
        end
    end
    
endmodule
