`timescale 1ns / 1ps

module pul_generate(
    input clk,
    input rst,
    input start,//单次脉冲输出开始信号
    input [31:0] pul_data,//脉冲宽度
    output reg pul_out,//脉冲输出
    output reg done//单次脉冲输出完成信号
    );

    reg [15:0] gap_value;
    reg [15:0] count;

    localparam START =2'b00;
    localparam RUN =2'b01;

    reg [1:0] state;

    //当外部传来start信号时，将外部传来的脉冲宽度赋值给gap_value
    always@(posedge clk or negedge rst)begin
        if(rst)begin
            gap_value <= 16'd0;
        end
        else if(start)
            gap_value <= pul_data;
        else
            gap_value <= gap_value;
    end
    //脉冲输出状态机
    //START:开始状态，检测到start信号为1，状态转换为RUN
    //RUN:计数等于间隔值时，计数清零，状态转换为START;
    always@(posedge clk or negedge rst)begin
        if(rst)begin
            state <= START;
            count <= 16'd0;
        end
        else begin
            case(state)
                START:begin
                    if(start)begin
                        state <= RUN;
                    end
                    else begin
                        state <= START;
                    end
                end
                RUN:begin
                    if(count == gap_value-1'b1)begin
                        state <= START;
                        count <= 16'd0;
                    end
                    else if(count==gap_value/2-2&&!start)begin//为了使单个脉冲输出停止，在脉冲输出低电平翻转前检测到start信号为0时，停止输出，若已翻转，则继续输出
                        state <= START;
                        count <= 16'd0;
                    end
                    else begin
                        count <= count + 1'b1;
                    end                   
                end

                default :begin
                    state <= START;
                    count <= 16'd0;
                end
            endcase
        end
    end

    //脉冲输出,计数值小于间隔值的一半时为0，大于间隔值的一半时为1
    //计数值等于间隔值时，产生done信号
    always@(posedge clk or negedge rst)begin
        if(rst)begin
            pul_out <= 1'b0;
            done <= 1'b0;
        end
        else begin
            if(count == gap_value-1'b1)begin
                pul_out <= 1'b0;
                done <= 1'b1;
            end
            else if(count==gap_value/2-1)begin
                pul_out <= 1'b1;
                done <= 1'b0;
            end
            else
                done <= 1'b0;           
        end
    end
    
endmodule


