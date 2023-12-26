`timescale 1ns / 1ps

module pul_generate(
    input clk,
    input rst,
    input start,//�������������ʼ�ź�
    input [31:0] pul_data,//������
    output reg pul_out,//�������
    output reg done//���������������ź�
    );

    reg [15:0] gap_value;
    reg [15:0] count;

    localparam START =2'b00;
    localparam RUN =2'b01;

    reg [1:0] state;

    //���ⲿ����start�ź�ʱ�����ⲿ�����������ȸ�ֵ��gap_value
    always@(posedge clk or negedge rst)begin
        if(rst)begin
            gap_value <= 16'd0;
        end
        else if(start)
            gap_value <= pul_data;
        else
            gap_value <= gap_value;
    end
    //�������״̬��
    //START:��ʼ״̬����⵽start�ź�Ϊ1��״̬ת��ΪRUN
    //RUN:�������ڼ��ֵʱ���������㣬״̬ת��ΪSTART;
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
                    else if(count==gap_value/2-2&&!start)begin//Ϊ��ʹ�����������ֹͣ������������͵�ƽ��תǰ��⵽start�ź�Ϊ0ʱ��ֹͣ��������ѷ�ת����������
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

    //�������,����ֵС�ڼ��ֵ��һ��ʱΪ0�����ڼ��ֵ��һ��ʱΪ1
    //����ֵ���ڼ��ֵʱ������done�ź�
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


