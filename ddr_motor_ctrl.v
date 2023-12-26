`timescale 1ns / 1ps


module motor_ctrl(
    input  clk,
    input  rst,
    input   en,//ʹ�ܶ�FIFO�ź�
    input pul_rst,//���帴λ�ź�
    input [1:0] pul_mode,//����ģʽѡ���ź�
    input pul_stop,//ֹͣ��������ź�
    input [31:0] step,//�����������
    input [15:0] accel_end,//���ٽ�������
    input [15:0] decel_begin,//���ٿ�ʼ����
    input [31:0] pul_value,//��ps��DDR�ж�����������
    input pul_dir,//���巽���ź�
    input pos_clr,//���λ�������ź�
    output pul_out,//��������ź�
    output reg read,//���������������ź�
    output pul_state,//����˶�״̬�ź�
    output reg [31:0] step_pos,//���λ�÷���
    output [31:0] step_speed//����ٶȷ���
    );

    reg [31:0] pul_cnt;
    reg pul_en;
    wire rst_n;
    reg pul_over;

    assign step_speed = pul_value;

    //��������ģ��
     pul_generate  pul_generate_0 (
    .clk(clk),
    .rst(rst_n),
    .start(pul_en),
    .pul_data(pul_value),
    .pul_out(pul_out),
    .done(done)
    );

reg adcel_mode;//�Ӽ���ģʽ
reg uniform_mode;//����ģʽ

//�Ӽ���ģʽ״̬��
//R_START:��ʼ״̬����FIFO�ǿ�״̬�¼�⵽start�źţ�״̬ת��ΪR_FIFO
//R_FIFO:��ȡFIFO״̬��FIFO��װ���ǼӼ���״̬�µ������ȣ������ٽ׶ν���ʱ��״̬ת��ΪR_WAIT�������ٽ׶���ɺ�״̬ת��ΪR_FIFO
//R_WAIT:����״̬�������ٽ׶ο�ʼʱ��״̬ת��ΪR_FIFO
localparam      R_START     = 2'b01	;
localparam      R_FIFO     	= 2'b10	;
localparam      R_WAIT     	= 2'b11	;


reg [1:0] r_state;//�Ӽ���ģʽ״̬
reg r_first;//��һ�ζ�ȡFIFO�ź�
reg [7:0] r_cnt;//��λ��ȴ�һ��ʱ�䣬���ܶ�FIFO���ж�д����
reg prev_write;

always@(posedge clk or negedge rst_n)begin
    if(rst_n)begin
        r_state <= R_START;
        r_first <=1'b0;
    end
    else begin
        case(r_state)
            R_START:begin
                if(!pul_stop&&adcel_mode&&en&&!pul_over)begin//��λ��empty����������һ���������������д�ź��γɿ��Զ�ȡFIFO������  
                    r_state <= R_FIFO;
                    r_first <= 1'b1;
                end
                else begin
                    r_state <= R_START;
                end
            end
            R_FIFO:begin
                if(pul_stop||pul_over)begin//��stop�ź�Ϊ1�������������ɺ�״̬ת��ΪR_START
                    r_state <= R_START;
            end
            else if(pul_cnt==accel_end-1)begin//���ٽ׶ν�����ת��R_WAIT״̬��������ٽ����������ڼ��ٿ�ʼ������ת����R_WAIT״̬
                r_state <= R_WAIT;   
            end
            else begin
                r_state <= R_FIFO;
            end
                r_first <= 1'b0;
            end
            R_WAIT:begin
                if(pul_stop)
                    r_state <= R_START;
                else if(pul_cnt==decel_begin-1)
                    r_state <= R_FIFO;
                else
                    r_state <= R_WAIT;
            end

            default:r_state <= R_START;
        endcase
    end
end

//����ģʽ״̬��
//U_START:��ʼ״̬����FIFO�ǿ�״̬�¼�⵽start�źţ�״̬ת��ΪU_RUN
//U_RUN:����״̬����stop�ź�Ϊ1ʱ��״̬ת��ΪU_STOP,�����������ɺ�״̬ת��ΪU_START
//U_STOP:ֹͣ״̬����stop�ź�Ϊ0ʱ��״̬ת��ΪU_RUN
localparam      U_START     = 2'b01	;
localparam      U_RUN     	= 2'b10	;
localparam      U_STOP     	= 2'b11	;
reg [1:0] u_state;
reg u_first;

always@(posedge clk or negedge rst_n)begin
    if(rst_n)begin
        u_state <= U_START;
        u_first <= 1'b0;
    end
    else begin
        case(u_state)
            U_START:begin
                if(en&&uniform_mode&&!pul_over)begin
                    u_state <= U_RUN;
                    u_first <= 1'b1;
                end
                else begin
                    u_state <= U_START;
                end
            end
            U_RUN:begin
                if(pul_over)begin
                    u_state <= U_START;
                end
                else if (pul_stop)begin
                    u_state <= U_STOP;
                end
                else begin
                    u_state <= U_RUN;
                end
                u_first <= 1'b0;
            end
            U_STOP:begin
                if(!pul_stop)begin
                    u_state <= U_RUN;
                end
                else begin
                    u_state <= U_STOP;
                end
            end
            default:u_state <= U_START;
        endcase

    end
end



//���ź�����,��һ���ɿ�ʼ״̬���ɣ�ͨ�����r_first��
//֮����R_FIFO״̬�£�������������ʱ�����ɶ������źţ���FIFO�ǼӼ���״̬��
always@(posedge clk or negedge rst_n)begin
    if(rst_n)begin
        read <= 1'b0;
    end
    else if((r_state==R_FIFO&&done&&(pul_cnt!=step-1))||r_first||u_first)begin
        read <= 1'b1;
        end
    else 
        read <= 1'b0;
end

//�����������������������ʱ�����������1����������趨ֵstepʱ����������
always@(posedge clk or negedge rst_n)begin
    if(rst_n)begin
        pul_cnt <= 32'd0;
    end
    else if(done)begin
        pul_cnt <= pul_cnt + 1'b1;
    end
    else begin
        pul_cnt <= pul_cnt;
    end
end

//����ʹ���ź�,��״̬Ϊ�Ӽ���ģʽ�µ�R_FIFO����R_WAIT���Լ�����ģʽ��U_RUNʱ���������ʹ���ź�Ϊ1
always@(posedge clk or negedge rst_n)begin
    if(rst_n)begin
        pul_en <= 1'b0;
    end
    else if(r_state==R_FIFO||r_state==R_WAIT||u_state==U_RUN)begin
        pul_en <= 1'b1;
    end
    else begin
        pul_en <= 1'b0;
    end
end

//�������������stepʱ������������,pul_over��һ
always@(posedge clk or negedge rst_n)begin
    if(rst_n)begin
        pul_over <= 1'b0;
    end
    else if(pul_cnt==step)begin
        pul_over <= 1'b1;
    end
    else begin
        pul_over <= pul_over;
    end
end

assign pul_state = pul_en;//Ϊ0ʱ�����ֹͣ��Ϊ1ʱ������˶�
assign rst_n=rst||pul_rst;//��rst����pul_rstΪ1ʱ��FIFO��λ





//ģʽѡ�񣬼��pul_mode�źţ�01Ϊ�Ӽ���ģʽ��10Ϊ����ģʽ
always @(posedge clk)
begin
  if (rst)
  begin
    adcel_mode <= 0;
    uniform_mode <= 0;
  end
  else case (pul_mode)
    2'b01: begin
      adcel_mode <= 1;
      uniform_mode <= 0;
    end
    2'b10: begin
      adcel_mode <= 0;
      uniform_mode <= 1;
    end
    default: begin 
      adcel_mode <= 0;
      uniform_mode <= 0;
    end
  endcase
end

//���λ�÷�����
always @(posedge clk)
begin
  if (rst||pos_clr)
  begin
	step_pos <= 0;
  end
  else if(done)
  begin
	if(pul_dir) 
	step_pos <= step_pos + 1;
	else 
	step_pos <= step_pos - 1;
  end
  else begin
	step_pos <= step_pos;
  end
  end




endmodule
