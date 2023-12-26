`timescale 1ns / 1ps


module motor_ctrl(
    input  clk,
    input  rst,
    input   en,//使能读FIFO信号
    input  write,//写FIFO信号
    input pul_rst,//脉冲复位信号
    input [1:0] pul_mode,//脉冲模式选择信号
    input pul_stop,//停止脉冲输出信号
    input [31:0] step,//脉冲输出步数
    input [15:0] accel_end,//加速结束步数
    input [15:0] decel_begin,//减速开始步数
    input [15:0] indata,//FIFO写入数据
    input pul_dir,//脉冲方向信号
    input pos_clr,//电机位置清零信号
    output pul_out,//脉冲输出信号
    output reg wr_enable,//允许读写FIFO信号
    output done,//单次脉冲输出完成信号
    output pul_state,//电机运动状态信号
    output reg [31:0] step_pos,//电机位置反馈
    output [31:0] step_speed//电机速度反馈
    );


    wire [15:0] outdata;
    reg [31:0] pul_cnt;
    wire full;
    wire empty;
    reg pul_en;
    reg r_en;
    wire rst_n;
    wire w_en;
    reg pul_over;

    assign step_speed = outdata;

    //脉冲生成模块
     pul_generate  pul_generate_0 (
    .clk(clk),
    .rst(rst_n),
    .start(pul_en),
    .pul_data(outdata),
    .pul_out(pul_out),
    .done(done)
    );
    //FIFO模块
    fifo_generator_0 fifo (
  .rst(rst_n),                      // input wire rst
  .wr_clk(clk),                // input wire wr_clk
  .rd_clk(clk),                // input wire rd_clk
  .din(indata),                      // input wire [15 : 0] din
  .wr_en(w_en),                  // input wire wr_en
  .rd_en(r_en),                  // input wire rd_en
  .dout(outdata),                    // output wire [15 : 0] dout
  .full(full),                    // output wire full
  .empty(empty)                 // output wire empty
);

reg adcel_mode;//加减速模式
reg uniform_mode;//匀速模式

//加减速模式状态机
//R_START:开始状态，在FIFO非空状态下检测到start信号，状态转换为R_FIFO
//R_FIFO:读取FIFO状态，FIFO中装的是加减速状态下的脉冲宽度，当加速阶段结束时，状态转换为R_WAIT，当减速阶段完成后，状态转换为R_FIFO
//R_WAIT:匀速状态，当减速阶段开始时，状态转换为R_FIFO
localparam      R_START     = 2'b01	;
localparam      R_FIFO     	= 2'b10	;
localparam      R_WAIT     	= 2'b11	;


reg [1:0] r_state;//加减速模式状态
reg r_first;//第一次读取FIFO信号
reg [7:0] r_cnt;//复位后等待一定时间，才能对FIFO进行读写操作
//reg wr_enable;//复位后，允许读写FIFO信号
reg prev_write;

always@(posedge clk or negedge rst_n)begin
    if(rst_n)begin
        r_state <= R_START;
        r_first <=1'b0;
    end
    else begin
        case(r_state)
            R_START:begin
                if(!empty&&!pul_stop&&wr_enable&&adcel_mode&&en&&!pul_over)begin//复位后，empty不会马上置一，所以与上允许读写信号形成可以读取FIFO的条件  
                    r_state <= R_FIFO;
                    r_first <= 1'b1;
                end
                else begin
                    r_state <= R_START;
                end
            end
            R_FIFO:begin
                if(pul_stop||pul_over)begin//当stop信号为1或者脉冲输出完成后，状态转换为R_START
                    r_state <= R_START;
            end
            else if(pul_cnt==accel_end-1&&accel_end!=decel_begin)begin//加速阶段结束后转到R_WAIT状态；如果加速结束步数等于减速开始步数不转换到R_WAIT状态
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

//匀速模式状态机
//U_START:开始状态，在FIFO非空状态下检测到start信号，状态转换为U_RUN
//U_RUN:匀速状态，当stop信号为1时，状态转换为U_STOP,当脉冲输出完成后，状态转换为U_START
//U_STOP:停止状态，当stop信号为0时，状态转换为U_RUN
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
                if(!empty&&wr_enable&&uniform_mode)begin
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


//复位后等待一定时间，才能对FIFO进行读写操作
always@(posedge clk or negedge rst_n)begin
    if(rst_n)begin
        r_cnt <= 8'd0;
        wr_enable <= 1'b0;
    end
    else if(r_cnt==8'd80)begin
        wr_enable <= 1'b1;
    end
    else begin
        r_cnt <= r_cnt + 1'b1;
    end
end

//读信号生成,第一次由开始状态生成（通过检测r_first）
//之后在R_FIFO状态下，当脉冲输出完成时，生成读信号（读FIFO是加减速状态）
always@(posedge clk or negedge rst_n)begin
    if(rst_n)begin
        r_en <= 1'b0;
    end
    else if((r_state==R_FIFO&&done&&(pul_cnt!=step-1))||r_first||u_first)begin
        r_en <= 1'b1;
        end
    else 
        r_en <= 1'b0;
end

//脉冲计数，单次脉冲输出完成时，脉冲计数加1，并且完成设定值step时，计数清零
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

//脉冲使能信号,当状态为加减速模式下的R_FIFO或者R_WAIT，以及匀速模式下U_RUN时，脉冲输出使能信号为1
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

//当脉冲计数等于step时，脉冲输出完成,pul_over置一
always@(posedge clk or negedge rst_n)begin
    if(rst_n)begin
        pul_over <= 1'b0;
    end
    else if(pul_cnt==step)begin
        pul_over <= 1'b1;
    end
    else begin
        pul_en <= pul_en;
    end
end

assign pul_state = pul_en;//为0时，电机停止，为1时，电机运动
assign rst_n=rst||pul_rst;//当rst或者pul_rst为1时，FIFO复位

//写模块,在允许读写情况下，检测到write信号从0变为1时，产生脉冲写使能信号
always @(posedge clk) begin
    if(rst_n) begin
       prev_write <= 0;
    end
    else if(wr_enable) begin
        prev_write <= write;
    end
    else begin
        prev_write <= prev_write;
    end
end
assign w_en = write && !prev_write; 



//模式选择，检测pul_mode信号，01为加减速模式，10为匀速模式
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

//电机位置反馈，
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
