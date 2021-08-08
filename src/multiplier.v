

/*module Ascii_to_Binary_TB();
	
	parameter c_CLOCK_PERIOD_NS = 10;
	parameter c_CLKS_PER_BIT = 217;
	parameter c_BIT_PERIOD = 8600;

	reg r_TX_DV = 0;
	reg r_Clock = 0;
	wire w_TX_Active,w_UART_Line;
	wire w_TX_Serial;
	wire [7:0] out;
	reg [7:0] r_TX_Byte = 0;
	wire [7:0] w_RX_Byte;
	wire MHz ;
	parameter s0 = 2'b00, s1 = 2'b01, s2 = 2'b10, s3 = 2'b11;
	reg fsm;
	reg [7:0] a;
	reg [1:0] state;
	
	clk_div MHz_25(.clk(r_Clock),.clk_out(MHz));

	uart_rx #(.CLKS_PER_BIT(c_CLKS_PER_BIT)) UART_RX_Inst
		(.i_Clock(MHz),
		.i_Rx_Serial(w_UART_Line),
		.o_Rx_DV(w_RX_DV),
		.o_Rx_Byte(w_RX_Byte)
		);

	uart_tx #(.CLKS_PER_BIT(c_CLKS_PER_BIT)) UART_TX_Inst
		(.i_Clock(MHz),
		.i_Tx_DV(r_TX_DV),
		.i_Tx_Byte(r_TX_Byte),
		.o_Tx_Active(w_TX_Active),
		.o_Tx_Serial(w_TX_Serial),
		.o_Tx_Done()
		);
		
	task ascii2binary;
	input [7:0] w_RX_byte;
	output [7:0]out;	
	begin
		case (w_RX_byte)
		48: out = 0;
		49: out = 1;
		50: out = 2;
		51: out = 3;
		52: out = 4;
		53: out = 5; 
		54: out = 6;
		55: out = 7;
		56: out = 8;
		57: out = 9;
		endcase
	end
	endtask
	
always @ (posedge clk)begin

	if (!rst)begin
		state = s0; out = 0; a =0 ;	
	end
	else begin
	case (state)
		s0: begin
			fsm =0; 
			if (w_RX_dv) begin
				state = s1;
				ascii2binary(w_RX_Byte,a);
				out = out +  a * 100;
				$display ($realtime, "ns %d", out);
 			end
			else begin
				state = s0;
			end
		end
		s1: begin
			fsm =0; 
			if (w_RX_dv) begin
				state = s2;
				ascii2binary(w_RX_Byte,a);
				out = out + a * 10;
				$display ($realtime, "ns %d", out);
 			end
			else begin
				state = s1;
			end
		end
		s2: begin
			fsm =0; 
			if (w_RX_dv) begin
				state = s3;
				ascii2binary(w_RX_Byte,a);
				out = out + a * 1;
				$display ($realtime, "ns %d", out);
 			end
			else begin
				state = s2;
			end
		end
		s3: begin
			fsm =1;
			if (w_RX_dv == 0) begin
				$display ($realtime, "ns %d", out);
				state = s0;
 			end
			else begin
				state = s3;
			end 
		end
	endcase
	end
end

	assign w_UART_Line = w_TX_Active ? w_TX_Serial:1'b1;

	always
	#(c_CLOCK_PERIOD_NS/2) r_Clock <= !r_Clock;

	initial begin
		@(posedge MHz);
		@(posedge MHz);
		r_TX_DV <= 1'b1;
		r_TX_Byte <= 8'b00110001;
		#3000
		@(posedge MHz);
		r_TX_DV <= 1'b0;
		@(posedge MHz);
		@(posedge MHz);
		r_TX_DV <= 1'b1;
		r_TX_Byte <= 8'b00110000;
		@(posedge MHz);
		r_TX_DV <= 1'b0;
		#3000
		@(posedge MHz);
		@(posedge MHz);
		r_TX_DV <= 1'b1;
		r_TX_Byte <= 8'b00110001;
		@(posedge MHz);
		r_TX_DV <= 1'b0;
		#3000
		$display("%d",out);
		$finish;
	end
endmodule*/


module Ascii_to_Binary(clk,rx,tx,rst,out,LED,nxt,LED1);
  
  input clk;
  input rx;
  input rst;
  input nxt;
  output tx;
  output reg [7:0] out;
  output reg [10:0] LED;
  output reg [10:0] LED1;
  reg [9:0] RAM[15:0];
  
  wire w_TX_Active,w_TX_Serial;
  wire w_RX_dv;
  wire [7:0] w_RX_Byte;
  wire MHz;
  parameter s0 = 3'b000, s1 = 3'b001, s2 = 3'b010, s3 = 3'b011 , s4 = 3'b100, s5 = 3'b101;
  reg fsm;
  reg [7:0] a;
  reg [1:0] state;
  reg [4:0] i;
  reg m = 0;
  reg rw;
  reg [4:0] addr;
  integer j = 0;
  wire nxt1;
  
  debounce(nxt,MHz,nxt1);
  
  clk_div MHz_25(.clk(clk),.clk_out(MHz));

  uart_rx #(.CLKS_PER_BIT(217)) UART_RX_Inst
    (.i_Clock(MHz),
    .i_Rx_Serial(rx),
    .o_Rx_DV(w_RX_dv),
    .o_Rx_Byte(w_RX_Byte)
    );

  uart_tx #(.CLKS_PER_BIT(217)) UART_TX_Inst
    (.i_Clock(MHz),
    .i_Tx_DV(w_RX_dv),
    .i_Tx_Byte(w_RX_Byte),
    .o_Tx_Active(w_TX_Active),
    .o_Tx_Serial(w_TX_Serial),
    .o_Tx_Done()
    );

 task cla4;
    input cin;
    input [3:0] a,b;
    output [3:0] s,cout;
    begin
      cout[0] = (a[0] & b[0]) | ((a[0] ^ b[0]) & cin) ;
      cout[1] = (a[1] & b[1]) | ((a[1] ^ b[1]) & (a[0] & b[0])) | ((a[1] ^ b[1]) & (a[0] ^ b[0]) & cin) ;
      cout[2] = (a[2] & b[2]) | ((a[2] ^ b[2]) & (a[1] & b[1])) | ((a[2] ^ b[2]) & (a[1] ^ b[1]) & (a[0] & b[0])) | ((a[2] ^ b[2]) & (a[1] ^ b[1]) & (a[0] ^ b[0]) & cin) ;
      cout[3] = (a[3] & b[3]) | ((a[3] ^ b[3]) & (a[2] & b[2])) | ((a[3] ^ b[3]) & (a[2] ^ b[2]) & (a[1] & b[1])) | ((a[3] ^ b[3]) & (a[2] ^ b[2]) & (a[1] ^ b[1]) & (a[0] & b[0])) | ((a[3] ^ b[3]) & (a[2] ^ b[2]) & (a[1] ^ b[1]) & (a[0] ^ b[0]) & cin) ;
      s[0] = a[0] ^ b[0] ^ cin;
      s[1] = a[1] ^ b[1] ^ cout[0];
      s[2] = a[2] ^ b[2] ^ cout[1];
      s[3] = a[3] ^ b[3] ^ cout[2];
    end
  endtask
  task cla24;
    input cin;
    input [23:0] x,y;
    output [23:0] sum;
    reg [23:0] cout;
    begin
      cla4(cin, x[3:0],y[3:0],sum[3:0],cout[3:0]);
      cla4(cout[3], x[7:4],y[7:4],sum[7:4],cout[7:4]);
      cla4(cout[7],x[11:8],y[11:8],sum[11:8],cout[11:8]);
      cla4(cout[11],x[15:12],y[15:12],sum[15:12],cout[15:12]);
      cla4(cout[15],x[19:16],y[19:16],sum[19:16],cout[19:16]);
      cla4(cout[19], x[23:20],y[23:20],sum[23:20],cout[23:20]);
    end
  endtask
  task multiply;
    input [9:0] a,b;
    output [119:0] mem ;
    reg [23:0] ans;
    integer i, operate;   
    for( i = 1; i <= 9; i = i + 2 ) begin
      if(i==1) begin
        operate = b[0] - b[1] - b[1]; 
      end   
      else begin 
        operate = b[i-1] + b[i-2] - b[i] - b[i];
      end
      case (operate)
        1: begin
          ans = a;
          ans  = ans<< (i-1);
        end
        2: begin 
          ans = a<<1;
          ans = ans << (i-1);
        end
        -1: begin
          ans = ~a+1;
          ans = ans << (i-1);
        end
        -2: begin
          ans = a<<1;
          ans = ~ans + 1;
          ans = ans << (i-1);
        end
        0: begin
          ans = 0;
        end
      endcase
      if (i==1)
        mem[23:0] = ans;
      else if (i == 3)
        mem[47:24] = ans;
      else if (i==5)
        mem[71:48] = ans;
      else if (i == 7)
        mem[95:72] = ans;
      else 
        mem[119:96] = ans;
    end
  endtask
  task adder;
    input [119:0]mem;
    output [23:0] multiple;
    reg [23:0] s0,s1,s2;
    begin
      cla24 (0,mem[23:0],mem[47:24],s0);
      cla24 (0,mem[71:48],mem[95:72],s1);
      cla24 (0,mem[119:96],s0,s2);
      cla24 (0,s2,s1,multiple);
    end
  endtask
  task adder4;
    input [23:0] a,b,c,d;
    output [23:0] out;
    reg [23:0] s[1:0];
    begin
      cla24 (0,a,b,s[0]);
      cla24 (0,c,d,s[1]);
      cla24 (0,s[0],s[1],out);
    end
  endtask
  task MAC;
    input [9:0] a,b;
    output [23:0] result;
    reg [119:0] ppresult;
    begin
      multiply(a,b,ppresult);
      adder(ppresult,result);
    end
  endtask
task RAM_MUL_Write;
  input rw;
  input [9:0] in;
  input [4:0] addr;
  output [10:0] dout0,dout1,dout2,dout3,dout4,dout5,dout6,dout7,dout8,dout9,dout10,dout11,dout12,dout13,dout14,dout15;
  reg [9:0] RAM[31:0];  //matrix 1 [15:0] //matrix 2 [31:16]
  reg [4:0] addr;
  reg [23:0] intermediate[15:0]; //store output of 16 multiplicatications that takes place at the same time
  reg [23:0] result [15:0];
  input [4:0] i;
  begin
    if(rw == 0)begin
      RAM[addr] = in;
    end
    else if (rw) begin
                                               //if rw == 1 then multiplication will takes place
      MAC(RAM[i]   ,RAM[16] ,intermediate[0]);
      MAC(RAM[i+1] ,RAM[17] ,intermediate[1]);
      MAC(RAM[i+2] ,RAM[18] ,intermediate[2]);
      MAC(RAM[i+3] ,RAM[19] ,intermediate[3]);
      MAC(RAM[i]   ,RAM[20] ,intermediate[4]);
      MAC(RAM[i+1] ,RAM[21] ,intermediate[5]);
      MAC(RAM[i+2] ,RAM[22] ,intermediate[6]);
      MAC(RAM[i+3] ,RAM[23] ,intermediate[7]);
      MAC(RAM[i]   ,RAM[24] ,intermediate[8]);
      MAC(RAM[i+1] ,RAM[25] ,intermediate[9]);
      MAC(RAM[i+2] ,RAM[26],intermediate[10]);
      MAC(RAM[i+3] ,RAM[27],intermediate[11]);
      MAC(RAM[i]   ,RAM[28],intermediate[12]);
      MAC(RAM[i+1] ,RAM[29],intermediate[13]);
      MAC(RAM[i+2] ,RAM[30],intermediate[14]);
      MAC(RAM[i+3] ,RAM[31],intermediate[15]);  

      adder4(intermediate[0],intermediate[1],intermediate[2],intermediate[3],result[i]);
      adder4(intermediate[4],intermediate[5],intermediate[6],intermediate[7],result[i+1]);
      adder4(intermediate[8],intermediate[9],intermediate[10],intermediate[11],result[i+2]);
      adder4(intermediate[12],intermediate[13],intermediate[14],intermediate[15],result[i+3]);    
  
    
    end
	dout0 = result[0][10:0];
   dout1 = result[1][10:0];
	dout2 = result[2][10:0];
	dout3 = result[3][10:0];
	dout4 = result[4][10:0];
	dout5 = result[5][10:0];
	dout6 = result[6][10:0];
	dout7 = result[7][10:0];
	dout8 = result[8][10:0];
	dout9 = result[9][10:0];
	dout10 = result[10][10:0];
	dout11 = result[11][10:0];
	dout12 = result[12][10:0];
	dout13 = result[13][10:0];
	dout14 = result[14][10:0];
	dout15 = result[15][10:0];
  end
endtask

task ascii2binary;
  input [7:0] w_RX_byte;
  output [7:0]out;  
  begin
    case (w_RX_byte)
    48: out = 0;
    49: out = 1;
    50: out = 2;
    51: out = 3;
    52: out = 4;
    53: out = 5; 
    54: out = 6;
    55: out = 7;
    56: out = 8;
    57: out = 9;
    endcase
  end
endtask
  
always @ (posedge MHz)begin

  if (!rst)begin
    state = s0; out = 0; a =0; addr = 0;
  end
  else begin
      case (state)
        s0: begin
          fsm =0; 
          if (w_RX_dv) begin
            state = s1;
            ascii2binary(w_RX_Byte,a);
            out = out +  a * 100;
            $display ($realtime, "ns %d", out);
          end
          else begin
            state = s0;
          end  
        end
        s1: begin
          fsm =0; 
          if (w_RX_dv) begin
            state = s2;
            ascii2binary(w_RX_Byte,a);
            out = out + a * 10;
            $display ($realtime, "ns %d", out);
          end
          else begin
            state = s1;
          end
        end
        s2: begin
          fsm =0; 
          if (w_RX_dv) begin
            state = s3;
            ascii2binary(w_RX_Byte,a);
            out = out + a * 1;
            $display ($realtime, "ns %d", out);
          end
          else begin
            state = s2;
          end
        end
        s3: begin
          fsm =1;
          if (w_RX_dv == 0) begin
				//RAM_MUL_Write(0,out,addr,RAM,i);
            RAM_MUL_Write(0,out,addr,RAM[0],RAM[1],RAM[2],RAM[3],RAM[4],RAM[5],RAM[6],RAM[7],RAM[8],RAM[9],RAM[10],RAM[11],RAM[12],RAM[13],
			RAM[14],RAM[15],i);
            addr = addr + 1;
            out = 0;
            state = s0;
            if(addr == 16) begin
					m = 1;
					i = 0;
				end
            $display ($realtime, "ns %d", out);
          end 
          else begin
            state = s3;
          end 
        end
      endcase
		if(m) begin
			//RAM_MUL_Write(1,out,addr,RAM,i);
			RAM_MUL_Write(1,out,addr,RAM[0],RAM[1],RAM[2],RAM[3],RAM[4],RAM[5],RAM[6],RAM[7],RAM[8],RAM[9],RAM[10],RAM[11],RAM[12],RAM[13],
			RAM[14],RAM[15],i);
			i = i+4;
		end
  end
end

	always @ (negedge nxt1 or negedge rst) begin
		if(!rst) begin
			LED = 0;
			LED1 = 0;
			j = 0;
		end
		else begin
			LED = RAM[j];
			LED1 = RAM[j];
			j = j+1;
			if(j == 16)
				j = 0;
		end
	end


  assign tx = w_TX_Active ? w_TX_Serial:1'b1;
  
  
endmodule



module uart_rx 
  #(parameter CLKS_PER_BIT = 434)
  (
   input        i_Clock,
   input        i_Rx_Serial,
   output       o_Rx_DV,
   output [7:0] o_Rx_Byte
   );
    
  parameter s_IDLE         = 3'b000;
  parameter s_RX_START_BIT = 3'b001;
  parameter s_RX_DATA_BITS = 3'b010;
  parameter s_RX_STOP_BIT  = 3'b011;
  parameter s_CLEANUP      = 3'b100;
   
  reg           r_Rx_Data_R = 1'b1;
  reg           r_Rx_Data   = 1'b1;
   
  reg [7:0]     r_Clock_Count = 0;
  reg [2:0]     r_Bit_Index   = 0; //8 bits total
  reg [7:0]     r_Rx_Byte     = 0;
  reg           r_Rx_DV       = 0;
  reg [2:0]     r_SM_Main     = 0;
   
  // Purpose: Double-register the incoming data.
  // This allows it to be used in the UART RX Clock Domain.
  // (It removes problems caused by metastability)
  always @(posedge i_Clock)
    begin
      r_Rx_Data_R <= i_Rx_Serial;
      r_Rx_Data   <= r_Rx_Data_R;
    end
   
   
  // Purpose: Control RX state machine
  always @(posedge i_Clock)
    begin
       
      case (r_SM_Main)
        s_IDLE :
          begin
            r_Rx_DV       <= 1'b0;
            r_Clock_Count <= 0;
            r_Bit_Index   <= 0;
             
            if (r_Rx_Data == 1'b0)          // Start bit detected
              r_SM_Main <= s_RX_START_BIT;
            else
              r_SM_Main <= s_IDLE;
          end
         
        // Check middle of start bit to make sure it's still low
        s_RX_START_BIT :
          begin
            if (r_Clock_Count == (CLKS_PER_BIT-1)/2)
              begin
                if (r_Rx_Data == 1'b0)
                  begin
                    r_Clock_Count <= 0;  // reset counter, found the middle
                    r_SM_Main     <= s_RX_DATA_BITS;
                  end
                else
                  r_SM_Main <= s_IDLE;
              end
            else
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_START_BIT;
              end
          end // case: s_RX_START_BIT
         
         
        // Wait CLKS_PER_BIT-1 clock cycles to sample serial data
        s_RX_DATA_BITS :
          begin
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_DATA_BITS;
              end
            else
              begin
                r_Clock_Count          <= 0;
                r_Rx_Byte[r_Bit_Index] <= r_Rx_Data;
                 
                // Check if we have received all bits
                if (r_Bit_Index < 7)
                  begin
                    r_Bit_Index <= r_Bit_Index + 1;
                    r_SM_Main   <= s_RX_DATA_BITS;
                  end
                else
                  begin
                    r_Bit_Index <= 0;
                    r_SM_Main   <= s_RX_STOP_BIT;
                  end
              end
          end // case: s_RX_DATA_BITS
     
     
        // Receive Stop bit.  Stop bit = 1
        s_RX_STOP_BIT :
          begin
            // Wait CLKS_PER_BIT-1 clock cycles for Stop bit to finish
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_RX_STOP_BIT;
              end
            else
              begin
                r_Rx_DV       <= 1'b1;
                r_Clock_Count <= 0;
                r_SM_Main     <= s_CLEANUP;
              end
          end // case: s_RX_STOP_BIT
     
         
        // Stay here 1 clock
        s_CLEANUP :
          begin
            r_SM_Main <= s_IDLE;
            r_Rx_DV   <= 1'b0;
          end
         
         
        default :
          r_SM_Main <= s_IDLE;
         
      endcase
    end   
   
  assign o_Rx_DV   = r_Rx_DV;
  assign o_Rx_Byte = r_Rx_Byte;
   
endmodule // uart_rx

module uart_tx 
  #(parameter CLKS_PER_BIT = 434)
  (
   input       i_Clock,
   input       i_Tx_DV,
   input [7:0] i_Tx_Byte, 
   output      o_Tx_Active,
   output reg  o_Tx_Serial,
   output      o_Tx_Done
   );
  
  parameter s_IDLE         = 3'b000;
  parameter s_TX_START_BIT = 3'b001;
  parameter s_TX_DATA_BITS = 3'b010;
  parameter s_TX_STOP_BIT  = 3'b011;
  parameter s_CLEANUP      = 3'b100;
   
  reg [2:0]    r_SM_Main     = 0;
  reg [7:0]    r_Clock_Count = 0;
  reg [2:0]    r_Bit_Index   = 0;
  reg [7:0]    r_Tx_Data     = 0;
  reg          r_Tx_Done     = 0;
  reg          r_Tx_Active   = 0;
     
  always @(posedge i_Clock)
    begin
       
      case (r_SM_Main)
        s_IDLE :
          begin
            o_Tx_Serial   <= 1'b1;         // Drive Line High for Idle
            r_Tx_Done     <= 1'b0;
            r_Clock_Count <= 0;
            r_Bit_Index   <= 0;
             
            if (i_Tx_DV == 1'b1)
              begin
                r_Tx_Active <= 1'b1;
                r_Tx_Data   <= i_Tx_Byte;
                r_SM_Main   <= s_TX_START_BIT;
              end
            else
              r_SM_Main <= s_IDLE;
          end // case: s_IDLE
         
         
        // Send out Start Bit. Start bit = 0
        s_TX_START_BIT :
          begin
            o_Tx_Serial <= 1'b0;
             
            // Wait CLKS_PER_BIT-1 clock cycles for start bit to finish
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_TX_START_BIT;
              end
            else
              begin
                r_Clock_Count <= 0;
                r_SM_Main     <= s_TX_DATA_BITS;
              end
          end // case: s_TX_START_BIT
         
         
        // Wait CLKS_PER_BIT-1 clock cycles for data bits to finish         
        s_TX_DATA_BITS :
          begin
            o_Tx_Serial <= r_Tx_Data[r_Bit_Index];
             
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_TX_DATA_BITS;
              end
            else
              begin
                r_Clock_Count <= 0;
                 
                // Check if we have sent out all bits
                if (r_Bit_Index < 7)
                  begin
                    r_Bit_Index <= r_Bit_Index + 1;
                    r_SM_Main   <= s_TX_DATA_BITS;
                  end
                else
                  begin
                    r_Bit_Index <= 0;
                    r_SM_Main   <= s_TX_STOP_BIT;
                  end
              end
          end // case: s_TX_DATA_BITS
         
         
        // Send out Stop bit.  Stop bit = 1
        s_TX_STOP_BIT :
          begin
            o_Tx_Serial <= 1'b1;
             
            // Wait CLKS_PER_BIT-1 clock cycles for Stop bit to finish
            if (r_Clock_Count < CLKS_PER_BIT-1)
              begin
                r_Clock_Count <= r_Clock_Count + 1;
                r_SM_Main     <= s_TX_STOP_BIT;
              end
            else
              begin
                r_Tx_Done     <= 1'b1;
                r_Clock_Count <= 0;
                r_SM_Main     <= s_CLEANUP;
                r_Tx_Active   <= 1'b0;
              end
          end // case: s_Tx_STOP_BIT
         
         
        // Stay here 1 clock
        s_CLEANUP :
          begin
            r_Tx_Done <= 1'b1;
            r_SM_Main <= s_IDLE;
          end
         
         
        default :
          r_SM_Main <= s_IDLE;
         
      endcase
    end
 
  assign o_Tx_Active = r_Tx_Active;
  assign o_Tx_Done   = r_Tx_Done;
   
endmodule

module clk_div (clk, clk_out);
 
input clk;
output reg clk_out = 0;
 
reg [1:0] r_reg = 2'b00;
wire [1:0] r_nxt;
//reg clk_track = 0;
 
always @(posedge clk )
 
begin
 
  if (r_nxt == 2'b10)
 	   begin
	     r_reg <= 0;
	     clk_out <= ~clk_out;
	   end
 
  else 
      r_reg <= r_nxt;
end
 
 assign r_nxt = r_reg+1;   	      
endmodule

module debounce(input pb_1,clk,output pb_out);
wire slow_clk;
wire Q1,Q2,Q2_bar;
clock_div u1(clk,slow_clk);
my_dff d1(slow_clk, pb_1,Q1 );
my_dff d2(slow_clk, Q1,Q2 );
assign Q2_bar = ~Q2;
assign pb_out = Q1 & Q2_bar;
endmodule
// Slow clock for debouncing 
module clock_div(input Clk_100M, output reg slow_clk

    );
    reg [26:0]counter=0;
    always @(posedge Clk_100M)
    begin
        counter <= (counter>=249999)?0:counter+1;
        slow_clk <= (counter < 125000)?1'b0:1'b1;
    end
endmodule
// D-flip-flop for debouncing module 
module my_dff(input DFF_CLOCK, D, output reg Q);

    always @ (posedge DFF_CLOCK) begin
        Q <= D;
    end

endmodule
