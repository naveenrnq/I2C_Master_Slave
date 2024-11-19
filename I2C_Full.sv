// We will be considering two i2c masters where we would be either pulling the line low or releasing the line
module i2c_master (input clk, rst, newd, // newd is raised but user to indicate new data
                   input [6:0] addr,
                   input op, // 1 for read 0 is for write
                   inout sda, // data can bidirectional
                   output scl, // cloxk is always provided by master
                   input [7:0] din,
                   output [7:0] dout,
                   output reg busy, ack_err, // Whenevr we get NACK from Slave
                   output reg done
                  );

  reg scl_t = 0; // Store temprory values before sending to SCL and SDA
  reg sda_t = 0;

  parameter sys_freq = 40000000; // Onboard clk 40 Mhz
  parameter i2c_freq = 100000; // Freq of I2C Peripheral for communicating data 100 Khz - 400 Khz

  parameter clk_count4 = (sys_freq/i2c_freq)*4; // ratio of sys clk and i2c clk for single bit duration 400 clock pulses
  parameter clk_count1 = clk_count4/4; // Divide one clk duration by 4 (100)

 
  integer count1 = 0;
  reg i2c_clk = 0;


  // Our entire bit duration have 400 clock pulses
  // 0 - 100 one clock pulse
  // 100 - 200 second clock pulse
  // 200 - 300 third clock pulse
  // 300 - 400 fourth clock pulse
  reg [1:0] pulse = 0; // Each bit duration is divided into 4 smaller parts

  always@(posedge clk)
  begin
    if(rst)
      begin 
        pulse <= 0;
        count1 <= 0;
      end

    else if(busy == 1'b0) // pulse count starts only after new data
                          // pulse counter should ot be incrementing until we have new data
      begin
        pulse <= 0;
        count1 <= 0;
      end
    else if(count1 == clk_count1 - 1) // 0 to 99 clock pulse
      begin
        pulse <= 1;
        count1 <= count1 + 1;
      end  
    else if(count1 == clk_count1*2 - 1)
      begin 
        pulse <= 2;
        count1 <= count1 + 1;
      end
    else if(count1 == clk_count1*3 - 1)
      begin
        pulse <= 3;
        count1 <= count1 + 1;
      end
    else if(count1 == clk_count1*4 - 1)
      begin
        pulse <= 0;
        count1 <= 0;
      end
    else
    begin
      count1 <= count1 + 1; // if no condition increment the counter
    end
  end


///////////////////////////////////////////////////////////////

  reg [3:0] bitcount = 0;
  reg [7:0] data_addr = 0, data_tx = 0;
  reg r_ack = 0;
  reg [7:0] rx_data = 0;
  reg sda_en = 0;

  typedef enum logic [3:0] {idle = 0, // Idle state
  start = 1, // Start bit I2C
  write_addr = 2, // State for writing an addr
  ack_1 = 3, // Wait for acknowledgement from slave
  write_data = 4, // Write the data to the slave
  read_data = 5, // Or read the data according to the operation
  stop = 6, // After receiving all acknowledgement stop the operation
  ack_2 = 7, // If we write the data we need secnd acknowledgement from the slave
  master_ack = 8 // If we read the data we need master acknowledgement to the slave
                           } state_type;
 
  state_type state = idle;

  always@(posedge clk)
  begin
    if(rst)
    begin
      bitcount <= 0;
      data_addr <= 0;
      data_tx <= 0;
      scl_t <= 1;
      sda_t <= 1;
      state <= idle;
      busy <= 1'b0;
      ack_err <= 1'b0;
      done <= 1'b0;
    end
    else
    begin
      case(state)
        // IDLE
        idle: 
        begin
          done <= 1'b0;
          if(newd == 1'b1)  // Whenever we have new data 1st thing is to store data and addr to the temprory variable
          begin
            data_addr <= {addr,op};  // MSB of the addr then addr then type of opration (on assignment completely opp)
            data_tx <= din;
            busy <= 1'b1;
            state <= start;
            ack_err <= 1'b0;
          end
          else
          begin
            data_addr <= 0;
            data_tx <= 0;
            busy <= 1'b0;
            state <= idle;
            ack_err <= 1'b0;
          end
        end

        // START: We know for start condition sda should go 1->0 in single bit duration
        start: 
        begin
          case(pulse)
            0: 
            begin 
               scl_t <= 1'b1; 
               sda_t <= 1'b1; 
            end
            1: 
            begin 
               scl_t <= 1'b1; 
               sda_t <= 1'b1; 
            end
            2: 
            begin 
               scl_t <= 1'b1; 
               sda_t <= 1'b0; 
            end
            3: 
            begin 
               scl_t <= 1'b1; 
               sda_t <= 1'b0; 
            end
          endcase
          
          // When count reaches 399 i.e. 400 clock ticks we switches to write data
          if(count1  == clk_count1*4 - 1)
          begin
            state <= write_addr;
            scl_t <= 1'b0;
          end
          else
            state <= start;
        end
        // Write Data
        write_addr: 
        begin
          sda_en <= 1'b1;  ///send addr to slave
          if(bitcount <= 7) // Wait till all 8 bits are sent
          begin
            case(pulse) // dividng clk into 4 pulses allow us to take two pulse as positive clk and two pulse as negative clk just to simulate or make
              0: 
              begin 
                scl_t <= 1'b0; 
                sda_t <= 1'b0; 
              end
              1: // We choose this pulse to change the data i.e. -ve pulse of clock
                 // We cannot change data when clock is high to maintain data validity
              begin 
                scl_t <= 1'b0; 
                sda_t <= data_addr[7 - bitcount]; 
              end
              2: // once scl is high we cant send data
              begin 
                scl_t <= 1'b1;  
              end
              3: 
              begin 
                scl_t <= 1'b1;  
              end
            endcase

            if(count1  == clk_count1*4 - 1) // End our bit duration
            begin
              state <= write_addr;
              scl_t <= 1'b0;
              bitcount <= bitcount + 1;
            end
            else
            begin
              state <= write_addr;
            end                  
          end
          else
          begin
            state <= ack_1; // once data is sent
            bitcount <= 0;
            sda_en <= 1'b0;  // release the sda line after data transfer
          end
        end   
        
        // Acknowledgement
        ack_1 : 
        begin
          sda_en <= 1'b0; ///recv ack from slave

          case(pulse)
          0: 
          begin 
            scl_t <= 1'b0; 
            sda_t <= 1'b0; 
          end
          1: 
          begin 
            scl_t <= 1'b0; 
            sda_t <= 1'b0; 
          end
          2: 
          begin 
            scl_t <= 1'b1;   // Sample data at positive edge of the clock i.e.e receive
            sda_t <= 1'b0; 
            r_ack <= sda; 
          end ///recv ack from slave
          3: 
          begin 
            scl_t <= 1'b1;  
          end
          endcase
                   
          if(count1  == clk_count1*4 - 1)
          begin
            if(r_ack == 1'b0 && data_addr[0] == 1'b0)  // If Ack signal goes low then do the read or write operation
            begin
              state <= write_data;
              sda_t <= 1'b0;
              sda_en <= 1'b1; /////write data to slave
              bitcount <= 0;
            end
            else if (r_ack == 1'b0 && data_addr[0] == 1'b1)
            begin
              state <= read_data;
              sda_t <= 1'b1;
              sda_en <= 1'b0; ///read data from slave
              bitcount <= 0;
            end
            else
            begin
              state <= stop;
              sda_en <= 1'b1; ////send stop to slave
              ack_err <= 1'b1;  // If Ack doesnt goes low throw this error and stop communication
            end
          end
          else
          begin
            state <= ack_1;
          end           
        end

        // Write DATA
        write_data: // Write whatever data into the SDA line
        begin
          if(bitcount <= 7) // Wait till all the data is sent
          begin
            case(pulse)
            0: 
            begin 
              scl_t <= 1'b0;   
            end
            1: 
            begin 
              scl_t <= 1'b0;
              sda_en <= 1'b1; 
              sda_t <= data_tx[7 - bitcount];                                  
            end
            2: 
            begin 
              scl_t <= 1'b1;  
            end
            3: 
            begin 
              scl_t <= 1'b1;  
            end
            endcase
                                 
            if(count1  == clk_count1*4 - 1)
            begin
              state <= write_data;
              scl_t <= 1'b0;
              bitcount <= bitcount + 1;
            end
            else
            begin
              state <= write_data;
            end              
          end
          else
          begin
            state <= ack_2;
            bitcount <= 0;
            sda_en <= 1'b0; 
          end          
        end 

        ///////////////////////////// read_data
        read_data: 
        begin
          sda_en <= 1'b0; ///read from slave sda line is in hgh impedance state
          if(bitcount <= 7)
          begin
            case(pulse)
            0: 
            begin 
              scl_t <= 1'b0; 
              sda_t <= 1'b0; 
            end
            1: 
            begin 
              scl_t <= 1'b0; 
              sda_t <= 1'b0; 
            end
            2: 
            begin 
              scl_t <= 1'b1; 
              rx_data[7:0] <= (count1 == 200) ? {rx_data[6:0],sda} : rx_data;  // sample data in rxdata
// Add data then shift the data to the left
            end
            3: 
            begin 
              scl_t <= 1'b1;  
            end
            endcase

            if(count1  == clk_count1*4 - 1)
            begin
              state <= read_data;
              scl_t <= 1'b0;
              bitcount <= bitcount + 1;
            end
            else
            begin
              state <= read_data;
            end     
          end
          else
          begin
            state <= master_ack;
            bitcount <= 0;
            sda_en <= 1'b1; ///master will send ack to slave
          end         
        end

        ////////////////////master ack -> send nack
        master_ack : 
        begin
          sda_en <= 1'b1;  
         
          case(pulse)
          0: 
          begin 
            scl_t <= 1'b0; // When SDA is always high it serves as negative acknowledgement to the slaves
            sda_t <= 1'b1; 
          end
          1: 
          begin 
            scl_t <= 1'b0; 
            sda_t <= 1'b1; 
          end
          2: 
          begin 
            scl_t <= 1'b1; 
            sda_t <= 1'b1; 
          end 
          3: 
          begin 
            scl_t <= 1'b1; 
            sda_t <= 1'b1; 
          end
          endcase
                   
          if(count1  == clk_count1*4 - 1)
          begin
            sda_t <= 1'b0;
            state <= stop;
            sda_en <= 1'b1; ///send stop to slave                       
          end
          else
          begin
            state <= master_ack;
          end    
        end
                              
        /////////////////ack 2
                 
        ack_2 : 
        begin
          sda_en <= 1'b0; ///recv ack from slave
          case(pulse)
          0: 
          begin 
            scl_t <= 1'b0; 
            sda_t <= 1'b0; 
          end
          1: 
          begin 
            scl_t <= 1'b0; 
            sda_t <= 1'b0; 
          end
          2: 
          begin 
            scl_t <= 1'b1; 
            sda_t <= 1'b0; 
            r_ack <= sda; 
          end ///recv ack from slave
          3: 
          begin 
            scl_t <= 1'b1;  
          end
          endcase
                   
          if(count1  == clk_count1*4 - 1)
          begin
            sda_t <= 1'b0;
            sda_en <= 1'b1; ///send stop to slave

            if(r_ack == 1'b0 )
            begin
              state <= stop;
              ack_err <= 1'b0;
            end
            else
            begin
              state <= stop;
              ack_err <= 1'b1;
            end
          end
          else
          begin
            state <= ack_2;
          end         
        end
 
        // stop  
        stop: 
        begin
          sda_en <= 1'b1; ///send stop to slave
          case(pulse)
          0: 
          begin 
            scl_t <= 1'b1; 
            sda_t <= 1'b0; 
          end
          1: 
          begin 
            scl_t <= 1'b1; 
            sda_t <= 1'b0; // In stop transition the sda goes from 0 to 1
          end
          2: 
          begin 
            scl_t <= 1'b1; 
            sda_t <= 1'b1; 
          end
          3: 
          begin 
            scl_t <= 1'b1; 
            sda_t <= 1'b1; 
          end
          endcase
                         
          if(count1  == clk_count1*4 - 1)
          begin
            state <= idle;
            scl_t <= 1'b0;
            busy <= 1'b0;
            sda_en <= 1'b1; ///send start to slave
            done   <= 1'b1;
          end
          else
            state <= stop;

        end
       
        default : 
          state <= idle;
      endcase
   end
end
 
assign sda = (sda_en == 1) ? (sda_t == 0) ? 1'b0 : 1'b1 : 1'bz; /// en = 1 -> write to slave else read
////// if sda_en == 1 then if sda_t == 0 pull line low else release so that pull up make line high
/*
if(sda_en)
   if(!sda_t)
      sda = 0
   else
      sda = z
else
   sda = z
*/
assign scl = scl_t;
assign dout = rx_data;
endmodule







///////////////////// Slave
// Exactly opposite to the master

module i2c_Slave(
input scl,clk,rst,
inout sda,
output reg ack_err, done
    );
  
typedef enum logic [3:0] {idle = 0, read_addr = 1, send_ack1 = 2, send_data = 3, master_ack = 4, read_data = 5, send_ack2 = 6, wait_p = 7, detect_stop = 8} state_type;
state_type state = idle;    
 
  reg [7:0] mem [128]; // Master can have maximum 7 bits of data
                       // Here we have initialized the memory such that it can have 
                       // 128 memory locations with 8 bit data in it
  reg [7:0] r_addr; 
  reg [6:0] addr; 
  reg r_mem = 0; 
  reg w_mem = 0; 
  reg [7:0] dout;
  reg [7:0] din;
  reg sda_t;
  reg sda_en;
  reg [3:0] bitcnt = 0;
 
 
 
 
///////////// initialize mem
always@(posedge clk)
begin
  if(rst)
  begin
    for(int i = 0 ; i < 128; i++) 
      begin
        mem[i] = i; // if reset initialize the memory with random value
      end 
    dout <= 8'h0;  // Temp variable that slave will send to the master if master initiates re-transaction
  end
  else if (r_mem == 1'b1)  // Read Memory
   begin
      dout <= mem[addr];
   end
  else if (w_mem == 1'b1)  /// Write Memory
   begin
      mem[addr] <= din;
   end 
   
end
 
/////////////////////////pulse_gen logic
parameter sys_freq = 40000000;
parameter i2c_freq = 100000;
 
 
parameter clk_count4 = (sys_freq/i2c_freq);
parameter clk_count1 = clk_count4/4;
integer count1 = 0;
reg i2c_clk = 0;
 
///////4x clock
reg [1:0] pulse = 0;
reg busy;
always@(posedge clk)
begin
      if(rst)
      begin
        pulse <= 0;
        count1 <= 0;
      end
      else if(busy == 1'b0)
       begin
       pulse <= 2;   // This value is for aligning everything with the Master
       count1 <= 202; // For exeact count value and pulse value for master and slave
       end
      else if(count1  == clk_count1 - 1)
       begin
       pulse <= 1;
       count1 <= count1 + 1;
       end
      else if(count1  == clk_count1*2 - 1)
       begin
       pulse <= 2;
       count1 <= count1 + 1;
       end
      else if(count1  == clk_count1*3 - 1)
       begin
       pulse <= 3;
       count1 <= count1 + 1;
       end
      else if(count1  == clk_count1*4 - 1)
       begin
       pulse <= 0;
       count1 <= 0;
       end
      else
       begin
       count1 <= count1 + 1;
       end
end
 
 
 
 
 
reg scl_t;
wire start;
always@(posedge clk)
begin
scl_t <= scl;
end
 
assign start = ~scl & scl_t; 
 
reg r_ack;
 
always@(posedge clk)
begin
if(rst)
 begin
                  bitcnt <= 0;
                  state  <= idle;
                  r_addr <= 7'b0000000;
                  sda_en <= 1'b0;
                  sda_t <= 1'b0;
                  addr  <= 0;
                  r_mem <= 0;
                  din   <= 8'h00; 
                  ack_err <= 0;
                  done    <= 1'b0;
                  busy <= 1'b0;
 end
 
 else
    begin
      case(state)
               idle: begin
                   if(scl == 1'b1 && sda == 1'b0)
                    begin
                    busy <= 1'b1;
                    state <= wait_p; 
                    end
                   else
                    begin
                    state <= idle;
                    end
               end
               //////////////////////
               wait_p :  
               begin
                if (pulse == 2'b11 && count1 == 399)
                    state <= read_addr;  // Perfectly aligned with master write address
                else
                    state <= wait_p;
               
               end
               /////////////////////////////////////
               read_addr: 
               begin
                 sda_en <= 1'b0;  ///read addr to slave
                   if(bitcnt <= 7)
                     begin
                       case(pulse)
                         0: 
                         begin  
                         end
                         1: 
                         begin  
                         end
                         2: 
                         begin   
                           r_addr <= (count1 == 200) ? {r_addr[6:0],sda} : r_addr; // Read at 3rd pulse of the bit duration exactly when scl goes high
                         end 
                         3: 
                         begin  
                         end
                       endcase

                       if(count1  == clk_count1*4 - 1)
                         begin
                           state <= read_addr;
                           bitcnt <= bitcnt + 1;
                         end
                       else
                         begin
                           state <= read_addr;
                         end
                                     
                     end
                     else
                       begin
                         state  <= send_ack1;
                         bitcnt <= 0;
                         sda_en <= 1'b1;
                         addr <= r_addr[7:1];
                       end
                       
                      end
                      /////////////////////////// send ack
                      
                      send_ack1: 
                      begin
                        case(pulse) 
                          0: begin  sda_t <= 1'b0; end 
                          1: begin  end
                          2: begin  end 
                          3: begin  end
                        endcase   
                                  if(count1  == clk_count1*4 - 1)
                                       begin
                                         if(r_addr[0] == 1'b1) // Define type of operation based on LSB bit i.e. read
                                            begin
                                            state <= send_data; // In this case slave will be reading data from memory and storing this to temprory variable for sending to the master
                                            r_mem <= 1'b1;
                                            end
                                          else
                                            begin
                                             state <= read_data; // Here else indicate the write operation
                                             r_mem <= 1'b0; // Here read memory is 0 and write memory is 1 where we have made it in send ack state
                                              // At the end of reception of data we will make write memory 1
                                            end
                                         end
                                  else
                                       begin
                                       state <= send_ack1;
                                       end
                                    
                                    
                      end
                      
                      
                      ///////////////////////read data
                      
                       read_data: 
                        begin
                          sda_en <= 1'b0;  ///read addr to slave
                                           // 0 so that master could add data to this line
                              if(bitcnt <= 7)
                                 begin
                                         case(pulse)
                                         0: begin  end
                                         1: begin  end
                                         2: begin   din <= (count1 == 200) ? {din[6:0],sda} : din; end // Add data and do left shift so that all data can be easilu assigned to din
                                         3: begin  end
                                         endcase
                                         if(count1  == clk_count1*4 - 1)
                                         begin
                                            state <= read_data;
                                            bitcnt <= bitcnt + 1;
                                         end
                                         else
                                         begin
                                            state <= read_data;
                                         end
                                     
                                 end
                              else
                                begin
                                state  <= send_ack2;
                                bitcnt <= 0;
                                sda_en <= 1'b1;
                                w_mem  <= 1'b1;
                                end
                       
                      end
                      /////////////////////////////////////////////
                      send_ack2: 
                      begin
                                   
                                   case(pulse)
                                         0: begin  sda_t <= 1'b0; end
                                         1: begin  w_mem <= 1'b0; end
                                         2: begin  end 
                                         3: begin  end
                                     endcase
                                  if(count1  == clk_count1*4 - 1)
                                         begin
                                          state <= detect_stop;
                                          sda_en <= 1'b0;
                                         end
                                  else
                                       begin
                                       state <= send_ack2;
                                       end
                      end
                 /////////////
                 send_data : begin
                     sda_en <= 1'b1;  ///read addr to slave
                              if(bitcnt <= 7)
                                 begin
                                         r_mem  <= 1'b0;
                                         case(pulse)
                                         0: begin    end
                                         1: begin sda_t <= (count1 == 100) ? dout[7 - bitcnt] : sda_t; end
                                         2: begin    end 
                                         3: begin    end
                                         endcase
                                         if(count1  == clk_count1*4 - 1)
                                         begin
                                            state <= send_data;
                                            bitcnt <= bitcnt + 1;
                                         end
                                         else
                                         begin
                                            state <= send_data;
                                         end
                                     
                                 end
                              else
                                begin
                                state  <= master_ack;
                                bitcnt <= 0;
                                sda_en <= 1'b0;
                                end
                     end  
                   //////////////////////////
                   master_ack: 
                   begin
                                   case(pulse)
                                         0: begin  end
                                         1: begin  end
                                         2: begin r_ack <= (count1 == 200) ? sda : r_ack; end 
                                         3: begin  end
                                     endcase
                                  if(count1  == clk_count1*4 - 1)
                                         begin
                                               if(r_ack == 1'b1) ///nack
                                                   begin
                                                   ack_err <= 1'b0;
                                                   state <= detect_stop;
                                                   sda_en <= 1'b0;
                                                   end
                                               else
                                                    begin
                                                    ack_err <= 1'b1;
                                                    state   <= detect_stop;
                                                    sda_en <= 1'b0;
                                                    end
                                         end
                                  else
                                       begin
                                       state <= master_ack;
                                       end
                      end
                   /////////////////////////////////////////
                  
                   detect_stop: 
                   begin
                       if(pulse == 2'b11 && count1 == 399)
                           begin
                           state <= idle;
                           busy <= 1'b0;
                           done <= 1'b1;
                          end
                         else
                           state <= detect_stop;
                        
                   end
                   
                 
             
      
      default: state <= idle;
      
      endcase
    end
end
 
assign sda = (sda_en == 1'b1) ? sda_t : 1'bz;
 
endmodule




////////////////top
 
`timescale 1ns / 1ps
 
module i2c_top(
input clk, rst, newd, op,
input [6:0] addr,
input [7:0] din,
output [7:0] dout,
output busy,ack_err,
output done
);
wire sda, scl;
wire ack_errm, ack_errs;
 
 
i2c_master master (clk, rst, newd, addr, op, sda, scl, din, dout, busy, ack_errm , done);
i2c_Slave slave (scl, clk, rst, sda, ack_errs, );
 
assign ack_err = ack_errs | ack_errm;
 
 
endmodule
 
///////////////////////////////////////////////////////
interface i2c_if;
  
  logic clk;
  logic rst;
  logic newd;
  logic op;   
  logic [7:0] din;
  logic [6:0] addr;
  logic [7:0] dout;
  logic  done;
  logic busy, ack_err;  
  
endinterface
