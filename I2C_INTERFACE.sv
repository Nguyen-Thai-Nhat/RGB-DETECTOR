/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                         //
//                                                                                                         //
//    -+++==-          -*****+                                                                             //
//     *%###-          .@@@@@-                                                                             //
//      +*++=.         *@@@@-                                                                              //
//       ====-        +%%%%=                                                                               //
//       .====-      -####=  :++++++++++++   -++++++++=-.         :=+****+-.      .+++:       =++=         //
//        .=+++=    :+++*-   -%%%%%%%%%%%#   +%%%%%%%%%%%#-     =#%%%%##%%%%*:    :%%%%=      #%%#         //
//         :***#=  .====-                    =%%%:   .:*%%%-  .#%%%+:   .-*%%%+   :%%%%%*:    *%%#         //
//          -%###=.===-:                     =%%%.     .%%%*  *%%%:        +%%%-  :%%%%%%%=   *%%#         //
//           =%%%#**++-      -%%%%%%%%%%%#   =%%%.     =%%%= .%%%+          #%%*  :%%%==%%%*. *%%#         //
//            =@%%###+       :++++++++++++   =%%%*****#%%%+  .%%%+         .%%%*  :%%%- :#%%%=#%%#         //
//             +@%%%#.                       =%%%####%%%*.    *%%%-        *%%%:  :%%%=   =%%%%%%#         //
//              *@@%:         ............   =%%%:   =%%%=     *%%%*-:..:=#%%%-   :%%%=    :#%%%%#         //
//               #@:         =@%%%%%%%%%%%   +%%%.    :#%@*.    -*%%%%%%%%%#+.    :%%@=      +%%%#         //
//               .:          .------------   :---      .--=:      .:-=+==-:        ---.       :---         //
//                                                                                                         //
//     :- :: -: : :: -.. :- ::   : -... - .-...     .. -. - ::.. -: :.  . -: -: :: : :- : : :: -:          //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Company: Veron Group                                                                                    //
// Author:  RTL DN TEAM                                                                                    //
//                                                                                                         //
// Create Date:    26/09/2025                                                                              //
// Module Name:    I2C_INTERFACE                                                                           //
// Revision History:                                                                                       //
// Revision 0.01 - File Created (Author)                                                                   //
// Revision 0.02 - modified to work with Cyclone10 board                                                   //
// Property of Veron Group                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
module I2C_INTERFACE#(
    parameter integer unsigned NUMBER_OF_DATA_BYTES         = 1,
    parameter integer unsigned NUMBER_OF_REGISTER_BYTES     = 1,
    parameter integer unsigned ADDRESS_WIDTH                = 7,
    parameter integer unsigned CHECK_FOR_CLOCK_STRETCHING   = 1,    //set to non zero value to enable
    parameter integer unsigned CLOCK_STRETCHING_MAX_COUNT   = 'hFF  //set to 0 to disable, max number of divider ticks to wait during stretch check
)(
    input   wire                                        clock,
    input   wire                                        reset_n,
    input   wire                                        pll_clk_300k,   // ADD: 300 kHz clock from PLL
    input   wire                                        pll_locked,     // ADD (optional)
    input   wire                                        enable,
    input   wire                                        read_write,
    input   wire    [(NUMBER_OF_DATA_BYTES*8)-1:0]      mosi_data,
    input   wire    [(NUMBER_OF_REGISTER_BYTES*8)-1:0]  register_address,
    input   wire    [ADDRESS_WIDTH-1:0]                 device_address,


    output  reg     [(NUMBER_OF_DATA_BYTES*8)-1:0]      miso_data,
    output  logic                                       busy,

    inout                                               sda,
    inout                                               scl
);

localparam DATA_WIDTH                   = (NUMBER_OF_DATA_BYTES         != 0)   ? (NUMBER_OF_DATA_BYTES     * 8) : 8;
localparam REGISTER_WIDTH               = (NUMBER_OF_REGISTER_BYTES     != 0)   ? (NUMBER_OF_REGISTER_BYTES * 8) : 8;
localparam MAX_NUMBER_BYTES             = (DATA_WIDTH > REGISTER_WIDTH)         ? (DATA_WIDTH/8) : (REGISTER_WIDTH/8);
localparam CLOCK_STRETCHING_TIMER_WIDTH = (CLOCK_STRETCHING_MAX_COUNT   != 0)   ? $clog2(CLOCK_STRETCHING_MAX_COUNT) : 1;

wire            timeout_wait_time_clock;
wire            timeout_wait_time_reset_n;
wire            timeout_wait_time_enable;
logic           timeout_wait_time_load_count;
wire  [15:0]    timeout_wait_time_count;
wire            timeout_wait_time_expired;

wire divider_tick = divider_tick_r;
wire safe_reset_n = reset_n & pll_locked;  // use this if you have 'pll_locked'

typedef enum
{
    S_IDLE                  = 0,
    S_START                 = 1,
    S_WRITE_ADDR_W          = 2,
    S_CHECK_ACK             = 3,
    S_WRITE_REG_ADDR        = 4,
    S_RESTART               = 5,
    S_WRITE_ADDR_R          = 6,
    S_READ_REG              = 7,
    S_SEND_NACK             = 8,
    S_SEND_STOP             = 9,
    S_WRITE_REG_DATA        = 10,
    S_SEND_ACK              = 11
} state_type;

state_type                                  state;
state_type                                  state_n;               
state_type                                  post_state;             
state_type                                  post_state_n;           
reg                                         serial_clock;           
logic                                       scl_n;
reg     [ADDRESS_WIDTH:0]                   saved_device_address;
logic   [ADDRESS_WIDTH:0]                   dev_addr_n;
reg     [REGISTER_WIDTH-1:0]                saved_register_address;
logic   [REGISTER_WIDTH-1:0]                reg_addr_n;
reg     [DATA_WIDTH-1:0]                    saved_mosi_data;
logic   [DATA_WIDTH-1:0]                    mosi_n;
reg     [1:0]                               process_counter;        
logic   [1:0]                               proc_cnt_n;
reg     [3:0]                               bit_counter;
logic   [3:0]                               bit_cnt_n;
reg                                         serial_data;
logic                                       sda_n;
reg                                         post_serial_data;
logic                                       sda_post_n;
reg                                         last_acknowledge;
logic                                       last_ack_n;
logic                                       rw_n;
reg                                         saved_read_write;
reg     [15:0]                              divider_counter;
logic   [15:0]                              div_cnt_n;
logic   [DATA_WIDTH-1:0]                    miso_n;
logic                                       serial_data_output_enable;
logic                                       serial_clock_output_enable;
logic   [$clog2(MAX_NUMBER_BYTES)-1:0]      byte_cnt_n;
reg     [$clog2(MAX_NUMBER_BYTES)-1:0]      byte_counter;

reg     [CLOCK_STRETCHING_TIMER_WIDTH-1:0]  counter;
logic   [CLOCK_STRETCHING_TIMER_WIDTH-1:0]  counter_temp;


assign scl          = (serial_clock_output_enable)  ? serial_clock : 1'bz;
assign sda          = (serial_data_output_enable)   ? serial_data  : 1'bz;

assign timeout_wait_time_clock      = clock;
assign timeout_wait_time_reset_n    = reset_n;
assign timeout_wait_time_enable     = (CLOCK_STRETCHING_MAX_COUNT != 0) ? divider_tick : 0;
assign timeout_wait_time_count      = CLOCK_STRETCHING_MAX_COUNT;


//===========================================================================
//=============================== CYCLONE PLL ===============================
//===========================================================================
// ==================== PLL-to-system clock synchronizer ====================
// Goal: sync pll_clk_300k to the 'clock' domain (the FSM system clock)
reg [2:0] pll_sync;
always @(posedge clock or negedge safe_reset_n) begin
    if (!safe_reset_n) begin
        pll_sync        <= 3'b111;      // assume idle level is 1
    end else begin
        pll_sync        <= {pll_sync[1:0], pll_clk_300k};
    end
end
// ==================== Falling-edge detect: make divider_tick ==============
// divider_tick = 1 for one 'clock' cycle when pll_clk_300k has a falling edge
reg divider_tick_r;
always @(posedge clock or negedge safe_reset_n) begin
    if (!safe_reset_n) begin
        divider_tick_r <= 1'b0;
    end else begin
        // falling edge (1 -> 0) after synchronization
        divider_tick_r <= (pll_sync[2:1] == 2'b10);
    end
end


//===========================================================================
//==================== FSM I2C TRANSACTION HANDLER ==========================
//===========================================================================
always_comb begin
    state_n                         = state;
    post_state_n                    = post_state;
    proc_cnt_n                      = process_counter;
    bit_cnt_n                       = bit_counter;
    last_ack_n                      = last_acknowledge;
    miso_n                          = miso_data;
    rw_n                            = saved_read_write;
    div_cnt_n                       = divider_counter;
    reg_addr_n                      = saved_register_address;
    dev_addr_n                      = saved_device_address;
    mosi_n                          = saved_mosi_data;
    sda_n                           = serial_data;
    scl_n                           = serial_clock;
    sda_post_n                      = post_serial_data;
    byte_cnt_n                      = byte_counter;
    timeout_wait_time_load_count    = 0;
    serial_data_output_enable       = 1;
    busy                            = (state == S_IDLE) ? 0 : 1;

    if (state != S_IDLE && process_counter != 1 && process_counter != 2) begin
        serial_clock_output_enable   = 1;
    end
    else begin
        serial_clock_output_enable   = 0;
    end

    if (process_counter == 0) begin
        timeout_wait_time_load_count = 1;
    end

    case (state)
        S_IDLE: begin
            serial_data_output_enable   = 0;
            proc_cnt_n                  = 0;
            bit_cnt_n                   = 0;
            last_ack_n                  = 0;
            sda_n                       = 1;
            scl_n                       = 1;
            rw_n                        = read_write;

            if (NUMBER_OF_DATA_BYTES == 0) begin
                mosi_n          = '0;
            end
            else begin
                mosi_n          = mosi_data;
            end
            if (NUMBER_OF_REGISTER_BYTES == 0) begin
                reg_addr_n      = '0;
            end
            else begin
                reg_addr_n      = register_address;
            end
            if (ADDRESS_WIDTH == 0) begin
                dev_addr_n      = '0;
            end
            else begin
                dev_addr_n      = {device_address, 1'b0};  // write
            end

            if (enable) begin
                state_n         = S_START;
                post_state_n    = S_WRITE_ADDR_W;
            end
        end

        S_START: begin
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        proc_cnt_n  = 1;
                    end
                    1: begin
                        sda_n       = 0;
                        proc_cnt_n  = 2;
                    end
                    2:  begin
                        bit_cnt_n   = ADDRESS_WIDTH + 1;
                        proc_cnt_n  = 3;
                    end
                    3:  begin
                        scl_n       = 0;
                        proc_cnt_n  = 0;
                        state_n     = post_state;
                        sda_n       = saved_device_address[ADDRESS_WIDTH];
                    end
                endcase
            end
        end
        S_WRITE_ADDR_W: begin
            if (process_counter == 3 && bit_counter == 0) begin
                serial_data_output_enable   = 0;
            end
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        scl_n       = 1;
                        proc_cnt_n  = 1;
                    end
                    1: begin
                        //check for clock stretching
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_wait_time_expired) begin
                                proc_cnt_n  = 0;
                                state_n     = S_IDLE;
                            end
                        end
                        if (scl || !CHECK_FOR_CLOCK_STRETCHING) begin
                            proc_cnt_n      = 2;
                            state_n         = S_WRITE_ADDR_W;
                        end
                    end
                    2: begin
                        scl_n               = 0;
                        bit_cnt_n           = bit_counter -   1;
                        proc_cnt_n          = 3;
                    end
                    3: begin
                        proc_cnt_n          = 0;

                        if (bit_counter == 0) begin
                            sda_post_n      = saved_register_address[REGISTER_WIDTH-1];
                            reg_addr_n      = {saved_register_address[REGISTER_WIDTH-2:0], saved_register_address[REGISTER_WIDTH-1]};
                            post_state_n    = S_WRITE_REG_ADDR;
                            state_n         = S_CHECK_ACK;
                            bit_cnt_n       = 8;
                            byte_cnt_n      = NUMBER_OF_REGISTER_BYTES - 1;
                        end
                        else begin
                            sda_n           = saved_device_address[ADDRESS_WIDTH-1];
                        end

                        if (ADDRESS_WIDTH != 0) begin
                            dev_addr_n      = {saved_device_address[ADDRESS_WIDTH-1:0], saved_device_address[ADDRESS_WIDTH]};
                        end
                    end
                endcase
            end
        end

        S_CHECK_ACK: begin
            serial_data_output_enable   = 0;
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        scl_n           = 1;
                        proc_cnt_n      = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_wait_time_expired) begin
                                proc_cnt_n  = 0;
                                state_n     = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (scl || !CHECK_FOR_CLOCK_STRETCHING) begin
                            last_ack_n      = 0;
                            proc_cnt_n      = 2;
                        end
                    end
                    2: begin
                        scl_n           = 0;

                        if (sda == 0) begin
                            last_ack_n  = 1;    
                        end
                        proc_cnt_n      = 3;
                    end
                    3:  begin
                        if (last_acknowledge == 1) begin
                            last_ack_n  = 0;
                            sda_n       = post_serial_data;
                            state_n     = post_state;
                        end
                        else begin
                            state_n     = S_SEND_STOP;
                        end
                        proc_cnt_n      = 0;
                    end
                endcase
            end
        end

        S_WRITE_REG_ADDR: begin
            if (process_counter == 3 && bit_counter == 0) begin
                serial_data_output_enable   = 0;
            end
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        scl_n           = 1;
                        proc_cnt_n      = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_wait_time_expired) begin
                                proc_cnt_n  = 0;
                                state_n     = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (scl || !CHECK_FOR_CLOCK_STRETCHING) begin
                            last_ack_n      = 0;
                            proc_cnt_n      = 2;
                        end
                    end
                    2: begin
                        scl_n               = 0;
                        bit_cnt_n           = bit_counter - 1;
                        proc_cnt_n          = 3;
                    end
                    3: begin
                        if (bit_counter == 0) begin
                            byte_cnt_n      = byte_counter - 1;
                            bit_cnt_n       = 8;
                            sda_n           = 0;
                            state_n         = S_CHECK_ACK;

                            if (byte_counter == 0) begin
                                if (saved_read_write == 0) begin
                                    post_state_n    = S_WRITE_REG_DATA;
                                    sda_post_n      = saved_mosi_data[DATA_WIDTH-1];
                                    mosi_n          = {saved_mosi_data[DATA_WIDTH-2:0], saved_mosi_data[DATA_WIDTH-1]};
                                    byte_cnt_n      = NUMBER_OF_DATA_BYTES - 1;
                                end
                                else begin
                                    post_state_n    = S_RESTART;
                                    byte_cnt_n      = 0;
                                    sda_post_n      = 1;
                                end
                            end
                            else begin
                                post_state_n        = S_WRITE_REG_ADDR;
                            end
                        end
                        else begin
                            sda_n                   = saved_register_address[REGISTER_WIDTH-1];
                            reg_addr_n              = {saved_register_address[REGISTER_WIDTH-2:0], saved_register_address[REGISTER_WIDTH-1]}; 
                        end
                        proc_cnt_n                  = 0;
                    end
                endcase
            end
        end

        S_WRITE_REG_DATA: begin
            if (process_counter == 3 && bit_counter == 0) begin
                serial_data_output_enable           = 0;
            end

            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        scl_n                       = 1;
                        proc_cnt_n                  = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_wait_time_expired) begin
                                proc_cnt_n          = 0;
                                state_n             = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (scl || !CHECK_FOR_CLOCK_STRETCHING) begin
                            last_ack_n              = 0;
                            proc_cnt_n              = 2;
                        end
                    end
                    2: begin
                        scl_n                       = 0;
                        bit_cnt_n                   = bit_counter - 1;
                        proc_cnt_n                  = 3;
                    end
                    3: begin
                        if (bit_counter == 0) begin
                            byte_cnt_n              = byte_counter - 1;
                            state_n                 = S_CHECK_ACK;
                            bit_cnt_n               = 8;
                            sda_n                   = 0;

                            if (byte_counter == 0) begin
                                byte_cnt_n          = 0;
                                post_state_n        = S_SEND_STOP;
                                sda_post_n          = 0;
                            end
                            else begin
                                post_state_n        = S_WRITE_REG_DATA;
                                sda_post_n          = saved_mosi_data[DATA_WIDTH-1];
                                mosi_n              = {saved_mosi_data[DATA_WIDTH-2:0], saved_mosi_data[DATA_WIDTH-1]};
                            end

                        end
                        else begin
                            sda_n                   = saved_mosi_data[DATA_WIDTH-1];
                            mosi_n                  = {saved_mosi_data[DATA_WIDTH-2:0], saved_mosi_data[DATA_WIDTH-1]};
                        end
                        proc_cnt_n                  = 0;
                    end
                endcase
            end
        end

        S_RESTART: begin
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        proc_cnt_n      = 1;
                    end
                    1: begin
                        proc_cnt_n      = 2;
                        scl_n           = 1;
                    end
                    2: begin
                        proc_cnt_n      = 3;
                    end
                    3: begin
                        state_n         = S_START;
                        post_state_n    = S_WRITE_ADDR_R;
                        proc_cnt_n      = 0;

                        if (ADDRESS_WIDTH == 0) begin
                            dev_addr_n  = '1;
                        end
                        else begin
                            dev_addr_n[0]    = 1; //read
                        end
                    end
                endcase
            end
        end

        S_WRITE_ADDR_R: begin
            if (process_counter == 3 && bit_counter == 0) begin
                serial_data_output_enable   = 0;
            end
            
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        scl_n           = 1;
                        proc_cnt_n      = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_wait_time_expired) begin
                                proc_cnt_n      = 0;
                                state_n         = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (scl || !CHECK_FOR_CLOCK_STRETCHING) begin
                            last_ack_n          = 0;
                            proc_cnt_n          = 2;
                        end
                    end
                    2: begin
                        scl_n                   = 0;
                        bit_cnt_n               = bit_counter - 1;
                        proc_cnt_n              = 3;
                    end
                    3: begin
                        proc_cnt_n              = 0;

                        if (bit_counter == 0) begin
                            post_state_n        = S_READ_REG;
                            sda_post_n          = 0;
                            state_n             = S_CHECK_ACK;
                            bit_cnt_n           = 8;
                            byte_cnt_n          = NUMBER_OF_DATA_BYTES - 1;
                        end
                        else begin
                            sda_n               = saved_device_address[ADDRESS_WIDTH-1];
                        end

                        if (ADDRESS_WIDTH != 0) begin
                            dev_addr_n          = {saved_device_address[ADDRESS_WIDTH-1:0], saved_device_address[ADDRESS_WIDTH]};
                        end
                    end
                endcase
            end
        end

        S_READ_REG: begin
            if (process_counter != 3) begin
                serial_data_output_enable       = 0;
            end

            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        scl_n                   = 1;
                        proc_cnt_n              = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_wait_time_expired) begin
                                proc_cnt_n      = 0;
                                state_n         = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (scl || !CHECK_FOR_CLOCK_STRETCHING) begin
                            last_ack_n          = 0;
                            proc_cnt_n          = 2;
                        end
                    end
                    2: begin
                        scl_n                   = 0;
                        //sample data on this rising edge of scl
                        miso_n[0]               = sda;
                        miso_n[DATA_WIDTH-1:1]  = miso_data[DATA_WIDTH-2:0];
                        bit_cnt_n               = bit_counter - 1;
                        proc_cnt_n              = 3;
                    end
                    3: begin
                        if (bit_counter == 0) begin
                            byte_cnt_n          = byte_counter - 1;
                            bit_cnt_n           = 8;
                            sda_n               = 0;

                            if (byte_counter == 0) begin
                                byte_cnt_n      = 0;
                                state_n         = S_SEND_NACK;
                            end
                            else begin
                                post_state_n    = S_READ_REG;
                                state_n         = S_SEND_ACK;
                            end
                        end
                        proc_cnt_n              = 0;
                    end
                endcase
            end
        end

        S_SEND_NACK: begin
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        scl_n                   = 1;
                        sda_n                   = 1;
                        proc_cnt_n              = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_wait_time_expired) begin
                                proc_cnt_n      = 0;
                                state_n         = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (scl || !CHECK_FOR_CLOCK_STRETCHING) begin
                            last_ack_n          = 0;
                            proc_cnt_n          = 2;
                        end
                    end
                    2: begin
                        proc_cnt_n              = 3;
                        scl_n                   = 0;
                    end
                    3: begin
                        state_n                 = S_SEND_STOP;
                        proc_cnt_n              = 0;
                        sda_n                   = 0;
                    end
                endcase
            end
        end

        S_SEND_ACK: begin
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        scl_n                   = 1;
                        proc_cnt_n              = 1;
                        sda_n                   = 0;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_wait_time_expired) begin
                                proc_cnt_n      = 0;
                                state_n         = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (scl || !CHECK_FOR_CLOCK_STRETCHING) begin
                            last_ack_n          = 0;
                            proc_cnt_n          = 2;
                        end
                    end
                    2: begin
                        proc_cnt_n              = 3;
                        scl_n                   = 0;
                    end
                    3: begin
                        state_n                 = post_state;
                        proc_cnt_n              = 0;
                    end
                endcase
            end
        end
        
        S_SEND_STOP: begin
            if (divider_tick) begin
                case (process_counter)
                    0: begin
                        scl_n                   = 1;
                        proc_cnt_n              = 1;
                    end
                    1: begin
                        if (CLOCK_STRETCHING_MAX_COUNT != 0) begin
                            if (timeout_wait_time_expired) begin
                                proc_cnt_n      = 0;
                                state_n         = S_IDLE;
                            end
                        end
                        //check for clock stretching
                        if (scl || !CHECK_FOR_CLOCK_STRETCHING) begin
                            last_ack_n          = 0;
                            proc_cnt_n          = 2;
                        end
                    end
                    2: begin
                        proc_cnt_n              = 3;
                        sda_n                   = 1;
                    end
                    3: begin
                        state_n                 = S_IDLE;
                    end
                endcase
            end
        end
    endcase
end

always_ff @(posedge clock) begin
    if (!reset_n) begin
        state                   <= S_IDLE;
        post_state              <= S_IDLE;
        process_counter         <= 0;
        bit_counter             <= 0;
        last_acknowledge        <= 0;
        miso_data               <= 0;
        saved_read_write        <= 0;
        divider_counter         <= 0;
        saved_device_address    <= 0;
        saved_register_address  <= 0;
        saved_mosi_data         <= 0;
        serial_clock            <= 0;
        serial_data             <= 0;
        post_serial_data        <= 0;
        byte_counter            <= 0;
    end
    else begin
        state                   <= state_n;
        post_state              <= post_state_n;
        process_counter         <= proc_cnt_n;
        bit_counter             <= bit_cnt_n;
        last_acknowledge        <= last_ack_n;
        miso_data               <= miso_n;
        saved_read_write        <= rw_n;
        divider_counter         <= div_cnt_n;
        saved_device_address    <= dev_addr_n;
        saved_register_address  <= reg_addr_n;
        saved_mosi_data         <= mosi_n;
        serial_clock            <= scl_n;
        serial_data             <= sda_n;
        post_serial_data        <= sda_post_n;
        byte_counter            <= byte_cnt_n;
    end
 end


//===========================================================================
//=============================== WAIT TIME =================================
//===========================================================================
always_comb begin
    counter_temp    =   counter;
    if (counter == 0) begin
        timeout_wait_time_expired = 1;
    end
    else begin
        timeout_wait_time_expired = 0;
    end

    if (timeout_wait_time_enable) begin
        if (timeout_wait_time_load_count) begin
            counter_temp = timeout_wait_time_count;
        end
        else begin
            if (counter == 0) begin
            end
            else begin
                counter_temp    = counter - 1;
            end
        end
    end
end

always_ff @(posedge clock or negedge reset_n) begin
    if (!reset_n) begin
        counter <=  '1;
    end
    else begin
        counter <=  counter_temp;
    end
end
    
endmodule

