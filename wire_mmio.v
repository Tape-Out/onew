`ifndef ONE_WIRE_MMIO_V
`define ONE_WIRE_MMIO_V

`timescale 1ns/1ps

`ifndef WIRE_DEFAULT_SPEED
`define WIRE_DEFAULT_SPEED 0                                 // default: 0=standard mode, 1=overdrive mode
`endif

`ifndef WIRE_FIFO_DEPTH
`define WIRE_FIFO_DEPTH 32
`endif

module wire_mmio #(
    parameter [31:0]  BASE_ADDR      = 32'h8200_0000,
    parameter [31:0]  CLK_FREQ       = 32'd100_000_000,      // 100MHz
    parameter integer DEFAULT_SPEED  = `WIRE_DEFAULT_SPEED,
    parameter integer FIFO_DEPTH     = `WIRE_FIFO_DEPTH,
    parameter integer BUFFER_LEN     = 16,                   // 16 * 64, size is num of word(32bit)=LEN*2
    parameter integer SEARCH_NUM     = 16,                   // 16 * 64, ...  // todo!
    parameter integer RESET_TIMEOUT  = 1000                  // reset timeout cycles
)(
    input  wire                     clk,
    input  wire                     resetn,

    input  wire                     mem_valid,
    input  wire                     mem_instr,
    output reg                      mem_ready,
    input  wire [31:0]              mem_addr,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]              mem_wdata,
    /* verilator lint_on  UNUSEDSIGNAL */
    input  wire [3:0]               mem_wstrb,
    output reg  [31:0]              mem_rdata,

    inout  wire                     wire_io,

    output reg                      irq,
    input  wire                     eoi
);

    reg irq_next;

    reg [31:0] ctrl_reg;
    reg [31:0] status_reg;

    reg [63:0] search_tree;
    reg [(64*SEARCH_NUM*8)-1:0] search_roms;

    reg [(64*BUFFER_LEN*8)-1:0]  sendbuf;
    reg [(64*BUFFER_LEN*8)-1:0]  recvbuf;

    reg [$clog2(64*BUFFER_LEN*8)-1:0] send_ptr, send_count;
    reg [$clog2(64*BUFFER_LEN*8)-1:0] recv_ptr, recv_count;

    reg bus_busy;

    wire [31:0] wmask = { {8{mem_wstrb[3]}}, {8{mem_wstrb[2]}}, {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}} };
    wire [31:0] wdata = mem_wdata & wmask;

    wire        slot_bit_ready;
    wire        slot_bit_rdata;
    reg         slot_bit_valid;
    reg  [1:0]  slot_bit_cmd;
    reg         slot_bit_wdata;

    localparam [1:0]
        BIT_CMD_WRITE      = 0,
        BIT_CMD_READ       = 1,
        BIT_CMD_RESET      = 2;

    wire_slot #(
        .CLK_FREQ(CLK_FREQ)
    ) wire_slot_inst (
        .speed_mode(ctrl_reg[4]),
        .wire_io(wire_io),
        .bit_cmd(slot_bit_cmd),
        .bit_valid(slot_bit_valid),
        .bit_wdata(slot_bit_wdata),
        .clk(clk),
        .resetn(resetn),
        .bit_ready(slot_bit_ready),
        .bit_rdata(slot_bit_rdata)
    );

    localparam [31:0]
        RW_SEND_BUF         = BASE_ADDR + 32'h01 << 2,
        RO_RECV_BUF         = BASE_ADDR + 32'h04 << 2,
        RW_WIRE_CTRL        = BASE_ADDR + 32'h08,
        RO_WIRE_STATUS      = BASE_ADDR + 32'h0C,
        RO_ROMS_SEARCH      = BASE_ADDR + 32'h07 << 2,
        WO_SPEC_SEARCH      = BASE_ADDR + 32'h10,
        WO_SPEC_ALAME       = BASE_ADDR + 32'h18,
        WO_SPEC_1_ROM       = BASE_ADDR + 32'h1C,
        RW_SPEC_SPEED       = BASE_ADDR + 32'h20,
        RW_ROMS_ETREE_L     = BASE_ADDR + 32'h24,
        RW_ROMS_ETREE_H     = BASE_ADDR + 32'h28,
        RO_SIZE_SEND        = BASE_ADDR + 32'h2C,
        RO_SIZE_RECV        = BASE_ADDR + 32'h30,
        RW_CONT_SEND        = BASE_ADDR + 32'h34,
        RW_CONT_RECV        = BASE_ADDR + 32'h38,
        RO_SIZE_SEARCH      = BASE_ADDR + 32'h3C;

    localparam [1:0]
        SEND_IDLE        = 0,
        SEND_DING        = 1,
        SEND_ENDD        = 2,
        SEND_RSTN        = 3;

    localparam [1:0]
        RECV_IDLE        = 0,
        RECV_DING        = 1,
        RECV_ENDD        = 2,
        RECV_RSTN        = 3;

    reg [1:0] send_state;
    reg [1:0] recv_state;

    wire ctrl_wire_en   = ctrl_reg[0];
    wire need_bus_reset = ctrl_reg[1];
    wire send_is_submit = ctrl_reg[2];
    wire recv_is_submit = ctrl_reg[3];
    wire speed_mode     = ctrl_reg[4];     // 0=standard, 1=overdrive

    wire send_is_completed = status_reg[0];
    wire recv_is_completed = status_reg[1];
    wire bus_busy_error    = status_reg[2];

    wire [31:0] status_wire = {
        28'd0,
        bus_busy,
        bus_busy_error,
        recv_is_completed,
        send_is_completed
    };

    always @(posedge clk) begin: SEND_BUF
        if (!resetn) begin
            send_state <= SEND_IDLE;
            send_ptr <= 0;
            status_reg[0] <= 0;
            slot_bit_valid <= 0;
            slot_bit_wdata <= 0;
        end else begin
            case (send_state)
                SEND_IDLE: begin
                    if (send_is_submit && !send_is_completed && !bus_busy) begin
                        send_state <= need_bus_reset ? SEND_RSTN : SEND_DING;
                        send_ptr <= 0;
                        bus_busy <= 1;
                        status_reg[0] <= 0;
                    end
                end
                SEND_RSTN: begin
                    slot_bit_cmd <= BIT_CMD_RESET;
                    slot_bit_valid <= 1;
                    if (slot_bit_ready) begin
                        send_state <= SEND_DING;
                        slot_bit_valid <= 0;
                    end
                end
                SEND_DING: begin
                    if (send_ptr == send_count) begin
                        send_state <= SEND_ENDD;
                        slot_bit_valid <= 0;
                    end else begin
                        slot_bit_valid <= 1;
                        slot_bit_wdata <= sendbuf[send_ptr];
                        slot_bit_cmd   <= BIT_CMD_WRITE;
                        if (slot_bit_ready) begin
                            send_ptr <= send_ptr + 1;
                            slot_bit_valid <= 0;
                        end
                    end
                end
                SEND_ENDD: begin
                    slot_bit_valid <= 0;
                    send_state <= SEND_IDLE;
                    bus_busy <= 0;
                    status_reg[0] <= 1;
                end
                default: send_state <= SEND_IDLE;
            endcase
        end
    end

    always @(posedge clk) begin: RECV_BUF
        if (!resetn) begin
            recv_state <= RECV_IDLE;
            recv_ptr <= 0;
            status_reg[1] <= 0;
        end else begin
            case (recv_state)
                RECV_IDLE: begin
                    if (recv_is_submit && !recv_is_completed && !bus_busy) begin
                        recv_state <= need_bus_reset ? RECV_RSTN : RECV_DING;
                        recv_ptr <= 0;
                        bus_busy <= 1;
                        status_reg[1] <= 0;
                    end
                end
                RECV_RSTN: begin
                    slot_bit_cmd <= BIT_CMD_RESET;
                    slot_bit_valid <= 1;
                    if (slot_bit_ready) begin
                        recv_state <= RECV_DING;
                        slot_bit_valid <= 0;
                    end
                end
                RECV_DING: begin
                    if (recv_ptr == recv_count) begin
                        recv_state <= RECV_ENDD;
                        slot_bit_valid <= 0;
                    end else begin
                        slot_bit_valid <= 1;
                        slot_bit_cmd   <= BIT_CMD_READ;
                        if (slot_bit_ready) begin
                            recvbuf[recv_ptr] <= slot_bit_rdata;
                            recv_ptr <= recv_ptr + 1;
                            slot_bit_valid <= 0;
                        end
                    end
                end
                RECV_ENDD: begin
                    slot_bit_valid <= 0;
                    recv_state <= RECV_IDLE;
                    bus_busy <= 0;
                    status_reg[1] <= 1;
                end
                default: recv_state <= RECV_IDLE;
            endcase
        end
    end

    integer buf_index, word_offset;

    always @(posedge clk) begin: MMIO_READ
        if (!resetn) begin
            mem_rdata <= 0;
        end else begin
            if (mem_valid && (!mem_instr) && mem_wstrb == 0) begin
                case (mem_addr)
                    RW_WIRE_CTRL    : mem_rdata <= ctrl_reg;
                    RO_WIRE_STATUS  : mem_rdata <= status_wire;
                    RW_SPEC_SPEED   : mem_rdata <= {31'b0, speed_mode};
                    RW_ROMS_ETREE_L : mem_rdata <= search_tree[31: 0];
                    RW_ROMS_ETREE_H : mem_rdata <= search_tree[63:32];
                    RO_SIZE_SEND    : mem_rdata <= 2*BUFFER_LEN;
                    RO_SIZE_RECV    : mem_rdata <= 2*BUFFER_LEN;
                    RO_SIZE_SEARCH  : mem_rdata <= SEARCH_NUM*2;
                    default: begin
                        if (RW_SEND_BUF <= mem_addr&&mem_addr < RW_SEND_BUF + 2*BUFFER_LEN) begin: READ_SEND_BUF
                            buf_index = (mem_addr - RW_SEND_BUF) >> 3;
                            word_offset = (mem_addr - RW_SEND_BUF) & 32'h4;
                            if (word_offset == 0) begin
                                mem_rdata <= sendbuf[(buf_index * 64) +: 32];
                            end else begin
                                mem_rdata <= sendbuf[(buf_index * 64) + 32 +: 32];
                            end
                        end else if (RO_RECV_BUF <= mem_addr&&mem_addr < RO_RECV_BUF + 2*BUFFER_LEN) begin: READ_RECV_BUF
                            buf_index = (mem_addr - RO_RECV_BUF) >> 3;
                            word_offset = (mem_addr - RO_RECV_BUF) & 32'h4;

                            if (word_offset == 0) begin
                                mem_rdata <= recvbuf[(buf_index * 64) +: 32];
                            end else begin
                                mem_rdata <= recvbuf[(buf_index * 64) + 32 +: 32];
                            end
                        end else if (RO_ROMS_SEARCH <= mem_addr&&mem_addr < RO_ROMS_SEARCH + 2*SEARCH_NUM) begin: READ_SEARCH_REGS
                            buf_index = (mem_addr - RO_ROMS_SEARCH) >> 3;
                            word_offset = (mem_addr - RO_ROMS_SEARCH) & 32'h4;
                            if (word_offset == 0) begin
                                mem_rdata <= search_roms[(buf_index * 64) +: 32];
                            end else begin
                                mem_rdata <= search_roms[(buf_index * 64) + 32 +: 32];
                            end
                        end else begin
                            mem_rdata <= 0;
                        end
                    end
                endcase
            end
        end
    end

    always @(posedge clk) begin: MMIO_WRITE
        if (!resetn) begin
            bus_busy <= 0;
            ctrl_reg[0] <= 1;
            ctrl_reg[1] <= 1;
            ctrl_reg[2] <= 0;
            ctrl_reg[3] <= 0;
            ctrl_reg[4] <= 0;
            status_reg[0] <= 0;
            status_reg[1] <= 0;
            send_count <= 0;
            recv_count <= 0;
        end else begin
            if (mem_valid && (!mem_instr) && mem_wstrb != 0) begin
                case (mem_addr)
                    RW_CONT_SEND    : send_count            <= wdata[$clog2(64*BUFFER_LEN*8)-1:0];
                    RW_CONT_RECV    : recv_count            <= wdata[$clog2(64*BUFFER_LEN*8)-1:0];
                    RW_ROMS_ETREE_L : search_tree[31: 0]    <= wdata;
                    RW_ROMS_ETREE_H : search_tree[63:32]    <= wdata;
                    RW_WIRE_CTRL    : begin
                        ctrl_reg[1] <= wdata[1];
                        ctrl_reg[2] <= ctrl_reg[2] ? 1 : (!recv_is_submit && !wdata[3] && !bus_busy);
                        ctrl_reg[3] <= ctrl_reg[3] ? 1 : (!send_is_submit && !wdata[2] && !bus_busy);
                        status_reg[2] <= wdata[2] && wdata[3];
                    end
                    WO_SPEC_1_ROM   : begin end
                    RW_SPEC_SPEED   : begin
                        ctrl_reg[4] <= wdata[0];
                    end
                    WO_SPEC_SEARCH  : begin end
                    WO_SPEC_ALAME   : begin end
                    default: begin
                        if (RW_SEND_BUF <= mem_addr&&mem_addr < RW_SEND_BUF + 2*BUFFER_LEN) begin: WRITE_SEND_BUF
                            buf_index = (mem_addr - RW_SEND_BUF) >> 3;
                            word_offset = (mem_addr - RW_SEND_BUF) & 32'h4;
                            if (word_offset == 0) begin
                                sendbuf[(buf_index * 64) +: 32] <= wdata;
                            end else begin
                                sendbuf[(buf_index * 64) + 32 +: 32] <= wdata;
                            end
                        end
                    end
                endcase
            end
        end
    end

    // CRC-8 Dallas/Maxim Polynomial: x^8 + x^5 + x^4 + 1
    // function [7:0] crc8;
    //     input [7:0] data;
    //     input [7:0] crc;
    //     reg [7:0] new_crc;
    //     integer i;
    //     begin
    //         new_crc = crc;
    //         for (i = 0; i < 8; i = i + 1) begin
    //             if (data[i] ^ new_crc[0]) begin
    //                 new_crc = (new_crc >> 1) ^ 8'h8C;
    //             end else begin
    //                 new_crc = new_crc >> 1;
    //             end
    //         end
    //         crc8 = new_crc;
    //     end
    // endfunction

    always @(posedge clk) begin
        if (!resetn) begin
            mem_ready <= 0;
        end else mem_ready <= mem_valid && !mem_instr;
    end

    always @(*) begin
        irq_next = bus_busy_error;
    end

    always @(posedge clk) begin
        if (!resetn)
            irq <= 0;
        else
            irq <= eoi ? 0 : irq_next;
    end

endmodule

module wire_slot #(
    parameter [31:0]  CLK_FREQ       = 32'd100_000_000       // 100MHz
)(
    input wire          speed_mode,
    inout wire          wire_io,
    input wire [1:0]    bit_cmd,
    input wire          bit_valid,
    input wire          bit_wdata,
    input wire          clk, resetn,
    output reg          bit_ready,
    output reg          bit_rdata
);
    /* manage wire state only */

    localparam [1:0]
        BIT_CMD_WRITE      = 0,
        BIT_CMD_READ       = 1,
        BIT_CMD_RESET      = 2;

    localparam
        SPEED_STANDARD   = 1'b0,   // standard mode: 15.3kbps
        SPEED_OVERDRIVE  = 1'b1;   // overdrive mode: 125kbps

    reg [31:0] t_low_init;
    reg [31:0] t_write1_high;
    reg [31:0] t_write0_low;
    reg [31:0] t_write0_recovery;
    reg [31:0] t_read_sample;
    reg [31:0] t_read_recovery;
    reg [31:0] t_reset_delay;
    reg [31:0] t_reset_low;
    reg [31:0] t_presence;
    reg [31:0] t_reset_recovery;

    reg [4:0] wire_state;
    reg       wire_oe, wire_out;

    assign wire_io = wire_oe ? 1'bZ : wire_out;

    always @(*) begin
        case (speed_mode)
            SPEED_STANDARD: begin     // Standard Speed (~15.3kbps)
                t_low_init        = (CLK_FREQ * 6)     / 1000000;   // A = 6 µs
                t_write1_high     = (CLK_FREQ * 64)    / 1000000;   // B = 64 µs
                t_write0_low      = (CLK_FREQ * 60)    / 1000000;   // C = 60 µs
                t_write0_recovery = (CLK_FREQ * 10)    / 1000000;   // D = 10 µs
                t_read_sample     = (CLK_FREQ * 9)     / 1000000;   // E = 9 µs
                t_read_recovery   = (CLK_FREQ * 55)    / 1000000;   // F = 55 µs
                t_reset_delay     = 0;                              // G = 0 µs
                t_reset_low       = (CLK_FREQ * 480)   / 1000000;   // H = 480 µs
                t_presence        = (CLK_FREQ * 70)    / 1000000;   // I = 70 µs
                t_reset_recovery  = (CLK_FREQ * 410)   / 1000000;   // J = 410 µs
            end
            SPEED_OVERDRIVE: begin    // Overdrive Speed (~125kbps)
                t_low_init        = (CLK_FREQ * 1)     / 1000000;   // A = 1.0 µs
                t_write1_high     = (CLK_FREQ * 75)    /  100000;   // B = 7.5 µs
                t_write0_low      = (CLK_FREQ * 75)    /  100000;   // C = 7.5 µs
                t_write0_recovery = (CLK_FREQ * 25)    /  100000;   // D = 2.5 µs
                t_read_sample     = (CLK_FREQ * 1)     / 1000000;   // E = 1.0 µs
                t_read_recovery   = (CLK_FREQ * 7)     / 1000000;   // F = 7 µs
                t_reset_delay     = (CLK_FREQ * 25)    /  100000;   // G = 2.5 µs
                t_reset_low       = (CLK_FREQ * 70)    / 1000000;   // H = 70 µs
                t_presence        = (CLK_FREQ * 85)    /  100000;   // I = 8.5 µs
                t_reset_recovery  = (CLK_FREQ * 40)    / 1000000;   // J = 40 µs
            end
            default: begin
                t_low_init        = (CLK_FREQ * 6)     / 1000000;
                t_write1_high     = (CLK_FREQ * 64)    / 1000000;
                t_write0_low      = (CLK_FREQ * 60)    / 1000000;
                t_write0_recovery = (CLK_FREQ * 10)    / 1000000;
                t_read_sample     = (CLK_FREQ * 9)     / 1000000;
                t_read_recovery   = (CLK_FREQ * 55)    / 1000000;
                t_reset_delay     = 0;
                t_reset_low       = (CLK_FREQ * 480)   / 1000000;
                t_presence        = (CLK_FREQ * 70)    / 1000000;
                t_reset_recovery  = (CLK_FREQ * 410)   / 1000000;
            end
        endcase
    end

    reg [31:0] time_cnt;

    localparam [4:0]
        WIRE_STATE_IDLE         = 0,
        WIRE_RW_PRE             = 1,
        WIRE_WRITE_BIT          = 2,
        WIRE_WRITE_GAP          = 3,
        WIRE_READ_SAMPLE        = 4,
        WIRE_READ_RECOVERY      = 5,
        WIRE_RESET_PRE          = 6,
        WIRE_RESET_LOW          = 7,
        WIRE_PRESENCE_TM        = 8,
        WIRE_RESET_RECOVERY     = 9;

    always @(posedge clk) begin: WIRE_IO
        if (!resetn) begin
            wire_oe <= 1;
            wire_out <= 1;
            time_cnt <= 0;
            wire_state <= WIRE_STATE_IDLE;
            bit_ready <= 0;
            bit_rdata <= 0;
        end else begin
            bit_ready <= 0;
            bit_rdata <= 0;
            if (bit_valid) begin
                case (wire_state)
                    WIRE_STATE_IDLE     : begin
                        wire_out  <= 1;
                        wire_oe <= 1;
                        if (bit_cmd !=0 && bit_valid) begin
                            if (bit_ready) begin
                                bit_ready <= 0;
                            end else begin
                                case (bit_cmd)
                                    BIT_CMD_WRITE, BIT_CMD_READ: begin
                                        wire_state <= WIRE_RW_PRE;
                                        time_cnt <= t_low_init;
                                        wire_out <= 0;
                                        wire_oe <= 0;
                                    end
                                    BIT_CMD_RESET              : begin
                                        wire_state <= WIRE_RESET_PRE;
                                        time_cnt <= t_reset_delay;
                                        wire_out <= 1;
                                        wire_oe <= 0;
                                    end
                                    default: wire_state <= WIRE_STATE_IDLE;
                                endcase
                            end
                        end
                    end
                    WIRE_RW_PRE         : begin
                        if (time_cnt == 0) begin
                            case (bit_cmd)
                                BIT_CMD_READ : begin
                                    wire_state <= WIRE_READ_SAMPLE;
                                    wire_oe <= 1;
                                    time_cnt <= t_read_sample;
                                end
                                BIT_CMD_WRITE: begin
                                    wire_state <= WIRE_WRITE_BIT;
                                    time_cnt <= t_write0_low-t_low_init;
                                    wire_oe <= 0;
                                    wire_out <= bit_wdata;
                                end
                                default: wire_state <= WIRE_STATE_IDLE;
                            endcase
                        end else begin
                            time_cnt <= time_cnt - 1;
                            wire_oe <= 0;
                            wire_out  <= 0;
                        end
                    end
                    WIRE_WRITE_BIT      : begin
                        if (time_cnt == 0) begin
                            wire_state <= WIRE_WRITE_GAP;
                            time_cnt <= t_write0_recovery;
                            wire_oe <= 1;
                            wire_out <= 1;
                        end else begin
                            time_cnt <= time_cnt - 1;
                            wire_oe <= 0;
                            wire_out <= bit_wdata;
                        end
                    end
                    WIRE_WRITE_GAP      : begin
                        if (time_cnt == 0) begin
                            wire_state <= WIRE_STATE_IDLE;
                            wire_oe <= 1;
                            wire_out <= 1;
                            bit_ready <= 1;
                            bit_rdata <= 0;
                        end else begin
                            time_cnt <= time_cnt - 1;
                            wire_oe <= 1;
                            wire_out <= 1;
                        end
                    end
                    WIRE_READ_SAMPLE    : begin
                        if (time_cnt == 0) begin
                            bit_rdata <= wire_io;
                            wire_state <= WIRE_READ_RECOVERY;
                            time_cnt <= t_read_recovery;
                            wire_oe <= 1;
                            wire_out <= 1;
                        end else begin
                            wire_oe <= 1;
                            wire_out <= 1;
                            time_cnt <= time_cnt - 1;
                        end
                    end
                    WIRE_READ_RECOVERY  : begin
                        if (time_cnt == 0) begin
                            wire_state <= WIRE_STATE_IDLE;
                            wire_oe <= 1;
                            wire_out  <= 1;
                            bit_ready <= 1;
                        end else begin
                            wire_oe <= 1;
                            wire_out <= 1;
                            time_cnt <= time_cnt - 1;
                        end
                    end
                    WIRE_RESET_PRE      : begin
                        if (time_cnt == 0) begin
                            wire_state <= WIRE_RESET_LOW;
                            wire_oe <= 0;
                            wire_out <= 0;
                            time_cnt <= t_reset_low;
                        end else begin
                            time_cnt <= time_cnt - 1;
                            wire_oe <= 0;
                            wire_out <= 1;
                        end
                    end
                    WIRE_RESET_LOW      : begin
                        if (time_cnt == 0) begin
                            wire_state <= WIRE_PRESENCE_TM;
                            wire_oe <= 1;
                            wire_out <= 1;
                            time_cnt <= t_presence;
                        end else begin
                            time_cnt <= time_cnt - 1;
                            wire_oe <= 0;
                            wire_out <= 0;
                        end
                    end
                    WIRE_PRESENCE_TM    : begin
                        if (time_cnt == 0) begin
                            wire_state <= WIRE_RESET_RECOVERY;
                            wire_oe <= 1;
                            wire_out <= 1;
                            time_cnt <= t_reset_recovery;
                            bit_rdata <= ~wire_io;
                        end else begin
                            time_cnt <= time_cnt - 1;
                            wire_oe <= 1;
                            wire_out <= 1;
                        end
                    end
                    WIRE_RESET_RECOVERY : begin
                        if (time_cnt == 0) begin
                            wire_state <= WIRE_STATE_IDLE;
                            wire_oe <= 1;
                            wire_out <= 1;
                            bit_ready <= 1;
                        end else begin
                            time_cnt <= time_cnt - 1;
                            wire_oe <= 1;
                            wire_out <= 1;
                        end
                    end
                    default: wire_state <= WIRE_STATE_IDLE;
                endcase
            end else begin
                if (!bit_valid) begin
                    bit_ready <= 0;
                end
            end
        end
    end
endmodule

`endif
