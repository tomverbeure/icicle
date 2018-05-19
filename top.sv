`include "defines.sv"
`include "bus_arbiter.sv"
`include "pll.sv"
`include "ram.sv"
`include "rv32.sv"
`include "sync.sv"
`include "timer.sv"
`include "uart.sv"

module top (
`ifndef INTERNAL_OSC
    input clk,
`endif

`ifdef FLASH
    /* serial flash */
    output reg flash_clk,
    output reg flash_csn,
    inout flash_io0,
    inout flash_io1,
`endif

    /* LEDs */
    output reg [7:0] leds,

    /* UART */
    input uart_rx,
    output reg uart_tx
);

`ifdef FLASH
    reg flash_io0_en;
    reg flash_io0_in;
    reg flash_io0_out;

    reg flash_io1_en;
    reg flash_io1_in;
    reg flash_io1_out;
`endif

`ifdef INTERNAL_OSC
    reg clk;
    SB_HFOSC inthosc (
        .CLKHFPU(1'b1),
        .CLKHFEN(1'b1),
        .CLKHF(clk)
    );
`endif

`ifdef FLASH
    SB_IO #(
        .PIN_TYPE(6'b1010_01)
    ) flash_io [1:0] (
        .PACKAGE_PIN({flash_io1, flash_io0}),
        .OUTPUT_ENABLE({flash_io1_en, flash_io0_en}),
        .D_IN_0({flash_io1_in, flash_io0_in}),
        .D_OUT_0({flash_io1_out, flash_io0_out})
    );
`endif

    reg pll_clk;
    reg pll_locked_async;

    pll pll (
        .clock_in(clk),
        .clock_out(pll_clk),
        .locked(pll_locked_async)
    );

    reg pll_locked;
    reg reset;

    assign reset = ~pll_locked;

    sync sync (
        .clk(pll_clk),
        .in(pll_locked_async),
        .out(pll_locked)
    );

    /* instruction memory bus */
    reg [31:0] instr_address;
    reg instr_read;
    reg [31:0] instr_read_value;
    reg instr_ready;

    /* data memory bus */
    reg [31:0] data_address;
    reg data_read;
    reg data_write;
    reg [31:0] data_read_value;
    reg [3:0] data_write_mask;
    reg [31:0] data_write_value;
    reg data_ready;

    /* memory bus */
    reg [31:0] mem_address;
    reg mem_read;
    reg mem_write;
    reg [31:0] mem_read_value;
    reg [3:0] mem_write_mask;
    reg [31:0] mem_write_value;

    assign mem_read_value = ram_read_value | leds_read_value | uart_read_value | timer_read_value;

    bus_arbiter bus_arbiter (
        /* instruction memory bus */
        .instr_address_in(instr_address),
        .instr_read_in(instr_read),
        .instr_read_value_out(instr_read_value),
        .instr_ready(instr_ready),

        /* data memory bus */
        .data_address_in(data_address),
        .data_read_in(data_read),
        .data_write_in(data_write),
        .data_read_value_out(data_read_value),
        .data_write_mask_in(data_write_mask),
        .data_write_value_in(data_write_value),
        .data_ready(data_ready),

        /* common memory bus */
        .address_out(mem_address),
        .read_out(mem_read),
        .write_out(mem_write),
        .read_value_in(mem_read_value),
        .write_mask_out(mem_write_mask),
        .write_value_out(mem_write_value)
    );

    reg [63:0] cycle;

    rv32 rv32 (
        .clk(pll_clk),
        .reset_(!reset),

        /* instruction memory bus */
        .instr_address_out(instr_address),
        .instr_read_out(instr_read),
        .instr_read_value_in(instr_read_value),
        .instr_ready_in(instr_ready),

        /* data memory bus */
        .data_address_out(data_address),
        .data_read_out(data_read),
        .data_write_out(data_write),
        .data_read_value_in(data_read_value),
        .data_write_mask_out(data_write_mask),
        .data_write_value_out(data_write_value),
        .data_ready_in(data_ready),

        /* timer */
        .cycle_out(cycle)
    );

    reg ram_sel;
    reg leds_sel;
    reg uart_sel;
    reg timer_sel;

    always @(*) begin
        ram_sel = 0;
        leds_sel = 0;
        uart_sel = 0;
        timer_sel = 0;

        casez (mem_address)
            32'b00000000_00000000_????????_????????: ram_sel = 1;
            32'b00000000_00000001_00000000_000000??: leds_sel = 1;
            32'b00000000_00000010_00000000_0000????: uart_sel = 1;
            32'b00000000_00000011_00000000_0000????: timer_sel = 1;
        endcase
    end

    reg [31:0] ram_read_value;

    ram ram (
        .clk(pll_clk),

        /* memory bus */
        .address_in(mem_address),
        .sel_in(ram_sel),
        .read_value_out(ram_read_value),
        .write_mask_in(mem_write_mask),
        .write_value_in(mem_write_value)
    );

    reg [31:0] leds_read_value;

    assign leds_read_value = {24'b0, leds_sel ? leds : 8'b0};

    always @(posedge pll_clk) begin
        if (leds_sel && mem_write_mask[0])
            leds <= mem_write_value[7:0];
    end

    reg [31:0] uart_read_value;

    uart uart (
        .clk(pll_clk),
        .reset(reset),

        /* serial port */
        .rx_in(uart_rx),
        .tx_out(uart_tx),

        /* memory bus */
        .address_in(mem_address),
        .sel_in(uart_sel),
        .read_in(mem_read),
        .read_value_out(uart_read_value),
        .write_mask_in(mem_write_mask),
        .write_value_in(mem_write_value)
    );

    reg [31:0] timer_read_value;

    timer timer (
        .clk(pll_clk),
        .reset_(!reset),

        /* cycle count (from the CPU core) */
        .cycle_in(cycle),

        /* memory bus */
        .address_in(mem_address),
        .sel_in(timer_sel),
        .read_in(mem_read),
        .read_value_out(timer_read_value),
        .write_mask_in(mem_write_mask),
        .write_value_in(mem_write_value)
    );
endmodule
