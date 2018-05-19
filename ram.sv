`ifndef RAM
`define RAM

module ram (
    input clk,

    /* memory bus */
    input [31:0] address_in,
    input sel_in,
    output reg [31:0] read_value_out,
    input [3:0] write_mask_in,
    input [31:0] write_value_in
);
    reg [31:0] mem [2047:0];
    reg [31:0] read_value;

    initial
        $readmemh("progmem_syn.hex", mem);

    assign read_value_out = sel_in ? read_value : 0;

    always @(negedge clk) begin
        read_value <= {mem[address_in[31:2]][7:0], mem[address_in[31:2]][15:8], mem[address_in[31:2]][23:16], mem[address_in[31:2]][31:24]};

        if (sel_in) begin
            if (write_mask_in[3])
                mem[address_in[31:2]][7:0] <= write_value_in[31:24];

            if (write_mask_in[2])
                mem[address_in[31:2]][15:8] <= write_value_in[23:16];

            if (write_mask_in[1])
                mem[address_in[31:2]][23:16] <= write_value_in[15:8];

            if (write_mask_in[0])
                mem[address_in[31:2]][31:24] <= write_value_in[7:0];
        end
    end
endmodule

`endif
