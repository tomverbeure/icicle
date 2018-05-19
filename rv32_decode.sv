`ifndef RV32_DECODE
`define RV32_DECODE

`include "rv32_control.sv"
`include "rv32_regs.sv"

module rv32_decode (
    input clk,

    /* control in (from hazard) */
    input stall_in,
    input flush_in,

    /* control in (from fetch) */
    input branch_predicted_taken_in,

    /* control in (from writeback) */
    input [4:0] rd_in,
    input rd_write_in,

    /* data in */
    input [31:0] pc_in,
    input [31:0] instr_in,

    /* data in (from writeback) */
    input [31:0] rd_value_in,

    /* control out (to hazard) */
    output reg [4:0] rs1_unreg_out,
    output reg rs1_read_unreg_out,
    output reg [4:0] rs2_unreg_out,
    output reg rs2_read_unreg_out,
    output reg mem_fence_unreg_out,

    /* control out */
    output reg branch_predicted_taken_out,
    output reg valid_out,
    output reg [4:0] rs1_out,
    output reg [4:0] rs2_out,
    output reg [2:0] alu_op_out,
    output reg alu_sub_sra_out,
    output reg [1:0] alu_src1_out,
    output reg [1:0] alu_src2_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg [1:0] mem_width_out,
    output reg mem_zero_extend_out,
    output reg mem_fence_out,
    output reg csr_read_out,
    output reg csr_write_out,
    output reg [1:0] csr_write_op_out,
    output reg csr_src_out,
    output reg [1:0] branch_op_out,
    output reg branch_pc_src_out,
    output reg [4:0] rd_out,
    output reg rd_write_out,

    /* data out */
    output reg [31:0] pc_out,
    output reg [31:0] rs1_value_out,
    output reg [31:0] rs2_value_out,
    output reg [31:0] imm_value_out,
    output reg [11:0] csr_out
);
    reg [4:0] rs2;
    reg [4:0] rs1;
    reg [4:0] rd;

    assign rs2 = instr_in[24:20];
    assign rs1 = instr_in[19:15];
    assign rd  = instr_in[11:7];

    assign rs1_unreg_out = rs1;
    assign rs2_unreg_out = rs2;

    rv32_regs regs (
        .clk(clk),
        .stall_in(stall_in),

        /* control in */
        .rs1_in(rs1),
        .rs2_in(rs2),
        .rd_in(rd_in),
        .rd_write_in(rd_write_in),

        /* data in */
        .rd_value_in(rd_value_in),

        /* data out */
        .rs1_value_out(rs1_value_out),
        .rs2_value_out(rs2_value_out)
    );

    reg valid;
    reg rs1_read;
    reg rs2_read;
    reg [2:0] imm;
    reg [2:0] alu_op;
    reg alu_sub_sra;
    reg [1:0] alu_src1;
    reg [1:0] alu_src2;
    reg mem_read;
    reg mem_write;
    reg [1:0] mem_width;
    reg mem_zero_extend;
    reg mem_fence;
    reg csr_read;
    reg csr_write;
    reg [1:0] csr_write_op;
    reg csr_src;
    reg [1:0] branch_op;
    reg branch_pc_src;
    reg rd_write;

    assign rs1_read_unreg_out = rs1_read;
    assign rs2_read_unreg_out = rs2_read;
    assign mem_fence_unreg_out = mem_fence;

    rv32_control_unit control_unit (
        /* data in */
        .instr_in(instr_in),

        /* control in */
        .rs1_in(rs1),
        .rd_in(rd),

        /* control out */
        .valid_out(valid),
        .rs1_read_out(rs1_read),
        .rs2_read_out(rs2_read),
        .imm_out(imm),
        .alu_op_out(alu_op),
        .alu_sub_sra_out(alu_sub_sra),
        .alu_src1_out(alu_src1),
        .alu_src2_out(alu_src2),
        .mem_read_out(mem_read),
        .mem_write_out(mem_write),
        .mem_width_out(mem_width),
        .mem_zero_extend_out(mem_zero_extend),
        .mem_fence_out(mem_fence),
        .csr_read_out(csr_read),
        .csr_write_out(csr_write),
        .csr_write_op_out(csr_write_op),
        .csr_src_out(csr_src),
        .branch_op_out(branch_op),
        .branch_pc_src_out(branch_pc_src),
        .rd_write_out(rd_write)
    );

    reg [31:0] imm_value;

    rv32_imm_mux imm_mux (
        /* control in */
        .imm_in(imm),

        /* data in */
        .instr_in(instr_in),

        /* data out */
        .imm_value_out(imm_value)
    );

    reg [11:0] csr;

    assign csr = instr_in[31:20];

    always @(posedge clk) begin
        if (!stall_in) begin
            branch_predicted_taken_out <= branch_predicted_taken_in;
            valid_out <= valid;
            rs1_out <= rs1;
            rs2_out <= rs2;
            alu_op_out <= alu_op;
            alu_sub_sra_out <= alu_sub_sra;
            alu_src1_out <= alu_src1;
            alu_src2_out <= alu_src2;
            mem_read_out <= mem_read;
            mem_write_out <= mem_write;
            mem_width_out <= mem_width;
            mem_zero_extend_out <= mem_zero_extend;
            mem_fence_out <= mem_fence;
            csr_read_out <= csr_read;
            csr_write_out <= csr_write;
            csr_write_op_out <= csr_write_op;
            csr_src_out <= csr_src;
            branch_op_out <= branch_op;
            branch_pc_src_out <= branch_pc_src;
            rd_out <= rd;
            rd_write_out <= rd_write;

            pc_out <= pc_in;
            imm_value_out <= imm_value;
            csr_out <= csr;

            if (flush_in) begin
                valid_out <= 0;
                mem_read_out <= 0;
                mem_write_out <= 0;
                csr_read_out <= 0;
                csr_write_out <= 0;
                branch_op_out <= `RV32_BRANCH_OP_NEVER;
                rd_write_out <= 0;
            end
        end
    end
endmodule

`endif
