`ifndef RV32
`define RV32

`include "rv32_decode.sv"
`include "rv32_execute.sv"
`include "rv32_fetch.sv"
`include "rv32_hazard.sv"
`include "rv32_mem.sv"

module rv32 (
    input clk,

    /* instruction memory bus */
    output reg [31:0] instr_address_out,
    output reg instr_read_out,
    input [31:0] instr_read_value_in,
    input instr_ready_in,

    /* data memory bus */
    output reg [31:0] data_address_out,
    output reg data_read_out,
    output reg data_write_out,
    input [31:0] data_read_value_in,
    output reg [3:0] data_write_mask_out,
    output reg [31:0] data_write_value_out,
    input data_ready_in,

    /* timer */
    output reg [63:0] cycle_out
);
    /* hazard -> fetch control */
    reg fetch_stall;
    reg fetch_flush;

    /* hazard -> decode control */
    reg decode_stall;
    reg decode_flush;

    /* hazard -> execute control */
    reg execute_stall;
    reg execute_flush;

    /* hazard -> mem control */
    reg mem_stall;
    reg mem_flush;

    /* fetch -> decode control */
    reg fetch_branch_predicted_taken;

    /* fetch -> decode data */
    reg [31:0] fetch_pc;
    reg [31:0] fetch_instr;

    /* decode -> hazard control */
    reg [4:0] decode_rs1_unreg;
    reg decode_rs1_read_unreg;
    reg [4:0] decode_rs2_unreg;
    reg decode_rs2_read_unreg;
    reg decode_mem_fence_unreg;

    /* decode -> execute control */
    reg decode_branch_predicted_taken;
    reg decode_valid;
    reg [4:0] decode_rs1;
    reg [4:0] decode_rs2;
    reg [2:0] decode_alu_op;
    reg decode_alu_sub_sra;
    reg [1:0] decode_alu_src1;
    reg [1:0] decode_alu_src2;
    reg decode_mem_read;
    reg decode_mem_write;
    reg [1:0] decode_mem_width;
    reg decode_mem_zero_extend;
    reg decode_mem_fence;
    reg decode_csr_read;
    reg decode_csr_write;
    reg [1:0] decode_csr_write_op;
    reg decode_csr_src;
    reg [1:0] decode_branch_op;
    reg decode_branch_pc_src;
    reg [4:0] decode_rd;
    reg decode_rd_write;

    /* decode -> execute data */
    reg [31:0] decode_pc;
    reg [31:0] decode_rs1_value;
    reg [31:0] decode_rs2_value;
    reg [31:0] decode_imm_value;
    reg [11:0] decode_csr;

    /* execute -> mem control */
    reg execute_branch_predicted_taken;
    reg execute_valid;
    reg execute_alu_non_zero;
    reg execute_mem_read;
    reg execute_mem_write;
    reg [1:0] execute_mem_width;
    reg execute_mem_zero_extend;
    reg execute_mem_fence;
    reg [1:0] execute_branch_op;
    reg [4:0] execute_rd;
    reg execute_rd_write;

    /* execute -> mem data */
    reg [31:0] execute_result;
    reg [31:0] execute_rs2_value;
    reg [31:0] execute_branch_pc;

    /* mem -> writeback control */
    reg mem_valid;
    reg [4:0] mem_rd;
    reg mem_rd_write;

    /* mem -> fetch control */
    reg mem_branch_mispredicted;

    /* mem -> writeback data */
    reg [31:0] mem_rd_value;

    /* mem -> fetch data */
    reg [31:0] mem_branch_pc;

    rv32_hazard_unit hazard_unit (
        /* control in */
        .decode_rs1_unreg_in(decode_rs1_unreg),
        .decode_rs1_read_unreg_in(decode_rs1_read_unreg),
        .decode_rs2_unreg_in(decode_rs2_unreg),
        .decode_rs2_read_unreg_in(decode_rs2_read_unreg),
        .decode_mem_fence_unreg_in(decode_mem_fence_unreg),

        .decode_mem_read_in(decode_mem_read),
        .decode_mem_fence_in(decode_mem_fence),
        .decode_rd_in(decode_rd),
        .decode_rd_write_in(decode_rd_write),

        .execute_mem_fence_in(execute_mem_fence),

        .mem_branch_mispredicted_in(mem_branch_mispredicted),

        .instr_read_in(instr_read_out),
        .instr_ready_in(instr_ready_in),

        .data_read_in(data_read_out),
        .data_write_in(data_write_out),
        .data_ready_in(data_ready_in),

        /* control out */
        .fetch_stall_out(fetch_stall),
        .fetch_flush_out(fetch_flush),

        .decode_stall_out(decode_stall),
        .decode_flush_out(decode_flush),

        .execute_stall_out(execute_stall),
        .execute_flush_out(execute_flush),

        .mem_stall_out(mem_stall),
        .mem_flush_out(mem_flush)
    );

    rv32_fetch fetch (
        .clk(clk),

        /* control in (from hazard) */
        .stall_in(fetch_stall),
        .flush_in(fetch_flush),

        /* control in (from mem) */
        .branch_mispredicted_in(mem_branch_mispredicted),

        /* control out (to memory bus) */
        .instr_read_out(instr_read_out),

        /* control out */
        .branch_predicted_taken_out(fetch_branch_predicted_taken),

        /* data in (from mem) */
        .branch_pc_in(mem_branch_pc),

        /* data in (from memory bus) */
        .instr_read_value_in(instr_read_value_in),

        /* data out */
        .pc_out(fetch_pc),
        .instr_out(fetch_instr),

        /* data out (to memory bus) */
        .instr_address_out(instr_address_out)
    );

    rv32_decode decode (
        .clk(clk),

        /* control in (from hazard) */
        .stall_in(decode_stall),
        .flush_in(decode_flush),

        /* control in (from fetch) */
        .branch_predicted_taken_in(fetch_branch_predicted_taken),

        /* control in (from writeback) */
        .rd_in(mem_rd),
        .rd_write_in(mem_rd_write),

        /* data in */
        .pc_in(fetch_pc),
        .instr_in(fetch_instr),

        /* data in (from writeback) */
        .rd_value_in(mem_rd_value),

        /* control out (to hazard) */
        .rs1_unreg_out(decode_rs1_unreg),
        .rs1_read_unreg_out(decode_rs1_read_unreg),
        .rs2_unreg_out(decode_rs2_unreg),
        .rs2_read_unreg_out(decode_rs2_read_unreg),
        .mem_fence_unreg_out(decode_mem_fence_unreg),

        /* control out */
        .branch_predicted_taken_out(decode_branch_predicted_taken),
        .valid_out(decode_valid),
        .rs1_out(decode_rs1),
        .rs2_out(decode_rs2),
        .alu_op_out(decode_alu_op),
        .alu_sub_sra_out(decode_alu_sub_sra),
        .alu_src1_out(decode_alu_src1),
        .alu_src2_out(decode_alu_src2),
        .mem_read_out(decode_mem_read),
        .mem_write_out(decode_mem_write),
        .mem_width_out(decode_mem_width),
        .mem_zero_extend_out(decode_mem_zero_extend),
        .mem_fence_out(decode_mem_fence),
        .csr_read_out(decode_csr_read),
        .csr_write_out(decode_csr_write),
        .csr_write_op_out(decode_csr_write_op),
        .csr_src_out(decode_csr_src),
        .branch_op_out(decode_branch_op),
        .branch_pc_src_out(decode_branch_pc_src),
        .rd_out(decode_rd),
        .rd_write_out(decode_rd_write),

        /* data out */
        .pc_out(decode_pc),
        .rs1_value_out(decode_rs1_value),
        .rs2_value_out(decode_rs2_value),
        .imm_value_out(decode_imm_value),
        .csr_out(decode_csr)
    );

    rv32_execute execute (
        .clk(clk),

        /* control in (from hazard) */
        .stall_in(execute_stall),
        .flush_in(execute_flush),

        /* control in */
        .branch_predicted_taken_in(decode_branch_predicted_taken),
        .valid_in(decode_valid),
        .rs1_in(decode_rs1),
        .rs2_in(decode_rs2),
        .alu_op_in(decode_alu_op),
        .alu_sub_sra_in(decode_alu_sub_sra),
        .alu_src1_in(decode_alu_src1),
        .alu_src2_in(decode_alu_src2),
        .mem_read_in(decode_mem_read),
        .mem_write_in(decode_mem_write),
        .mem_width_in(decode_mem_width),
        .mem_zero_extend_in(decode_mem_zero_extend),
        .mem_fence_in(decode_mem_fence),
        .csr_read_in(decode_csr_read),
        .csr_write_in(decode_csr_write),
        .csr_write_op_in(decode_csr_write_op),
        .csr_src_in(decode_csr_src),
        .branch_op_in(decode_branch_op),
        .branch_pc_src_in(decode_branch_pc_src),
        .rd_in(decode_rd),
        .rd_write_in(decode_rd_write),

        /* control in (from writeback) */
        .writeback_valid_in(mem_valid),
        .writeback_rd_in(mem_rd),
        .writeback_rd_write_in(mem_rd_write),

        /* data in */
        .pc_in(decode_pc),
        .rs1_value_in(decode_rs1_value),
        .rs2_value_in(decode_rs2_value),
        .imm_value_in(decode_imm_value),
        .csr_in(decode_csr),

        /* data in (from writeback) */
        .writeback_rd_value_in(mem_rd_value),

        /* control out */
        .branch_predicted_taken_out(execute_branch_predicted_taken),
        .valid_out(execute_valid),
        .alu_non_zero_out(execute_alu_non_zero),
        .mem_read_out(execute_mem_read),
        .mem_write_out(execute_mem_write),
        .mem_width_out(execute_mem_width),
        .mem_zero_extend_out(execute_mem_zero_extend),
        .mem_fence_out(execute_mem_fence),
        .branch_op_out(execute_branch_op),
        .rd_out(execute_rd),
        .rd_write_out(execute_rd_write),

        /* data out */
        .result_out(execute_result),
        .rs2_value_out(execute_rs2_value),
        .branch_pc_out(execute_branch_pc),

        /* data out (to timer) */
        .cycle_out(cycle_out)
    );

    rv32_mem mem (
        .clk(clk),

        /* control in (from hazard) */
        .stall_in(mem_stall),
        .flush_in(mem_flush),

        /* control in */
        .branch_predicted_taken_in(execute_branch_predicted_taken),
        .valid_in(execute_valid),
        .alu_non_zero_in(execute_alu_non_zero),
        .read_in(execute_mem_read),
        .write_in(execute_mem_write),
        .width_in(execute_mem_width),
        .zero_extend_in(execute_mem_zero_extend),
        .branch_op_in(execute_branch_op),
        .rd_in(execute_rd),
        .rd_write_in(execute_rd_write),

        /* data in */
        .result_in(execute_result),
        .rs2_value_in(execute_rs2_value),
        .branch_pc_in(execute_branch_pc),

        /* data in (from memory bus) */
        .data_read_value_in(data_read_value_in),

        /* control out */
        .valid_out(mem_valid),
        .branch_mispredicted_out(mem_branch_mispredicted),
        .rd_out(mem_rd),
        .rd_write_out(mem_rd_write),

        /* control out (to memory bus) */
        .data_read_out(data_read_out),
        .data_write_out(data_write_out),
        .data_write_mask_out(data_write_mask_out),

        /* data out */
        .rd_value_out(mem_rd_value),
        .branch_pc_out(mem_branch_pc),

        /* data out (to memory bus) */
        .data_address_out(data_address_out),
        .data_write_value_out(data_write_value_out)
    );
endmodule

`endif
