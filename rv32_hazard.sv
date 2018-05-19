`ifndef RV32_HAZARD
`define RV32_HAZARD

module rv32_hazard_unit (
    /* control in */
    input [4:0] decode_rs1_unreg_in,
    input decode_rs1_read_unreg_in,
    input [4:0] decode_rs2_unreg_in,
    input decode_rs2_read_unreg_in,
    input decode_mem_fence_unreg_in,

    input decode_mem_read_in,
    input decode_mem_fence_in,
    input [4:0] decode_rd_in,
    input decode_rd_write_in,

    input execute_mem_fence_in,

    input mem_branch_mispredicted_in,

    input instr_read_in,
    input instr_ready_in,

    input data_read_in,
    input data_write_in,
    input data_ready_in,

    /* control out */
    output reg fetch_stall_out,
    output reg fetch_flush_out,

    output reg decode_stall_out,
    output reg decode_flush_out,

    output reg execute_stall_out,
    output reg execute_flush_out,

    output reg mem_stall_out,
    output reg mem_flush_out
);
    reg rs1_matches;
    reg rs2_matches;
    reg fetch_wait_for_bus;
    reg fetch_wait_for_mem_read;
    reg fetch_wait_for_mem_fence;
    reg mem_wait_for_bus;

    assign rs1_matches = decode_rs1_unreg_in == decode_rd_in && decode_rs1_read_unreg_in;
    assign rs2_matches = decode_rs2_unreg_in == decode_rd_in && decode_rs2_read_unreg_in;
    assign fetch_wait_for_bus = instr_read_in && !instr_ready_in;
    assign fetch_wait_for_mem_read = (rs1_matches || rs2_matches) && |decode_rd_in && decode_mem_read_in && decode_rd_write_in;
    assign fetch_wait_for_mem_fence = decode_mem_fence_unreg_in || decode_mem_fence_in || execute_mem_fence_in;
    assign mem_wait_for_bus = (data_read_in || data_write_in) && !data_ready_in;

    assign fetch_stall_out = decode_stall_out || fetch_wait_for_mem_read || fetch_wait_for_bus;
    assign fetch_flush_out = 0;

    assign decode_stall_out = execute_stall_out;
    assign decode_flush_out = fetch_stall_out || mem_branch_mispredicted_in;

    assign execute_stall_out = mem_stall_out;
    assign execute_flush_out = decode_stall_out || mem_branch_mispredicted_in;

    assign mem_stall_out = mem_wait_for_bus;
    assign mem_flush_out = execute_stall_out;
endmodule

`endif
