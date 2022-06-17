// Code your testbench here
// or browse Examples
`timescale 1ns/1ps

//`include"testbench.v"

module tb_arithmeticologic();

	tb TB();
	
	initial begin 

		TB.pc = 32'b0;
    //TB.top_CoreMem_inst.instr_mem.rstinstrMem;
    //TB.iniinstrMem;
		// Initialize registers
		TB.clk = 1'b0;
		TB.rst_n = 1'b0;
		#100
		
		// Load memory
		//$readmemb("data/instrramMem_b.mem", TB.top_CoreMem_inst.mem_instr_inst.mem, 0, 3);
		//$readmemh("../data/dataMem_h.mem", TB.top_CoreMem_inst.mem_data_inst.mem, 0, 3);
		//$readmemh("F:/TFG/Codi_original/RV32I/data/dataMem_h.mem", TB.top_CoreMem_inst.instr_mem.sp_ram_wrap_instr_i.sp_ram_instr_i.mem_instr);
		
		$readmemh("F:/TFG/Codi_original/RV32I/data/dataMem_h.txt", TB.top_CoreMem_inst.instr_mem.sp_ram_wrap_instr_i.sp_ram_instr_i.mem_instr);
		//$readmemh("F:/TFG/Codi_original/RV32I/data/contador.txt", TB.top_CoreMem_inst.instr_mem.sp_ram_wrap_instr_i.sp_ram_instr_i.mem_instr);
		//TEST1------------------------------------------------------------------------
		/*
		TB.test_andC;
		TB.rst_n = 1'b0;
		#100

		TB.test_add;
	  	TB.rst_n = 1'b0;
		#100
		
		TB.test_and;
		TB.rst_n = 1'b0;
		#100

		TB.test_andC;
		TB.rst_n = 1'b0;
		#100

		TB.test_andC;
		TB.rst_n = 1'b0;
		#100
		
		TB.test_and;
		TB.rst_n = 1'b0;
		*/
		//TEST2------------------------------------------------------------------------
		/*
		TB.test_MIXED_andC_addI;
		TB.rst_n = 1'b0;
		*/
		//TEST3:BRANCH------------------------------------------------------------------------
		
		TB.test_bneqzC;
		TB.rst_n = 1'b0;
		#100

		TB.test_beqzC;
	  	TB.rst_n = 1'b0;
		#100
		/*
		TB.test_beqzC;
		#100
    TB.rst_n = 1'b0;
    #100
		TB.test_slli;
		#100
    TB.rst_n = 1'b0;
    #100
		TB.test_slti;
		#100
    TB.rst_n = 1'b0;
    #100
		TB.test_sltiu;
		*/
  #1000
	$finish;
end

endmodule   