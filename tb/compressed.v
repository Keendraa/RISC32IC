// Code your testbench here
// or browse Examples
`timescale 1ns/1ps

//`include"testbench.v"

module tb_compressed();

	tb TB();
	
	initial begin 

		//TEST1------------------------------------------------------------------------
		TB.pc = 32'b0;
		TB.clk = 1'b0;
		TB.rst_n = 1'b0;
		#100

		$readmemh("F:/TFG/Codi_original/RV32I/data/dataMem_h.txt", TB.top_CoreMem_inst.instr_mem.sp_ram_wrap_instr_i.sp_ram_instr_i.mem_instr);
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
		
		//TEST2------------------------------------------------------------------------
		/*
		TB.pc = 32'b0;
		TB.clk = 1'b0;
		TB.rst_n = 1'b0;
		#100

		$readmemh("F:/TFG/Codi_original/RV32I/data/dataMem_h.txt", TB.top_CoreMem_inst.instr_mem.sp_ram_wrap_instr_i.sp_ram_instr_i.mem_instr)
		TB.test_MIXED_andC_addI;
		TB.rst_n = 1'b0;
		*/
		//TEST3:BRANCH------------------------------------------------------------------------
		/*

		TB.pc = 32'b0;
		TB.clk = 1'b0;
		TB.rst_n = 1'b0;
		#100

		$readmemh("F:/TFG/Codi_original/RV32I/data/dataMem_h.txt", TB.top_CoreMem_inst.instr_mem.sp_ram_wrap_instr_i.sp_ram_instr_i.mem_instr)
		TB.test_bneqzC;
		TB.rst_n = 1'b0;
		#100

		TB.test_beqzC;
	  	TB.rst_n = 1'b0;
		#100
		*/
		//TEST4: INSTRUCCIONS COMPRIMIDES------------------------------------------------------------------------
		/*
		TB.pc = 32'b0;
		TB.clk = 1'b0;
		TB.rst_n = 1'b0;
		#100

		$readmemh("F:/TFG/Codi_original/RV32I/data/dataMem_h.txt", TB.top_CoreMem_inst.instr_mem.sp_ram_wrap_instr_i.sp_ram_instr_i.mem_instr)
		TB.test_addC;
		TB.rst_n = 1'b0;
		#100

		TB.test_subC;
	  	TB.rst_n = 1'b0;
		#100
		
		TB.test_addiC;
	  	TB.rst_n = 1'b0;
		#100
		*/

		//TEST5:Restador------------------------------------------------------------------------
		/*
		TB.pc = 32'b0;

		TB.clk = 1'b0;
		TB.rst_n = 1'b0;
		#100

		$readmemh("F:/TFG/Codi_original/RV32I/data/restador.txt", TB.top_CoreMem_inst.instr_mem.sp_ram_wrap_instr_i.sp_ram_instr_i.mem_instr);
  		TB.clk = 1'b0;
  		TB.rst_n = 1'b0;
  		#100
		TB.rst_n = 1'b1;
		#40000
		
		TB.test_load;
		#100
		TB.rst_n = 1'b0;
		#100
		*/

  #100000
	$finish;
end

endmodule   