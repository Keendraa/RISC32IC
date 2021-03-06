// Code your testbench here
// or browse Examples
`timescale 1ns/1ps

//`include"testbench.v"

module tb_load_store();

	tb TB();
	
	initial begin 

		TB.pc = 32'b0;
        //TB.top_CoreMem_inst.mem_instr_inst.initializeinstrMem;
		/*
		$readmemh("../data/contador.txt", TB.top_CoreMem_inst.instr_mem.sp_ram_wrap_instr_i.sp_ram_instr_i.mem_instr,0, 3);
		// Initialize registers
  		TB.clk = 1'b0;
  		TB.rst_n = 1'b0;
  		#100
		TB.rst_n = 1'b1;
		#40000
		*/
		// Initialize registers
		TB.clk = 1'b0;
		TB.rst_n = 1'b0;
		#100
		// Load memory
		//$readmemb("data/instrramMem_b.mem", TB.top_CoreMem_inst.mem_instr_inst.mem, 0, 3);
		//$readmemh("../data/dataMem_h.mem", TB.top_CoreMem_inst.mem_data_inst.mem, 0, 3);
		//$readmemh("F:/TFG/Codi_original/RV32I/data/data/dataMem_h.mem", TB.top_CoreMem_inst.instr_mem.sp_ram_wrap_instr_i.sp_ram_instr_i.mem_instr);

		//$readmemh("F:/TFG/Codi_original/RV32I/data/contador.txt", TB.top_CoreMem_inst.instr_mem.sp_ram_wrap_instr_i.sp_ram_instr_i.mem_instr);
		// Initialize registers
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
		/*
		//Load data from memory
		//$readmemh("../data/dataMem_h.mem", TB.top_CoreMem_inst.mem_data_inst.mem, 0, 3);
		$readmemh("../data/dataMem_h.mem", TB.top_CoreMem_inst.data_mem.sp_ram_data_i.mem_data_inst.mem, 0, 3);
		TB.test_store;
        #100
        TB.rst_n = 1'b0;
        #100
		//Load data from memory
		//$readmemh("../data/dataMem_h.mem", TB.top_CoreMem_inst.mem_data_inst.mem, 0, 3);
        $readmemh("../data/dataMem_h.mem", TB.top_CoreMem_inst.data_mem.sp_ram_data_i.mem_data_inst.mem, 0, 3);
		
        TB.test_store_stall;
		*/
	    $stop;
    end

endmodule 