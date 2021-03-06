`timescale 1ns/1ps

`include "../defines.vh"

// Module Declaration
module id_stage 
       (
        clk,
        rst_n,
        d_instruction_i,
        e_ALU_op_o,
        e_STORE_op_o,
        e_LOAD_op_o,
        e_data_origin_o,
        e_is_load_store_o,  // execution_unit 
        e_pc4_o,
        e_brj_pc_o,
        e_data_target_o,
        e_data_wr_o,  // data_write
        e_data_rd_o,
        
        e_regfile_rs1_o,
        e_regfile_rs2_o,

        e_regfile_wr_o,
        e_regfile_waddr_o,
        
        e_imm_val_o,  //execution unit imm val rs1
        e_data_be_o,
        
        w_regfile_wr_i,
        w_regfile_waddr_i,
        w_regfile_rd_i,
        d_pc_i,
        d_pc4_i,
        stall_o,
        alu_i,
        m_regfile_rd_i,
        m_data_rd_i,  
        m_is_load_store_i,
        m_regfile_waddr_i,
        m_regfile_wr_i,
        brj_pc_o,
        brj_o
//        d_busy_alu_i, // Ongoing multi-cycle operation when high flag.
//        stall_general_o
        );

 input  wire                           clk;
 input  wire                           rst_n;
 input  wire [`DATA_WIDTH-1:0]         d_instruction_i;
 input  wire                           w_regfile_wr_i;
 input  wire [`REG_ADDR_WIDTH-1:0]     w_regfile_waddr_i;
 input  wire [`DATA_WIDTH-1:0]         w_regfile_rd_i;

 output reg  [`ALU_OP_WIDTH-1:0]       e_ALU_op_o;
 output reg  [1:0]                     e_STORE_op_o;
 output reg  [2:0]                     e_LOAD_op_o;
 output reg  [`DATA_ORIGIN_WIDTH-1:0]  e_data_origin_o;  // To indicate what data to use by the execution unit 
 output reg  [`DATA_WIDTH-1:0]         e_imm_val_o;
 output reg                            e_is_load_store_o;
 output reg                            e_data_wr_o;
 output reg                            e_data_rd_o;
 output reg                            e_regfile_wr_o;

 output reg  [`REG_ADDR_WIDTH-1:0]     e_regfile_waddr_o;
 output reg  [`DATA_WIDTH-1:0]         e_regfile_rs1_o;
 output reg  [`DATA_WIDTH-1:0]         e_regfile_rs2_o;
 output reg  [`MEM_TRANSFER_WIDTH-1:0] e_data_be_o;
 input  wire [`DATA_WIDTH-1:0]         d_pc_i;
 input  wire [`DATA_WIDTH-1:0]         d_pc4_i;
 output reg  [`DATA_WIDTH-1:0]         e_pc4_o;
 output reg  [`DATA_WIDTH-1:0]         e_brj_pc_o;
 output reg  [1:0]                     e_data_target_o;
 output wire                           stall_o;
 input  wire [`DATA_WIDTH-1:0]         alu_i;
 input  wire [`DATA_WIDTH-1:0]         m_regfile_rd_i;
 input  wire                           m_data_rd_i;
 input  wire                           m_is_load_store_i;
 input  wire [`REG_ADDR_WIDTH-1:0]     m_regfile_waddr_i;
 input  wire                           m_regfile_wr_i;
 output wire [`DATA_WIDTH-1:0]         brj_pc_o;
 output wire                           brj_o;
// input  wire                           d_busy_alu_i; // Flag of multi-cycle operation ongoing when high.
// output wire                           stall_general_o; // which enables the general stall of the core.
 
 wire [`REG_ADDR_WIDTH-1:0]            regfile_waddr_t; 
 wire [`DATA_WIDTH-1:0]                reg_file_rs1_t;
 wire [`DATA_WIDTH-1:0]                reg_file_rs2_t;
 wire [`ALU_OP_WIDTH-1:0]              ALU_op_t;
 wire [1:0]                            STORE_op_t;
 wire [2:0]                            LOAD_op_t;
 wire [`BR_OP_WIDTH-1:0]               BR_op_t;
 wire [`DATA_ORIGIN_WIDTH-1:0]         data_origin_t; 
 wire                                  reg_file_wr_t;
 wire                                  is_load_store_t;
 wire                                  data_wr_t;
 wire [`DATA_WIDTH-1:0]                imm_val_t;
 wire [`MEM_TRANSFER_WIDTH-1:0]        data_be_t;
 wire [1:0]                            data_target_t;
 wire                                  data_rd_t;
 wire                                  i_r1_t;
 wire                                  i_r2_t;
 wire                                  branch_t;
 wire                                  jalr_t;
 wire [`REG_ADDR_WIDTH-1:0]            regfile_raddr_rs1_t;
 wire [`REG_ADDR_WIDTH-1:0]            regfile_raddr_rs2_t;
 reg  [`DATA_WIDTH-1:0]                e_regfile_rs1_t;
 reg  [`DATA_WIDTH-1:0]                e_regfile_rs1_tt;
 reg  [`DATA_WIDTH-1:0]                e_regfile_rs2_t;
 reg  [`DATA_WIDTH-1:0]                e_regfile_rs2_tt;
 wire [`CSR_ADDR_WIDTH-1:0]            csr_addr_t; 
 reg  [`DATA_WIDTH-1:0]                csr_wdata_t;
 reg  [`DATA_WIDTH-1:0]                csr_wop1_t;
 wire                                  csr_wr_t;
 wire [`DATA_WIDTH-1:0]                csr_rdata_t;
 wire [`CSR_OP_WIDTH-1:0]              csr_op_t;
 wire [`CSR_IMM_WIDTH-1:0]             csr_imm_t;
 

 control_unit control_unit_inst(
        .instruction                   (d_instruction_i    ),  // Instruction from prog mem input
        .ALU_op                        (ALU_op_t           ),  // ALU operation output
        .STORE_op                      (STORE_op_t         ),  // Load Store Operation output
        .LOAD_op                       (LOAD_op_t          ),  // Load Store Operation output
        .BR_op_o                       (BR_op_t            ),  // Branch operation output
        .csr_imm_o                     (csr_imm_t          ),
        .csr_raddr_o                   (csr_addr_t         ),
        .csr_op_o                      (csr_op_t           ),
        .csr_wr_o                      (csr_wr_t           ),
        .data_origin_o                 (data_origin_t      ),  // Data origin output (Rs2 or imm or pc)
        .data_rd_o                     (data_rd_t          ),
        .data_target_o                 (data_target_t      ),
        .data_wr                       (data_wr_t          ),  // LoadStore indicator output
        .data_be_o                     (data_be_t          ),
        .branch_i                      (branch_t & !stall_o),  // Branch indicator output
        .brj_o                         (brj_o              ),    // During a stall for dataMem access, do not branch.
        .is_load_store                 (is_load_store_t    ),  // execution_unit 
        .regfile_raddr_rs1_o           (regfile_raddr_rs1_t),  // RS1 addr
        .regfile_raddr_rs2_o           (regfile_raddr_rs2_t),  // RS2 addr
        .regfile_waddr                 (regfile_waddr_t    ),  // RD addr
        .regfile_wr                    (reg_file_wr_t      ),  // RegFile write enable
        .imm_val_o                     (imm_val_t          ),  // execution unit imm val
        .i_r1_o                        (i_r1_t             ),
        .i_r2_o                        (i_r2_t             ),
        .jalr_o                        (jalr_t             )
    );


 reg_file reg_file_inst(
        .rst_n                         (rst_n              ),  // Reset Neg
        .clk                           (clk                ),  // Clock
        .regfile_we_i                  (w_regfile_wr_i     ),  // Write Enable
        .regfile_raddr_rs1_i           (regfile_raddr_rs1_t),  // Address of r1 Read
        .regfile_raddr_rs2_i           (regfile_raddr_rs2_t),  // Address of r2 Read
        .regfile_waddr_i               (w_regfile_waddr_i  ),  // Addres of Write Register
        .regfile_data_i                (w_regfile_rd_i     ),  // Data to write
        .regfile_rs1_o                 (reg_file_rs1_t     ),  // Output register 1
        .regfile_rs2_o                 (reg_file_rs2_t     )   // Output register 2
        );

 
 crs_unit crs_unit_inst(
        .rst_n                         (rst_n              ),
        .clk                           (clk                ),
        .csr_waddr_i                   (csr_addr_t         ),
        .csr_wdata_i                   (csr_wdata_t        ), 
        .csr_wr_i                      (csr_wr_t           ),
        .csr_raddr_i                   (csr_addr_t         ),
        .csr_rdata_o                   (csr_rdata_t        )
        );

 br br_inst(
        .BR_op_i                       (BR_op_t            ),
        .pc_i                          (d_pc_i             ),
        .imm_val_i                     (imm_val_t          ),
        .brj_pc_o                      (brj_pc_o           ),
        .branch_o                      (branch_t           ),
        .regfile_rs1_i                 (e_regfile_rs1_t    ),
        .regfile_rs2_i                 (e_regfile_rs2_t    ),
        .jalr_i                        (jalr_t             )
         );


  // The forwarding logic sets the ability to input operands at the ALU (through the output registers of the id_stage) from the ALU's output, the DataMem input (aka, 
  // the exe_stage output registers), the DataMem (through the mem_stage output registers) or the RegFile registers, while keeping the recording function to the correct
  // location. This means that whatever are the ALU's inputs, even if is a recent result, from the DataMem or Regfile, the correct operand will be at the input and 
  // whatever result will continue its way to be recorded by the instruction assignment.
  //Forwarding logic op1   
 always @(*)// 1st check if it writes on REGS. 2nd check if is not x0. 3rd check if is same register as requested. 4th check if is a load.
       if (e_regfile_wr_o & (e_regfile_waddr_o != 0) & (e_regfile_waddr_o == regfile_raddr_rs1_t)) e_regfile_rs1_t = alu_i;
  else if (m_regfile_wr_i & (m_regfile_waddr_i != 0) & (m_regfile_waddr_i == regfile_raddr_rs1_t)) e_regfile_rs1_t = m_regfile_rd_i;
  else if (w_regfile_wr_i & (w_regfile_waddr_i != 0) & (w_regfile_waddr_i == regfile_raddr_rs1_t)) e_regfile_rs1_t = w_regfile_rd_i;
  else e_regfile_rs1_t = reg_file_rs1_t;

  //Forwarding logic op2
 always @(*) 
       if (e_regfile_wr_o & (e_regfile_waddr_o != 0) & (e_regfile_waddr_o == regfile_raddr_rs2_t)) e_regfile_rs2_t = alu_i;
  else if (m_regfile_wr_i & (m_regfile_waddr_i != 0) & (m_regfile_waddr_i == regfile_raddr_rs2_t)) e_regfile_rs2_t = m_regfile_rd_i;
  else if (w_regfile_wr_i & (w_regfile_waddr_i != 0) & (w_regfile_waddr_i == regfile_raddr_rs2_t)) e_regfile_rs2_t = w_regfile_rd_i;
  else e_regfile_rs2_t = reg_file_rs2_t;
   
 // In the case where the requested operand is not yet loaded, stall PC and do not transmit read/write signals to further stages. 
 // If the LOAD op is at the EXE stage (this stage regs), stall for two clock cycles.
 // If the LOAD op is at the MEM stage (EXE stage regs), stall for a single clock cycle to retrieve the operand (the forwarding will do it).
  assign stall_o =   (e_data_rd_o & (  ((e_regfile_waddr_o == regfile_raddr_rs1_t) & i_r1_t)
                                     | ((e_regfile_waddr_o == regfile_raddr_rs2_t) & i_r2_t)))
                   | (m_data_rd_i & (  ((m_regfile_waddr_i == regfile_raddr_rs1_t) & i_r1_t)
                                     | ((m_regfile_waddr_i == regfile_raddr_rs2_t) & i_r2_t)));
 // For multi-cycle operations stall. General because stops the pipeline.
//  assign stall_general_o = d_busy_alu_i; 

 
 // CSR operands to write on rd. At EXE, rs1 = 0 and rs2 = CSR when executing CSR read operation.
 always @(*)
  case(|csr_op_t[1:0])
   1'd0: e_regfile_rs1_tt = e_regfile_rs1_t;
   1'd1: e_regfile_rs1_tt = {`DATA_WIDTH{1'b0}};
  endcase

 always @(*)
  case(|csr_op_t[1:0])
   1'd0: e_regfile_rs2_tt = e_regfile_rs2_t;
   1'd1: e_regfile_rs2_tt = csr_rdata_t;
  endcase

 // CSR operands to write on CSR. First take between RS1 and UIMM and then operate what to write.
 always @(*)
  case(csr_op_t[2])
   1'b0: csr_wop1_t = e_regfile_rs1_t;
   1'b1: csr_wop1_t = {{`DATA_WIDTH-`CSR_IMM_WIDTH{1'b0}}, csr_imm_t};
  endcase

 always @(*)
  case(csr_op_t[1:0])
   2'd0: csr_wdata_t = csr_rdata_t;               // ECALL/EBREAK. Don't write
   2'd1: csr_wdata_t = csr_wop1_t;                // CSRRW[I], CSR = RS1/CSRIMM
   2'd2: csr_wdata_t = csr_wop1_t | csr_rdata_t;  // CSRRS[I], set CSR positions that RS1/CSRIMM are high
   2'd3: csr_wdata_t = ~csr_wop1_t & csr_rdata_t; // CSRRC[I], clear CSR positions that RS1/CSRIMM are high
  endcase


    //Registered PC for pipeline
 always@(posedge clk or negedge rst_n)
  if (!rst_n) 
   begin
    e_regfile_waddr_o <= {`REG_ADDR_WIDTH{1'b0}};
    e_regfile_rs1_o <= {`DATA_WIDTH{1'b0}};
    e_regfile_rs2_o <= {`DATA_WIDTH{1'b0}};
    e_ALU_op_o <= {`ALU_OP_WIDTH{1'b0}};
    e_STORE_op_o <= 2'h0;
    e_LOAD_op_o <= 3'h0;
    e_data_origin_o <= {`DATA_ORIGIN_WIDTH{1'b0}};
    e_regfile_wr_o <= 1'b0;
    e_is_load_store_o <= 1'b0;
    e_data_wr_o <= 1'b0;
    e_data_rd_o <= 1'b0;
    e_imm_val_o <= {`DATA_WIDTH{1'b0}};
    e_data_be_o <= {`MEM_TRANSFER_WIDTH{1'b0}};
    e_data_target_o <= 2'b0;
    e_pc4_o <= {`DATA_WIDTH{1'b0}};
    e_brj_pc_o <= {`DATA_WIDTH{1'b0}};
   end
  else //if(!stall_general_o)
   begin
    e_regfile_waddr_o <= regfile_waddr_t;
    e_regfile_rs1_o <= e_regfile_rs1_tt;
    e_regfile_rs2_o <= e_regfile_rs2_tt;
    e_ALU_op_o <= stall_o ? `ALU_OP_ADD : ALU_op_t;
    e_STORE_op_o <= STORE_op_t;                     // To avoid executing an operation
    e_LOAD_op_o <= LOAD_op_t;                       // with wrong values or transmiting
    e_data_origin_o <= data_origin_t;               // through the pipeline read/write
    e_regfile_wr_o <= reg_file_wr_t & !stall_o;     // commands during a forwaring stall
    e_is_load_store_o <= is_load_store_t;           // because the operands are not loaded
    e_data_wr_o <= data_wr_t & !stall_o;            // yet from the dataMem, the stall_o is used.
    e_data_rd_o <= data_rd_t & !stall_o;        
    e_imm_val_o <= imm_val_t;
    e_data_be_o <= data_be_t;
    e_data_target_o <= data_target_t;
    e_pc4_o <= d_pc4_i;
    e_brj_pc_o <= brj_pc_o;
   end
     

endmodule