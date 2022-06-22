`timescale 1ns/1ps
`include "../../defines.vh"

module control_unit 
       (
        instruction,
        ALU_op,
        STORE_op,
        LOAD_op,
        BR_op_o,
        csr_imm_o,
        csr_raddr_o,
        csr_op_o,
        csr_wr_o,
        data_origin_o,
        branch_i,
        brj_o,  // branch indicator
        is_load_store,  // execution_unit 
        regfile_raddr_rs1_o,
        regfile_raddr_rs2_o,
        regfile_waddr,
        regfile_wr,
        imm_val_o,  //execution unit imm val rs1
        data_rd_o,
        data_target_o,
        data_wr,  // data_wrrite
        data_be_o,
        i_r1_o,
        i_r2_o,
        jalr_o
        );

localparam OPCODE_U_LUI    = 7'b0110111;
localparam OPCODE_U_AUIPC  = 7'b0010111;
localparam OPCODE_J_JAL    = 7'b1101111;
localparam OPCODE_I_JALR   = 7'b1100111;
localparam OPCODE_B_BRANCH = 7'b1100011;
localparam OPCODE_I_LOAD   = 7'b0000011;  //
localparam OPCODE_S_STORE  = 7'b0100011;  //
localparam OPCODE_I_IMM    = 7'b0010011;
localparam OPCODE_R_ALU    = 7'b0110011;  // PROV NAME
localparam OPCODE_I_FENCE  = 7'b0001111;  //
localparam OPCODE_I_SYSTEM = 7'b1110011;

localparam REGS            = 0;
localparam RS2IMM_RS1      = 1;
localparam RS2IMM_RS1PC    = 2;

    // FUNCT3
    //  I_JARL
localparam FUNCT3_JARL        = 3'b000;

    //  B_BRANCH
localparam FUNCT3_BEQ         = 3'b000;
localparam FUNCT3_BNE         = 3'b001;
localparam FUNCT3_BLT         = 3'b100;
localparam FUNCT3_BGE         = 3'b101;
localparam FUNCT3_BLTU        = 3'b110;
localparam FUNCT3_BGEU        = 3'b111;

    // I_LOAD
localparam FUNCT3_LB          = 3'b000;
localparam FUNCT3_LH          = 3'b001;
localparam FUNCT3_LW          = 3'b010;
localparam FUNCT3_LBU         = 3'b100;
localparam FUNCT3_LHU         = 3'b101;

    //  S_STORES
localparam FUNCT3_SB          = 3'b000;
localparam FUNCT3_SH          = 3'b001;
localparam FUNCT3_SW          = 3'b010;

    //  I_IMM
localparam FUNCT3_ADDI        = 3'b000;
localparam FUNCT3_SLTI        = 3'b010;
localparam FUNCT3_SLTIU       = 3'b011;
localparam FUNCT3_XORI        = 3'b100;
localparam FUNCT3_ORI         = 3'b110;
localparam FUNCT3_ANDI        = 3'b111;
localparam FUNCT3_SLLI        = 3'b001;
localparam FUNCT3_SRLI_SRAI   = 3'b101;

    //  R_ALU
localparam FUNCT3_ADD_SUB     = 3'b000;
localparam FUNCT3_SLL         = 3'b001;
localparam FUNCT3_SLT         = 3'b010;
localparam FUNCT3_SLTU        = 3'b011;
localparam FUNCT3_XOR         = 3'b100;
localparam FUNCT3_SRL_SRA     = 3'b101;
localparam FUNCT3_OR          = 3'b110;
localparam FUNCT3_AND         = 3'b111;

    //  R_ALU_M
localparam FUNCT3_MUL         = 3'b000;
localparam FUNCT3_MULH        = 3'b001;
localparam FUNCT3_MULHSU      = 3'b010;
localparam FUNCT3_MULHU       = 3'b011;
localparam FUNCT3_DIV         = 3'b100;
localparam FUNCT3_DIVU        = 3'b101;
localparam FUNCT3_REM         = 3'b110;
localparam FUNCT3_REMU        = 3'b111;

    //  I_SYSTEM
localparam FUNCT3_ECALL_EBREAK = 3'b000;
localparam FUNCT3_CSRRW        = 3'b001;
localparam FUNCT3_CSRRS        = 3'b010;
localparam FUNCT3_CSRRC        = 3'b011;
localparam FUNCT3_CSRRWI       = 3'b101;
localparam FUNCT3_CSRRSI       = 3'b110;
localparam FUNCT3_CSRRCI       = 3'b111;

    // BR_OPERATIONS
localparam BR_NOOP = 0;
localparam BR_EQ   = 1;
localparam BR_NE   = 2;
localparam BR_LT   = 3;
localparam BR_LTU  = 4;
localparam BR_GE   = 5;
localparam BR_GEU  = 6;
localparam BR_EQZ   = 7;
localparam BR_NEQZ   = 8;
    
    // LOAD_OPERATIONS
localparam LOAD_LB  = 0;
localparam LOAD_LH  = 1;
localparam LOAD_LW  = 2;
localparam LOAD_LBU = 3;
localparam LOAD_LHU = 4;

    // STORE_OPERATIONS
localparam STORE_SB = 0;
localparam STORE_SH = 1;
localparam STORE_SW = 2;


//QUADRANT 01 COMPRESSED
localparam INTERNAL_COMPRESSED_OPCODE = 7'b0000000;
localparam OPCODE_C_AND = 10'b1000111101; 


input       [`DATA_WIDTH-1:0]         instruction;
output      [`ALU_OP_WIDTH-1:0]       ALU_op;
output reg  [`STORE_OP_WIDTH-1:0]     STORE_op;
output reg  [`LOAD_OP_WIDTH-1:0]      LOAD_op;
output      [`BR_OP_WIDTH-1:0]        BR_op_o;
output wire [`CSR_IMM_WIDTH-1:0]      csr_imm_o;
output wire [`CSR_ADDR_WIDTH-1:0]     csr_raddr_o;
output reg  [`CSR_OP_WIDTH-1:0]       csr_op_o;
output reg   csr_wr_o;
output      [`DATA_ORIGIN_WIDTH-1:0]  data_origin_o;  // To indicate what data to use by the execution unit 
output      [`DATA_WIDTH-1:0]         imm_val_o;
output       is_load_store;
output       data_wr;
output       regfile_wr;
output wire [`REG_ADDR_WIDTH-1:0]     regfile_raddr_rs1_o; // as a reg due to LUI requiring to
output wire [`REG_ADDR_WIDTH-1:0]     regfile_raddr_rs2_o;
output wire [`REG_ADDR_WIDTH-1:0]     regfile_waddr;
input  wire  branch_i;
output wire  brj_o;  // branch indicator
output      [`MEM_TRANSFER_WIDTH-1:0] data_be_o;
output reg [1:0]                      data_target_o;
output reg   data_rd_o;
output reg   i_r1_o; // To indicate Regfile access
output reg   i_r2_o; // To indicate Regfile access
output reg   jalr_o;
 
reg          data_wr;
reg          regfile_wr;
reg  [`DATA_WIDTH-1:0]                imm_val_o;
wire [`DATA_WIDTH-1:0]                instruction;
wire [6:0]   opcode; 
 //  Type R
wire [2:0]   funct3;
wire [6:0]   funct7;
 //  Type U
wire [19:0]  imm20;
 // Type I
wire [11:0]  imm12;
reg          is_load_store;
 // Type S
wire [11:0]  imm12s;
 // Type B
 //reg is_conditional_o;
wire [11:0]  imm12b;
wire [19:0]  imm20j;
 // Type J
reg          jump;
 // Decode
reg [`ALU_OP_WIDTH-1:0]         ALU_op;
reg [`BR_OP_WIDTH-1:0]          BR_op_o;
reg [`DATA_ORIGIN_WIDTH-1:0]    data_origin_o;
reg [`MEM_TRANSFER_WIDTH-1:0]   data_be_o;


//COMPRESSED AUXILIAR REGISTERS
reg [2:0] first_op_code;
reg [1:0] second_op_code;
reg [4:0] reg_d;
reg [4:0] rs_2;
reg [1:0] quadrant_code;

//OUTPUTS AND AUXILIARS
assign opcode = instruction [1:0] == 2'b11 ? instruction[6:0] : {7'b0000000}; //Adapted for Compressed instructions

assign funct7 = instruction[31:25];  
assign funct3 = instruction[14:12]; 
assign imm20 = instruction[31:12];
assign imm12 = instruction[31:20];
assign imm20j = {instruction[31], instruction[19:12], instruction[20], instruction[30:21]};
assign imm12b = {instruction[31], instruction[7], instruction[30:25], instruction[11:8]};
assign imm12s = {instruction[31:25], instruction[11:7]};

assign regfile_raddr_rs1_o = opcode == OPCODE_U_LUI ? 5'b0 : opcode == {7'b0000000} ? reg_d : instruction[19:15]; // OUT
assign regfile_raddr_rs2_o = opcode == {7'b0000000}? rs_2 : instruction[24:20] ; // OUT
assign regfile_waddr = opcode == {7'b0000000}? reg_d : instruction[11:7];  // OUT
assign brj_o = branch_i | jump; //OUT
assign csr_raddr_o = imm12;  //OUT
assign csr_imm_o = instruction[19:15]; //OUT
 
always@(*) 
  begin
    data_wr = 1'b0;
    data_be_o = {`MEM_TRANSFER_WIDTH{1'b0}};
    is_load_store = 1'b0;
    regfile_wr = 1'b0;
    jump = 1'b0;
    data_origin_o = `REGS; 
    ALU_op = `ALU_OP_ADD;
    BR_op_o = `BR_NOOP; 
    STORE_op = `STORE_SB;
    LOAD_op = `LOAD_LW; 
    imm_val_o = {`DATA_WIDTH{1'b0}};
    // Decode
    data_target_o = 2'b0;
    data_rd_o = 1'b0;
    i_r1_o = 1'b0;
    i_r2_o = 1'b0;
    jalr_o = 1'b0;
    csr_op_o = {`CSR_OP_WIDTH{1'b0}};
    csr_wr_o = 1'b0;
    reg_d <= 5'b00000;
    rs_2 <= 5'b00000;
    case(opcode)
      OPCODE_U_LUI: begin  // Set and sign extend the 20-bit immediate (shited 12 bits left) and zero the bottom 12 bits into rd
        data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value, in dis case 0
        imm_val_o = { imm20[19:0], {`DATA_WIDTH - 20 {1'b0}} };
        regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
        ALU_op = `ALU_OP_ADD;  // Sum with 0
      end

      OPCODE_U_AUIPC: begin  // Place the PC plus the 20-bit signed immediate (shited 12 bits left) into rd (used before JALR)
        data_target_o = 2'b11;
        imm_val_o = { imm20[19:0], {`DATA_WIDTH - 20 {1'b0}} };
        regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
        ALU_op = `ALU_OP_ADD;  // Add the values
      end

      OPCODE_J_JAL: begin  // Jump to the PC plus 20-bit signed immediate while saving PC+4 into rd
        data_target_o = 2'b11;          
        jump = 1'b1;
        data_origin_o = `RS2IMM_RS1PC;  // Send the immediate value and PC at the execution unit
        imm_val_o = {{`DATA_WIDTH - 21 {imm20j[19]}},  imm20j[19:0], 1'b0  }; // TODO last bit is used? or is always 0
        regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}}); // Write the resut in RD          
        ALU_op = `ALU_OP_ADD;  // to add the immideate value to the PC
      end

      OPCODE_I_JALR: begin  // jalr       "Jump to rs1 plus the 12-bit signed immediate while saving PC+4 into rd"
        data_target_o = 2'b11;
        i_r1_o = 1'b1;
        jump = 1'b1;
        jalr_o = 1'b1;
        data_origin_o = `RS2IMM_RS1PC;  // Send the immediate value and mantain RS1 the value
        ALU_op = `ALU_OP_ADD;  
        imm_val_o = {{`DATA_WIDTH - 12 {imm12[11]}},  imm12[11:0] }; // no ^2
        regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
      end

      OPCODE_B_BRANCH: begin
         data_origin_o = `REGS;  // Mantain RS2 value and RS1 value // DefaultValue
         i_r1_o = 1'b1;
         i_r2_o = 1'b1;
         imm_val_o = {{`DATA_WIDTH - 13 {imm12b[11]}},  imm12b[11:0], 1'b0  }; // TODO last bit is used? or is always 0
         case(funct3)
           FUNCT3_BEQ:  BR_op_o = `BR_EQ;   // beq        "Branch to PC relative 12-bit signed immediate (shifted 1 bit left) if rs1 == rs2"
           FUNCT3_BNE:  BR_op_o = `BR_NE;   // bne        "Branch to PC relative 12-bit signed immediate (shifted 1 bit left) if rs1 != rs2"
           FUNCT3_BLT:  BR_op_o = `BR_LT;   // blt        "Branch to PC relative 12-bit signed immediate (shifted 1 bit left) if rs1 < rs2 (signed)"
           FUNCT3_BGE:  BR_op_o = `BR_GE;   // bge        "Branch to PC relative 12-bit signed immediate (shifted 1 bit left) if rs1 >= rs2 (signed)"
           FUNCT3_BLTU: BR_op_o = `BR_LTU;  // bltu       "Branch to PC relative 12-bit signed immediate (shifted 1 bit left) if rs1 < rs2 (unsigned)"
           FUNCT3_BGEU: BR_op_o = `BR_GEU;  // bgeu       "Branch to PC relative 12-bit signed immediate (shifted 1 bit left) if rs1 >= rs2 (unsigned)"      
           default:     BR_op_o = `BR_NOOP;
         endcase
       end

      OPCODE_I_LOAD: begin  // Loads
         is_load_store = 1'b1;
         i_r1_o = 1'b1;
         regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});      
         ALU_op = `ALU_OP_ADD;  // to add the immideate value to the addr
         data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
         imm_val_o = {{`DATA_WIDTH - 12 {imm12[11]}},  imm12[11:0]  };
         data_rd_o = 1'b1;
         case(funct3)
          FUNCT3_LB:  LOAD_op = `LOAD_LB;   // lb         "Load 8-bit value from addr in rs1 plus the 12-bit signed immediate and place sign-extended result into rd"
          FUNCT3_LH:  LOAD_op = `LOAD_LH;   // lh         "Load 16-bit value from addr in rs1 plus the 12-bit signed immediate and place sign-extended result into rd"
          FUNCT3_LW:  LOAD_op = `LOAD_LW;   // lw         "Load 32-bit value from addr in rs1 plus the 12-bit signed immediate and place sign-extended result into rd"
          FUNCT3_LBU: LOAD_op = `LOAD_LBU;  // lbu        "Load 8-bit value from addr in rs1 plus the 12-bit signed immediate and place zero-extended result into rd"
          FUNCT3_LHU: LOAD_op = `LOAD_LHU;  // lhu        "Load 16-bit value from addr in rs1 plus the 12-bit signed immediate and place zero-extended result into rd"
          default: $display("Ilegal LOAD FUNCT3");  // TODO Throw interruption
         endcase
      end

      OPCODE_S_STORE: begin  // Store
        is_load_store = 1'b1;
        ALU_op = `ALU_OP_ADD;  // to add the immideate value to the addr
        data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
        imm_val_o = {{`DATA_WIDTH - 12 {imm12s[11]}}, imm12s[11:0] };
        data_wr = 1'b1;  // Set the bit to write to memory
        data_target_o = 2'b1;
        i_r1_o = 1'b1;
        i_r2_o = 1'b1;
        case(funct3)
          FUNCT3_SB: begin  // sb         "Store 8-bit value from the low bits of rs2 to addr in rs1 plus the 12-bit signed immediate"
            STORE_op = `STORE_SB;
            data_be_o = 4'b0001;
            end
          FUNCT3_SH: begin  // sh         "Store 16-bit value from the low bits of rs2 to addr in rs1 plus the 12-bit signed immediate"
            STORE_op = `STORE_SH;  
            data_be_o = 4'b0011;
            end 
          FUNCT3_SW: begin // sw         "Store 32-bit value from the low bits of rs2 to addr in rs1 plus the 12-bit signed immediate"
            STORE_op = `STORE_SW;  
            data_be_o = 4'b1111;
            end
          default: $display("Ilegal STORE FUNCT3");  // TODO Throw interruption
        endcase
      end

      OPCODE_I_IMM: begin
        data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
        regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});
        i_r1_o = 1'b1;
        case(funct3)
          FUNCT3_SRL_SRA: begin
            imm_val_o = {{`DATA_WIDTH - 5 {1'b0}}, imm12[4:0] };
            ALU_op = {funct7[5], funct3}; // To differentiate SRLI of SRAI
            end
          default: begin
            imm_val_o = { {`DATA_WIDTH - 12 {imm12[11]}}, imm12[11:0] };
            ALU_op = {{`ALU_OP_WIDTH-3{1'b0}}, funct3};
            end
        endcase
        /* Old way of encoding. Not necessary if well handled at the execution stage.
        case(funct3)
          FUNCT3_ADD_SUB: ALU_op = `ALU_OP_ADD;   // addi       "Add sign-extended 12-bit immediate to register rs1 and place the result in rd"
          FUNCT3_SLL:     ALU_op = `ALU_OP_SLL;   // slli       "Shift rs1 left by the 5 or 6 (RV32/64) bit (RV64) immediate and place the result into rd"
          FUNCT3_SLT:     ALU_op = `ALU_OP_SLT;   // slti       "Set rd to 1 if rs1 is less than the sign-extended 12-bit immediate, otherwise set rd to 0 (signed)"
          FUNCT3_SLTU:    ALU_op = `ALU_OP_SLTU;  // sltiu      "Set rd to 1 if rs1 is less than the sign-extended 12-bit immediate, otherwise set rd to 0 (unsigned)"
          FUNCT3_XOR:     ALU_op = `ALU_OP_XOR;   // xori       "Set rd to the bitwise xor of rs1 with the sign-extended 12-bit immediate"
          FUNCT3_SRL_SRA: ALU_op = funct7[5] == 1'b1 ? `ALU_OP_SRA : `ALU_OP_SRL; // srli       "Shift rs1 right by the 5 or 6 (RV32/64) bit immediate and place the result into rd" 
             // srai       "Shift rs1 right by the 5 or 6 (RV32/64) bit immediate and place the result into rd while retaining the sign"
          FUNCT3_OR:      ALU_op = `ALU_OP_OR;    // ori        "Set rd to the bitwise or of rs1 with the sign-extended 12-bit immediate"
          FUNCT3_AND:     ALU_op = `ALU_OP_AND;   // andi       "Set rd to the bitwise and of rs1 with the sign-extended 12-bit immediate"
        endcase */
      end

      OPCODE_R_ALU: begin
        regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});
        i_r1_o = 1'b1;
        i_r2_o = 1'b1;
        ALU_op = {funct7[5], funct3};
        /* Old way of encoding. Not necessary if well handled at the execution stage.
        if(funct7[0] == 1'b1) // Check if its an operation from Instruction set M
          case(funct3)
          FUNCT3_MUL    : ALU_op = `ALU_OP_MUL;
          FUNCT3_MULH   : ALU_op = `ALU_OP_MULH;
          FUNCT3_MULHSU : ALU_op = `ALU_OP_MULHSU;
          FUNCT3_MULHU  : ALU_op = `ALU_OP_MULHU;
          FUNCT3_DIV    : ALU_op = `ALU_OP_DIV;
          FUNCT3_DIVU   : ALU_op = `ALU_OP_DIVU;
          FUNCT3_REM    : ALU_op = `ALU_OP_REM;
          FUNCT3_REMU   : ALU_op = `ALU_OP_REMU;
          endcase
        else 
        case(funct3)
          FUNCT3_ADD_SUB : ALU_op = funct7[5] == 1'b1 ? `ALU_OP_SUB : `ALU_OP_ADD;
          FUNCT3_SLL     : ALU_op = `ALU_OP_SLL;
          FUNCT3_SLT     : ALU_op = `ALU_OP_SLT;
          FUNCT3_SLTU    : ALU_op = `ALU_OP_SLTU;
          FUNCT3_XOR     : ALU_op = `ALU_OP_XOR;
          FUNCT3_SRL_SRA : ALU_op = funct7[5] == 1'b1 ? `ALU_OP_SRA : `ALU_OP_SRL;
          FUNCT3_OR      : ALU_op = `ALU_OP_OR;
          FUNCT3_AND     : ALU_op = `ALU_OP_AND; 
        endcase */
      end

      OPCODE_I_FENCE: begin
      // fence      "Order device I/O and memory accesses viewed by other threads and devices"
      // fence.i    "Synchronize the instruction and data streams
      end

      OPCODE_I_SYSTEM: begin  // SYSTEM + CSR
        // All CSR instructions (CSRR[W,S,C][I]) write on the CSR destiny address.
        csr_wr_o = |funct3; // <- This discriminates CSR operations from ECALL/EBREAK

        // All CSR instructions (CSRR[W,S,C][I]) read old CSR and write it on rd, unless it's x0.
        regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}}) & (|funct3);
        ALU_op = `ALU_OP_ADD; //rs1 is set to 0 for EXE. rd = rs1 + rs2 = 0 + CSR

        // csr_op[2] = 1 indicates immediate. csr_op[1:0] indicates RW/RS/RC = 01/10/11. 
        csr_op_o = funct3;
        i_r1_o = !funct3[2];
      end

      INTERNAL_COMPRESSED_OPCODE:
      begin
        if(instruction!=32'b0)
        begin
        quadrant_code <= instruction[1:0];
        first_op_code <= instruction[15:13];
        
        case(quadrant_code)
          2'b00: //QUADRANT 00 ------------------------------------------------------------------------------------------------------------------------------
          case(first_op_code)
          3'b000:
          begin
            data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
            regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});
            i_r1_o = 1'b1;
            imm_val_o = {{`DATA_WIDTH - 9 {instruction[10]}}, instruction[9:7], instruction[12:11], instruction[5], instruction[6], 2'b00};
            reg_d <= instruction[4:2];
            ALU_op <= `ALU_OP_ADD;
          end
          //3'b001:
          3'b010://C.LW
          begin
            is_load_store = 1'b1;
            i_r1_o = 1'b1;
            i_r2_o = 1'b1;
            reg_d <= {3'b00,instruction[9:7]+5'b01000};
            rs_2 <= {3'b000,instruction[4:2]+5'b01000};
            regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});      
            ALU_op = `ALU_OP_ADD;  // to add the immideate value to the addr
            data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
            imm_val_o = {{`DATA_WIDTH - 6 {instruction[5]}},  instruction[12:10], instruction[6],2'b00  };
            data_rd_o = 1'b1;
            LOAD_op = `LOAD_LW;   // lw         "Load 32-bit value from addr in rs1 plus the 12-bit signed immediate and place sign-extended result into rd"
          end
          //3'b011:
          //3'b100: RESERVED
          //3'b101:
          3'b110://C.SW
          begin
            is_load_store = 1'b1;
            ALU_op = `ALU_OP_ADD;  // to add the immideate value to the addr
            data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
            imm_val_o = {{`DATA_WIDTH - 6 {instruction[5]}},  instruction[12:10], instruction[6], 2'b00  };
            data_wr = 1'b1;  // Set the bit to write to memory
            data_target_o = 2'b1;
            i_r1_o = 1'b1;
            i_r2_o = 1'b1;
            reg_d <= {3'b00,instruction[9:7]+5'b01000};
            rs_2 <= {3'b000,instruction[4:2]+5'b01000};
            STORE_op = `STORE_SW;  
            data_be_o = 4'b1111;
          end
          //3'b111:

          endcase
          2'b01://QUADRANT 01 ------------------------------------------------------------------------------------------------------------------------------
          begin

            case(first_op_code)
            3'b000://C.ADDI
            begin
              data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
              regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});
              i_r1_o = 1'b1;
              imm_val_o = {{`DATA_WIDTH - 5 {instruction[12]}}, instruction[6:2]};
              reg_d <= instruction[11:7];
              ALU_op <= `ALU_OP_ADD;
            end
            3'b001: //C.JAL
            begin
              data_target_o = 2'b11;          
              jump = 1'b1;
              data_origin_o = `RS2IMM_RS1PC;  // Send the immediate value and PC at the execution unit
              imm_val_o = {{`DATA_WIDTH - 11 {instruction[12]}}, instruction[8], instruction[10:9], instruction[6], instruction[7], instruction[2], instruction[11], instruction[5:3], 1'b0  }; 
              reg_d <= {5'b00001};//X[1]
              regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}}); // Write the resut in X[1]          
              ALU_op = `ALU_OP_ADD;  // to add the immideate value to the PC
            end
            3'b010: //C.LI
            begin
                data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value, in dis case 0
                imm_val_o = {{`DATA_WIDTH - 5 {instruction[12]}}, instruction[6:2]};
                regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
                ALU_op = `ALU_OP_ADD;  // Sum with 0              
            end
            3'b011: 
            begin
              if (instruction[11:7]==5'b00010)//C.ADDI16SP
              begin
                data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
                regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});
                i_r1_o = 1'b1;
                imm_val_o = {{`DATA_WIDTH - 11 {instruction[12]}}, instruction[4:3],instruction[5],instruction[2],instruction[6], 6'b000000};
                reg_d <= instruction[11:7];
                ALU_op <= `ALU_OP_ADD;               
              end
              else //C.LUI
              begin
                data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value, in dis case 0
                imm_val_o = {{`DATA_WIDTH - 17 {instruction[12]}}, instruction[6:2], 12'b0};
                regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
                ALU_op = `ALU_OP_ADD;  // Sum with 0
              end
            end
            3'b100: 
            begin
              if (instruction[12:10] == {3'b011} )
              begin
                  second_op_code = instruction[6:5];
                  case (second_op_code)
                    2'b00://C.SUB
                    begin
                      reg_d <= {3'b00,instruction[9:7]};
                      rs_2 <= {3'b000,instruction[4:2]};
                      ALU_op <= `ALU_OP_SUB;
                      regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
                      i_r2_o = 1'b1; // To indicate Regfile access
                      i_r1_o = 1'b1; // To indicate Regfile access
                    end
                    2'b01://C.XOR
                    begin
                      reg_d <= {3'b00,instruction[9:7]};
                      rs_2 <= {3'b000,instruction[4:2]};
                      ALU_op <= `ALU_OP_XOR;
                      regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
                      i_r2_o = 1'b1; // To indicate Regfile access
                      i_r1_o = 1'b1; // To indicate Regfile access
                    end
                    2'b10://C.OR
                    begin
                      reg_d <= {3'b00,instruction[9:7]};
                      rs_2 <= {3'b000,instruction[4:2]};
                      ALU_op <= `ALU_OP_OR;
                      regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
                      i_r2_o = 1'b1; // To indicate Regfile access
                      i_r1_o = 1'b1; // To indicate Regfile access
                    end
                    2'b11://C.AND
                    begin
                      reg_d <= {3'b00,instruction[9:7]};
                      rs_2 <= {3'b000,instruction[4:2]};
                      ALU_op <= `ALU_OP_AND;
                      regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
                      i_r2_o = 1'b1; // To indicate Regfile access
                      i_r1_o = 1'b1; // To indicate Regfile access
                      
                    end
                    /*2'b00:
                    begin
                    end*/
                  endcase
              end
              else if (instruction[11:10] == 3'b10) //C.AND
              begin
                data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
                imm_val_o = {{`DATA_WIDTH - 5 {instruction[12]}}, instruction[6:2]};
                reg_d <= {3'b00,instruction[9:7]};
                regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
                i_r1_o = 1'b1; // To indicate Regfile access
                ALU_op <= `ALU_OP_AND;
              end
              /*else if instruction[12:10] == 3'b001 and instruction[6:2]==5'b0 //C.SRAI64
              begin

              end
              else if instruction[12:10] == 3'b000 and instruction[6:2]==5'b0 //C.SRLI64
              begin

              end*/
              else if (instruction[11:10] == 2'b01) //C.SRAI
              begin
                data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
                imm_val_o = {{`DATA_WIDTH - 5 {instruction[12]}}, instruction[6:2]};
                reg_d <= {3'b00,instruction[9:7]};
                regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
                i_r1_o = 1'b1; // To indicate Regfile access
                ALU_op <= `ALU_OP_SRA;
              end
              else if (instruction[11:10] == 2'b00)//C.SRLI
              begin
                data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
                imm_val_o = {{`DATA_WIDTH - 5 {instruction[12]}}, instruction[6:2]};
                reg_d <= {3'b00,instruction[9:7]};
                regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
                i_r1_o = 1'b1; // To indicate Regfile access
                ALU_op <= `ALU_OP_SRL;
              end
              /*
              else if instruction[12:10] == 3'b111 
              begin
                second_op_code = instruction[6:5];
                case (second_op_code)
                  2'b00://C.SUBW
                  begin
                  end
                  2'b01://C.ADDW
                  begin
                  end
                endcase
              end*/
            end
            3'b101://C.J 
            begin
              data_target_o = 2'b11;          
              jump = 1'b1;
              data_origin_o = `RS2IMM_RS1PC;  // Send the immediate value and PC at the execution unit
              imm_val_o = {{`DATA_WIDTH - 11 {instruction[12]}}, instruction[8], instruction[10:9], instruction[6], instruction[7], instruction[2], instruction[11], instruction[5:3], 1'b0  }; 
              regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}}); // Write the resut in X[1]          
              ALU_op = `ALU_OP_ADD;  // to add the immideate value to the PC              
            end
            3'b110:
            begin
              data_origin_o = `REGS;  // Mantain RS2 value and RS1 value // DefaultValue
              i_r1_o = 1'b1;
              i_r2_o = 1'b0;
              reg_d <= {3'b00,instruction[9:7]};
              imm_val_o = {{`DATA_WIDTH - 9 {instruction[12]}}, instruction[12], instruction[6:5], instruction[2],instruction[11:10], instruction[4:3],1'b0};
              BR_op_o = `BR_EQZ;
            end
            3'b111:
            begin
              data_origin_o = `REGS;  // Mantain RS2 value and RS1 value // DefaultValue
              i_r1_o = 1'b1;
              i_r2_o = 1'b0;
              reg_d <= {3'b00,instruction[9:7]};
              imm_val_o = {{`DATA_WIDTH - 9 {instruction[12]}}, instruction[12], instruction[6:5], instruction[2],instruction[11:10], instruction[4:3],1'b0};
              BR_op_o = `BR_NEQZ;
            end
            endcase
          end
          2'b10:// QUADRANT 10 ------------------------------------------------------------------------------------------------------------------------------
          begin
            case(first_op_code)
            3'b000: //C.SLLI
            begin
              data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
              regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});
              i_r1_o = 1'b1;
              reg_d <= instruction[11:7];
              imm_val_o = {{`DATA_WIDTH - 5 {instruction[12]}}, instruction[6:2]};
              ALU_op = `ALU_OP_SLL;
            end
            3'b010: //C.LWSP
            begin
              is_load_store = 1'b1;
              i_r1_o = 1'b1;
              i_r2_o = 1'b1;
              reg_d <= {instruction[11:7]};
              rs_2 <= {5'b00010};
              regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});      
              ALU_op = `ALU_OP_ADD;  // to add the immideate value to the addr
              data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
              imm_val_o = {{`DATA_WIDTH - 7 {instruction[3]}},  instruction[2], instruction[12], instruction[6:4], 2'b00  };
              data_rd_o = 1'b1;
              LOAD_op = `LOAD_LW;   // lw         "Load 32-bit value from addr in rs1 plus the 12-bit signed immediate and place sign-extended result into rd"         
            end   
            3'b100: 
            begin
              if(instruction[12:2]==10'b10000000000)//C.EBREAK
              begin
                // All CSR instructions (CSRR[W,S,C][I]) write on the CSR destiny address.
                csr_wr_o = |`FUNCT3_ECALL_EBREAK; // <- This discriminates CSR operations from ECALL/EBREAK
                // All CSR instructions (CSRR[W,S,C][I]) read old CSR and write it on rd, unless it's x0.
                regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}}) & (|`FUNCT3_ECALL_EBREAK);
                ALU_op = `ALU_OP_ADD; //rs1 is set to 0 for EXE. rd = rs1 + rs2 = 0 + CSR
                // csr_op[2] = 1 indicates immediate. csr_op[1:0] indicates RW/RS/RC = 01/10/11. 
                csr_op_o = `FUNCT3_ECALL_EBREAK;
                i_r1_o = 1'b1;
              end
              else if((instruction[12]==10'b1) && (instruction[6:2]==5'b00000))//C.JALR
              begin
                data_target_o = 2'b11;
                i_r2_o = 1'b1;
                jump = 1'b1;
                jalr_o = 1'b1;
                data_origin_o = `RS2IMM_RS1PC;  // Send the immediate value and mantain RS1 the value
                ALU_op = `ALU_OP_ADD;  
                imm_val_o = {`DATA_WIDTH {0}}; 
                regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});  // Write the resut in RD
                rs_2 <= instruction[11:7];
                reg_d <= 5'b00001; 
              end
              else if((instruction[12]==10'b0) && (instruction[6:2]==5'b00000))//C.JR
              begin
                data_target_o = 2'b11;
                i_r1_o = 1'b1;
                jump = 1'b1;
                jalr_o = 1'b1;
                data_origin_o = `RS2IMM_RS1PC;  // Send the immediate value and mantain RS1 the value
                ALU_op = `ALU_OP_ADD;  
                imm_val_o = {`DATA_WIDTH {0}}; 
                reg_d <= instruction[11:7];
              end
              else if(instruction[12]==10'b0 )//C.MV
              begin
                data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
                regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});
                i_r2_o = 1'b1;
                i_r1_o = 1'b1;
                imm_val_o = {`DATA_WIDTH {0}};
                reg_d <= instruction[11:7];
                rs_2 <= instruction[6:2];
                ALU_op <= `ALU_OP_ADD;              
              end
              else if(instruction[12]==10'b1 )//C.ADD
              begin
                regfile_wr = (regfile_waddr != {`REG_ADDR_WIDTH {1'b0}});
                i_r2_o = 1'b1;
                i_r1_o = 1'b1;
                reg_d <= instruction[11:7];
                rs_2 <= instruction[6:2];
                ALU_op <= `ALU_OP_ADD;  
              end
            end
            3'b110: //C.SWSP
            begin
              is_load_store = 1'b1;
              ALU_op = `ALU_OP_ADD;  // to add the immideate value to the addr
              data_origin_o = `RS2IMM_RS1;  // Send the immediate value and mantain RS1 the value
              imm_val_o = {{`DATA_WIDTH - 7 {instruction[8]}},  instruction[7], instruction[12:9], 2'b00  };
              data_wr = 1'b1;  // Set the bit to write to memory
              data_target_o = 2'b1;
              i_r1_o = 1'b1;
              i_r2_o = 1'b1;
              reg_d <= {3'b00010};
              rs_2 <= {3'b000,instruction[4:2]+5'b01000};
              STORE_op = `STORE_SW;  
              data_be_o = 4'b1111;   
            end     
            endcase
          end
          
        endcase
        end
      end

      default: ; //default required by Quartus II 13.1
    endcase
 end
endmodule
