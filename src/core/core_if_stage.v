`timescale 1ns/1ps
`include "../defines.vh"


// Module Declaration
module if_stage 
    (
    clk,                               //Clock.
    brj_i,                             //Branch/jump control signal. Active high.
    brj_pc_i,                          //New address for branch/jump instructions.
    d_instruction_o,                   //Registered fetched instruction.
    d_pc_o,                            //Address of the fetched instruction. 
    d_pc4_o,                           //Address of the next instruction to fetch.
    flush_inst_o,
    instruction_addr_o,                //Address of instruction to fetch. To memory.
    instruction_rdata_i,               //Fetched instruction. From memory.
    rst_n,                             //Reset. Asynchronous active low.
    stall_i                           // Stall core one cycle flag.
//  stall_general_i                    // Stall core many cicles flag.
    );

    input  wire                        rst_n;
    input  wire                        clk;
    input  wire                        brj_i;
    input  wire [`DATA_WIDTH-1:0]      brj_pc_i;
    input  wire [`DATA_WIDTH-1:0]      instruction_rdata_i;
    output wire [`MEM_ADDR_INSTR_WIDTH-1:0]  instruction_addr_o;
    output wire [`DATA_WIDTH-1:0]      d_instruction_o;
    output reg  [`DATA_WIDTH-1:0]      d_pc_o;
    output reg  [`DATA_WIDTH-1:0]      d_pc4_o;
    input  wire                        stall_i;
//  input  wire                        stall_general_i;
    output wire                        flush_inst_o;
                
    wire [`DATA_WIDTH-1:0]             pc4;
    reg  [`DATA_WIDTH-1:0]             pc;
    reg  [`DATA_WIDTH-1:0]             dd_instruction;
    wire                               stall_any;
    reg                                stall_reg;   
    reg                                brj_reg;    
    reg                                core_init; // High during the first clock cycle after rst
                                                  // to avoid loading a wrong first instruction.
    
    assign instruction_addr_o = pc[`MEM_ADDR_INSTR_WIDTH-1:0];
    assign pc4  = pc + {{`DATA_WIDTH-3{1'b0}}, 3'd4};
    assign flush_inst_o = !brj_i;
    assign stall_any = stall_i; //| stall_general_i;

    //Auxiliars for Compressed Instrucction Set
    wire [1:0]                             op_1;
    wire [1:0]                             op_2;
    reg [(`DATA_WIDTH/2)-1:0]              save;
    reg [2:0]                              code;
    wire                                   pc_no_aligned;

    //STATES for acces to memory
    reg [3:0]                              state;
    localparam                             ALIGNED = 0;
    localparam                             UNALIGNED_TO_ALIGNED = 1;
    localparam                             UNALIGNED = 2;
    localparam                             IDLE = 3;
    localparam                             BRANCHED = 4;
        
    //Registered instruction and PC for pipeline
    always@(posedge clk or negedge rst_n)
        if (!rst_n) 
        begin
            d_pc_o <= {`DATA_WIDTH{1'b0}};
            d_pc4_o <= {`DATA_WIDTH{1'b0}};
            stall_reg <= 1'b0;
            brj_reg <= 1'b0;
            core_init <= 1'b1;
        end
        else 
        begin
            d_pc_o <= pc;
            d_pc4_o <= pc4;
            stall_reg <= stall_any;
            brj_reg <= brj_i;
            core_init <= 1'b0;
        end
           
   
// ORIGINAL CODE
//----------------------------------------------------------------------
 //The output of the program memory is registered!
/*
    assign d_instruction_o = brj_reg | core_init ? {25'b0, 7'b0010011}  : stall_reg ? dd_instruction : instruction_rdata_i;//flush_inst ? {25'b0, 7'b0010011} : instruction_rdata_i;
    //assign d_instruction_o = brj_reg | core_init ? {25'b0, 7'b0010011}  :  stall_reg ? dd_instruction : dd_instruction; //: instruction_rdata_i;//flush_inst ? {25'b0, 7'b0010011} : instruction_rdata_i;
    
    always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        dd_instruction <= {25'b0, 7'b0010011}; // NOOP
        end
    else if (!stall_reg) begin
        dd_instruction <= instruction_rdata_i;
        end 

    always @(posedge clk or negedge rst_n)
     if (!rst_n) pc <= {`DATA_WIDTH{1'b0}};
     else if (!stall_any) begin
                 pc <= brj_i ? brj_pc_i : pc4;
              end
    
*/
//VERSION 2.0
//-----------------------------------------------------------------------
//Adaptative program memory for unaligned memory WRONG: THE CASES NOT OCCUR IN ALL CONTEXTS, need to limitate
/*
    always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
                dd_instruction <= {25'b0, 7'b0010011}; // NOOP
                pc <= {`DATA_WIDTH{1'b0}};
                save <= {(`DATA_WIDTH/2){1'b0}};
                
                end
    else if (!stall_reg) 
    begin
        op_1 <= instruction_rdata_i[1:0];
        op_2 <= instruction_rdata_i[17:16];
        if ((op_1 == {2'b00} || op_1 == {2'b01} || op_1 == {2'b10}) && (op_2 == {2'b00} || op_2 == {2'b01} || op_2 == {2'b10}) && save == {(`DATA_WIDTH/2){1'b0}})
        begin
            dd_instruction <= {instruction_rdata_i[15:0], {(`DATA_WIDTH/2){1'b0}}};
            save <= instruction_rdata_i[31:16];
            pc <= brj_i ? brj_pc_i : pc;
        end
        if((op_1 == {2'b00} || op_1 == {2'b01} || op_1 == {2'b10}) && (op_2 == {2'b00} || op_2 == {2'b01} || op_2 == {2'b10}) && save != {(`DATA_WIDTH/2){1'b0}})
        begin
            dd_instruction <= {save, {(`DATA_WIDTH/2){1'b0}}};
            save <= {(`DATA_WIDTH/2){1'b0}};
            pc <= brj_i ? brj_pc_i : pc4;
        end
        if ((op_1 == {2'b00} || op_1 == {2'b01} || op_1 == {2'b10}) && (op_2 != {2'b00} || op_2 != {2'b01} || op_2 != {2'b10}))
        begin
            dd_instruction <= {instruction_rdata_i[15:0], {(`DATA_WIDTH/2){1'b0}}};
            save <= instruction_rdata_i[31:16];
            pc <= brj_i ? brj_pc_i : pc4;
        end
        if((op_1 != {2'b00} || op_1 != {2'b01} || op_1 != {2'b10}) && (op_2 != {2'b00} || op_2 != {2'b01} || op_2 != {2'b10}) && save == {(`DATA_WIDTH/2){1'b0}})
        begin
            dd_instruction <= instruction_rdata_i[31:0];
            save <= {(`DATA_WIDTH/2){1'b0}};
            pc <= brj_i ? brj_pc_i : pc4;
        end
        if((op_1 != {2'b00} || op_1 != {2'b01} || op_1 != {2'b10}) && (op_2 != {2'b00} || op_2 != {2'b01} || op_2 != {2'b10}) && save != {(`DATA_WIDTH/2){1'b0}})
        begin
            dd_instruction <= {save,instruction_rdata_i[15:0]};
            save <= instruction_rdata_i[31:16];
            pc <= brj_i ? brj_pc_i : pc4;
        end
        if((op_1 != {2'b00} || op_1 != {2'b01} || op_1 != {2'b10}) && (op_2 == {2'b00} || op_2 == {2'b01} || op_2 == {2'b10}) && save != {(`DATA_WIDTH/2){1'b0}}) 
        begin
            dd_instruction <= {save,instruction_rdata_i[15:0]};
            save <= {(`DATA_WIDTH/2){1'b0}};
            pc <= brj_i ? brj_pc_i : pc;
        end
        if((op_1 != {2'b00} || op_1 != {2'b01} || op_1 != {2'b10}) && (op_2 != {2'b00} || op_2 != {2'b01} || op_2 != {2'b10}) && save!= {(`DATA_WIDTH/2){1'b0}}) 
        begin
            dd_instruction <= {instruction_rdata_i[31:16], {(`DATA_WIDTH/2){1'b0}}};
            save <= {(`DATA_WIDTH/2){1'b0}};
            pc <= brj_i ? brj_pc_i : pc4;
        end
    end 
    */
//VERSION 2.1
//-----------------------------------------------------------------------
//Adaptative program memory for unaligned memory

    /*
    assign d_instruction_o = brj_reg | core_init ? {25'b0, 7'b0010011}  :  stall_reg ? dd_instruction : code == 3'b001 ? instruction_rdata_i:
                                                                                                        code == 3'b010 | code == 3'b011 & save!={(`DATA_WIDTH/2){1'b0}}? {{(`DATA_WIDTH/2){1'b0}},instruction_rdata_i[15:0]}:
                                                                                                        code == 3'b100 & save!={(`DATA_WIDTH/2){1'b0}}? {{(`DATA_WIDTH/2){1'b0}}, save}:
                                                                                                        code == 3'b101 | code == 3'b110 & save!={(`DATA_WIDTH/2){1'b0}}? {instruction_rdata_i[15:0], save}: {25'b0, 7'b0010011};//flush_inst ? {25'b0, 7'b0010011} : instruction_rdata_i;
    */   
    
    assign d_instruction_o = brj_reg | core_init ? {25'b0, 7'b0010011}  :   dd_instruction;                                                                                            
    assign    op_1 = instruction_rdata_i[1:0];
    assign    op_2 = instruction_rdata_i[17:16];

    always @(posedge clk or negedge rst_n)
    if (!rst_n) 
    begin
        dd_instruction = {25'b0, 7'b0010011}; // NOOP
        pc <= {`DATA_WIDTH{1'b0}};
        save <= {(`DATA_WIDTH/2){1'b0}};
        state = IDLE; 
        code <= 3'b111;        
    end
    else if (!stall_reg) 
    begin
        case (state)
            //STATE 11: 
            IDLE:  
                begin
                    //$display("IDLE");
                    dd_instruction = {25'b0, 7'b0010011}; // NOOP
                    save <= {(`DATA_WIDTH/2){1'b0}};
                    pc <= brj_i ? brj_pc_i : pc4;
                    state <= ALIGNED;
                    code <= 3'b000; 
                end
            //STATE 00: Memory Aligned
            ALIGNED:  
                begin
                    if (op_1 == 2'b11) // 32b Instruction read. Then, memory aligned.
                    begin
                        //$display("32b Instr");
                        dd_instruction = instruction_rdata_i;
                        save <= {(`DATA_WIDTH/2){1'b0}};
                        pc <= brj_i ? brj_pc_i : pc4;
                        
                        state <= ALIGNED;
                        code <= 3'b001;

                    end
                    else 
                    begin
                        //$display("NO 32b Instr");
                        if (op_2 == 2'b11) // 16b Instruction + 32b/2 Instruction read. Then, memory unaligned.
                        begin
                            //$display("NO 32b Instr + UNALIGNED");
                            dd_instruction = {{(`DATA_WIDTH/2){1'b0}},instruction_rdata_i[15:0]};
                            save <= instruction_rdata_i[31:16];
                            pc <= brj_i ? brj_pc_i : pc4;
                            state <= UNALIGNED; 
                            code <= 3'b010;
                        end
                        else // 16b Instruction + 16b Instruction read. Then, memory unaligned.
                        begin
                            //$display("NO 32b Instr + (16 16) UNALIGNED");
                            dd_instruction = {{(`DATA_WIDTH/2){1'b0}},instruction_rdata_i[15:0]};
                            save <= instruction_rdata_i[31:16];
                            pc <= brj_i ? brj_pc_i : pc;
                            state <= UNALIGNED_TO_ALIGNED; 
                            code <= 3'b011;
                        end
                    end
                end
            //STATE 01: Memory Unaligned, Compressed instruction already read in last cicle, must send it to ID_stage and add PC. Then, Memory Aligned again.
            UNALIGNED_TO_ALIGNED:  
                begin
                    //$display("16b Instr + ALIGNED");
                    dd_instruction = {{(`DATA_WIDTH/2){1'b0}}, save};
                    save <= {(`DATA_WIDTH/2){1'b0}};
                    pc <= brj_i ? brj_pc_i : pc4;
                    state <= ALIGNED; 
                    code <= 3'b100;
                end
            //STATE 10: Memory Unaligned, part of a 32b instruction already read.
            UNALIGNED:  
                begin
                    if (op_2 == 2'b11)
                    begin
                        //$display("NO 32b Instr + 32 UNALIGNED");
                        dd_instruction = {instruction_rdata_i[15:0], save};
                        save <= instruction_rdata_i[31:16];
                        pc <= brj_i ? brj_pc_i : pc4;
                        state <= UNALIGNED;
                        code <= 3'b101;
                    end
                    else
                    begin
                        //$display(" 32b Instr + 16 GO TO ALIGNED");
                        dd_instruction = {instruction_rdata_i[15:0], save};
                        save <= instruction_rdata_i[31:16];
                        pc <= brj_i ? brj_pc_i : pc;
                        state <= UNALIGNED_TO_ALIGNED; 
                        code <= 3'b110;
                    end
                end
        endcase
         //state <= next_state;
         //$display("STATE: %h", state);
         //$display("instr: %h", instruction_rdata_i); 
         //$display("instr: %h", dd_instruction); 
    end
    
//VERSION 2.2
//-----------------------------------------------------------------------
//Adapted for brach instructions and PC unaligned
/*
    assign d_instruction_o = brj_reg | core_init ? {25'b0, 7'b0010011}  :   dd_instruction;                                                                                            
    assign    op_1 = instruction_rdata_i[1:0];
    assign    op_2 = instruction_rdata_i[17:16];

    assign    pc_no_aligned = (brj_pc_i % 4) == 0? 0:1;


    always @(posedge clk or negedge rst_n)
    if (!rst_n) 
        begin
            dd_instruction = {25'b0, 7'b0010011}; // NOOP
            pc <= {`DATA_WIDTH{1'b0}};
            save <= {(`DATA_WIDTH/2){1'b0}};
            state = IDLE; 
        end
    else if (!stall_reg & !brj_i) 
    begin
        case (state)
            //STATE 11: 
            IDLE:  
                begin
                    //$display("IDLE");
                    dd_instruction = {25'b0, 7'b0010011}; // NOOP
                    save <= {(`DATA_WIDTH/2){1'b0}};
                    pc <= brj_i ? brj_pc_i : pc4;
                    state <= brj_i & pc_no_aligned? UNALIGNED : ALIGNED;
                end
            //STATE 00: Memory Aligned
            ALIGNED:  
                begin
                    if (op_1 == 2'b11) // 32b Instruction read. Then, memory aligned.
                    begin
                        //$display("32b Instr");
                        dd_instruction = instruction_rdata_i;
                        save <= {(`DATA_WIDTH/2){1'b0}};
                        pc <= brj_i ? brj_pc_i : pc4;
                        state <= brj_i & pc_no_aligned? BRANCHED : ALIGNED;

                    end
                    else 
                    begin
                        //$display("NO 32b Instr");
                        if (op_2 == 2'b11) // 16b Instruction + 32b/2 Instruction read. Then, memory unaligned.
                        begin
                            //$display("NO 32b Instr + UNALIGNED");
                            dd_instruction = {{(`DATA_WIDTH/2){1'b0}},instruction_rdata_i[15:0]};
                            save <= instruction_rdata_i[31:16];
                            pc <= brj_i ? brj_pc_i : pc4;
                            state <= brj_i & pc_no_aligned? BRANCHED : (brj_i? ALIGNED :UNALIGNED);
                        end
                        else // 16b Instruction + 16b Instruction read. Then, memory unaligned.
                        begin
                            //$display("NO 32b Instr + (16 16) UNALIGNED");
                            dd_instruction = {{(`DATA_WIDTH/2){1'b0}},instruction_rdata_i[15:0]};
                            save <= instruction_rdata_i[31:16];
                            pc <= brj_i ? brj_pc_i : pc;
                            state <= brj_i & pc_no_aligned? BRANCHED : (brj_i? ALIGNED :UNALIGNED_TO_ALIGNED);
                        end
                    end
                end
            //STATE 01: Memory Unaligned, Compressed instruction already read in last cicle, must send it to ID_stage and add PC. Then, Memory Aligned again.
            UNALIGNED_TO_ALIGNED:  
                begin
                    //$display("16b Instr + ALIGNED");
                    dd_instruction = {{(`DATA_WIDTH/2){1'b0}}, save};
                    save <= {(`DATA_WIDTH/2){1'b0}};
                    pc <= brj_i ? brj_pc_i : pc4;
                    state <= brj_i & pc_no_aligned? BRANCHED : ALIGNED; 
                end
            //STATE 10: Memory Unaligned, part of a 32b instruction already read.
            UNALIGNED:  
                begin
                    if (op_2 == 2'b11)
                    begin
                        //$display("NO 32b Instr + 32 UNALIGNED");
                        dd_instruction = {instruction_rdata_i[15:0], save};
                        save <= instruction_rdata_i[31:16];
                        pc <= brj_i ? brj_pc_i : pc4;
                        state <= brj_i & pc_no_aligned? BRANCHED : (brj_i? ALIGNED :UNALIGNED);
                    end
                    else
                    begin
                        //$display(" 32b Instr + 16 GO TO ALIGNED");
                        dd_instruction = {instruction_rdata_i[15:0], save};
                        save <= instruction_rdata_i[31:16];
                        pc <= brj_i ? brj_pc_i : pc;
                        state <= brj_i & pc_no_aligned? BRANCHED : (brj_i? ALIGNED :UNALIGNED_TO_ALIGNED); 
                    end
                end
            BRANCHED:
                begin
                    if (op_2 == 2'b11)
                    begin
                        //$display("32 UNALIGNED");
                        dd_instruction = {25'b0, 7'b0010011}; // NOOP;
                        save <= instruction_rdata_i[31:16];
                        pc <= brj_i ? brj_pc_i : pc + {{`DATA_WIDTH-3{1'b0}}, 3'd2};
                        state <= UNALIGNED;
                    end
                    else
                    begin
                        //$display(" 16 GO TO ALIGNED");
                        dd_instruction = {{(`DATA_WIDTH/2){1'b0}}, instruction_rdata_i[31:16]};
                        save <= {(`DATA_WIDTH/2){1'b0}};
                        pc <= brj_i ? brj_pc_i : pc + {{`DATA_WIDTH-3{1'b0}}, 3'd2};
                        state <= ALIGNED; 
                    end
                end
        endcase
    end
  */
endmodule 