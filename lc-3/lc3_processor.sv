/* 
*
*   A fully-functioning LC-3 processor modeled in SystemVerilog.
*   LC-3 (Little Computer 3) is a relatively simple instruction set architecture built by professors Sanjay J. Patel (UIUC) and Yale N. Patt (UT-Austin).
*   For more info: https://en.wikipedia.org/wiki/Little_Computer_3
*   
*   @author Ryan Xia, ryanxia2@illinois.edu
*   @ver 1.0, 1/3/2026
*
*/

//--top module, represents one core of an LC-3 processor--
module lc3_processor (
    input logic clk,
    input logic reset
);

    //--memory registers--
    logic [15:0] mar, mdr;

    //--datapath control signals--
    logic ld_ben, ld_mar, ld_mdr, ld_ir, ld_pc, ld_reg, ld_cc;
    logic gate_marmux, gate_mdr, gate_alu, gate_pc;
    logic marmux, addr1mux;
    logic [1:0] pcmux, addr2mux, drmux, sr1mux;
    logic [1:0] aluk;
    logic mio_en, r_w;

    //--status signals--
    logic [15:0] ir;                //MOVE TO DATAPATH
    logic ird, ben, mem_r, irpt;

    //--instantiate control unit--
    ctrl_unit lc3control (
        //--input--
        .clk(clk), .reset(reset),
        .ir(ir), .ird(ird), .ben(ben), .mem_r(mem_r), .irpt(irpt),

        //--output (to datapath)--
        .ld_ben(ld_ben), .ld_mar(ld_mar), .ld_mdr(ld_mdr), .ld_ir(ld_ir), .ld_pc(ld_pc), .ld_reg(ld_reg), .ld_cc(ld_cc),
        .gate_marmux(gate_marmux), .gate_mdr(gate_mdr), .gate_alu(gate_alu), .gate_pc(gate_pc),
        .marmux(marmux), .addr1mux(addr1mux), .pcmux(pcmux), .addr2mux(addr2mux), .drmux(drmux), .sr1mux(sr1mux),
        .aluk(aluk), .mio_en(mio_en), .r_w(r_w)
    );

    //--instantiate memory--
    memory lc3memory (
        //--input--
        .clk(clk),
        .reset(reset),
        .mio_en(mio_en), .r_w(r_w), .addr(mar), .data_in(mdr),

        //--output--
        .read_data(mdr), .mem_r(mem_r)
    );

endmodule


//--main data path--
module data_path (
    input logic clk, reset,
    input logic ld_ben, ld_mar, ld_mdr, ld_ir, ld_pc, ld_reg, ld_cc,
    input logic gate_marmux, gate_mdr, gate_alu, gate_pc,
    input logic marmux, addr1mux,
    input logic [1:0] pcmux, addr2mux, drmux, sr1mux,
    input logic [1:0] aluk,
    input logic mio_en, r_w
);
    //--bus--
    logic [15:0] main_bus;

    //--register immediate outputs--
    logic [15:0] ir_out, pc_out, mdr_out, mar_out, sr1out, sr2out;

    //--instantiating the IR--
    ir lc3_ir (
        .clk(clk), .reset(reset), .bus(main_bus), .ld(ld_ir),
        .ir_out(ir_out)
    );

    //--instantiating the PC--
    pc lc3_pc (
        .clk(clk), .reset(reset), .pc_in(pcmux_o), .ld(ld_pc),
        .pc_out(pc_out)
    );

    //--instantiating the MAR--
    mar lc3_mar (
        .clk(clk), .reset(reset), .bus(main_bus), .ld(ld_mar),
        .mar_out(mar_out)
    );

    //--instantiating the MDR--
    mdr lc3_mdr (
        .clk(clk), .reset(reset), .mdr_in(mioen_o), .ld(ld_mdr), 
        .mdr_out(mdr_out)
    )

    //--instantiating the reg file--
    reg_file lc3_regfile (
        .clk(clk), .reset(reset), .bus(main_bus), .dr(drmux_o), .sr1(sr1mux_o), .sr2(ir_out[2:0]), .ld(ld_reg),
        .sr1out(sr1out), .sr2out(sr2out)
    ); 
endmodule


//--control unit fsm--
module ctrl_unit (
    input logic clk, reset,
    input logic [15:0] ir,
    input logic ird, ben, mem_r, irpt,

    output logic ld_ben, ld_mar, ld_mdr, ld_ir, ld_pc, ld_reg, ld_cc,
    output logic gate_marmux, gate_mdr, gate_alu, gate_pc,
    output logic marmux, addr1mux,
    output logic [1:0] pcmux, addr2mux, drmux, sr1mux,
    output logic [1:0] aluk,
    output logic mio_en, r_w
);

    logic [24:0] rom_ctrl_signals;
    logic [2:0] rom_cond;
    logic [5:0] curr_state, next_state, next_state_imm;

    always_ff @(posedge clk) begin
        if(reset)
            curr_state <= 6'd18;                       //reset goes to fetch
        else
            curr_state <= next_state;
    end

    //--generate set of conditions for microsequencer--
    always_comb begin
        if(mem_r & next_state_imm[1])
            rom_cond = 3'b001;
        else if(ben & next_state_imm[2])
            rom_cond = 3'b010;
        else if(ir[11] & next_state_imm[0])
            rom_cond = 3'b011;
        else
            rom_cond = 3'b000;
    end

    //--rom_ctrl_signals for each state--
    always_comb begin
        case(curr_state)
            6'd 18: begin        //fetch 1 MAR <- PC, PC <- PC + 1
                rom_ctrl_signals = 25'b 0100100000100000000000000;
                next_state_imm = (irpt) ? 6'd 49 : 6'd 33;
            end
            6'd 33: begin        //fetch 2 MDR <- M[MAR]
                rom_ctrl_signals = 25'b 0010000000000000000000010;
                next_state_imm = 6'd 33;
            end
            6'd 35: begin        //fetch 3 IR <- MDR
                rom_ctrl_signals = 25'b 0001000010000000000000000;
                next_state_imm = 6'd 32;
            end
            6'd 32: begin        //decode
                rom_ctrl_signals = 25'b 1000000000000000000000000;
                next_state_imm = 6'd 18;
            end
            6'd 1: begin         //ADD : DR <- SR1 + OP2, setCC 
                rom_ctrl_signals = 25'b 0000011001000000000010000;
                next_state_imm = 6'd 18;
            end
            6'd 5: begin         //AND : DR <- SR1 & OP2, setCC
                rom_ctrl_signals = 25'b 0000011001000000000010100;
                next_state_imm = 6'd 18;
            end
            6'd 9: begin         //NOT : DR <- NOT(SR), setCC
                rom_ctrl_signals = 25'b 0000011001000000000011000;
                next_state_imm = 6'd 18;
            end
            6'd 14: begin        //LEA : DR <- PC + off9, setCC
                rom_ctrl_signals = 25'b 0000010100010001000000000;
                next_state_imm = 6'd 18;
            end
            6'd 2: begin         //LD : MAR <- PC + off9
                rom_ctrl_signals = 25'b 0100000000101001000000000;
                next_state_imm = 6'd 25;
            end
            6'd 6: begin         //LDR : MAR <- BaseR + off6
                rom_ctrl_signals = 25'b 0100000100010010100000000;
                next_state_imm = 6'd 25;
            end
            6'd 10: begin        //LDI : MAR <- PC + off9
                rom_ctrl_signals = 25'b 0100000000101001000000000;
                next_state_imm = 6'd 24;
            end
            6'd 11: begin        //STI : MAR <- PC + off9
                rom_ctrl_signals = 25'b 0100000000101001000000000;
                next_state_imm = 6'd 29;
            end
            6'd 7: begin         //STR : MAR <- BaseR + off6
                rom_ctrl_signals = 25'b 0100000100010010100000000;
                next_state_imm = 6'd 23;
            end
            6'd 3: begin         //ST : MAR <- PC + off9
                rom_ctrl_signals = 25'b 0100000000101001000000000;
                next_state_imm = 6'd 23;
            end
            6'd 4: begin         //JSR : R7 <- PC , IR[11]
                rom_ctrl_signals = 25'b 0000010000100000001000000;
                next_state_imm = (ir[11]) ? 6'd 21 : 6'd 20;
            end
            6'd 12: begin        //JMP : PC <- BaseR
                rom_ctrl_signals = 25'b 0000100001000100000011100;
                next_state_imm = 6'd 18;
            end
            6'd 0: begin         //BR : [BEN]
                next_state_imm = (ben) ? 6'd 22 : 6'd 18;
            end
            6'd 28: begin        //MDR <- M[MAR], R7 <- PC, after 15
                rom_ctrl_signals = 25'b 0010010000100000001000010;
                next_state_imm = 6'd 28;
            end
            6'd 30: begin        //PC <- MDR, after 28
                rom_ctrl_signals = 25'b 0000100010000100000000000;
                next_state_imm = 6'd 18;
            end
            6'd 24: begin        //MDR <- M[MAR], after 10
                rom_ctrl_signals = 25'b 0010000000000000000000010;
                next_state_imm = 6'd 24;
            end
            6'd 29: begin        //MDR <- M[MAR], after 11
                rom_ctrl_signals = 25'b 0010000000000000000000010;
                next_state_imm = 6'd 29;
            end
            6'd 26: begin        //MAR <- MDR, after 24
                rom_ctrl_signals = 25'b 0100000010000000000000000;
                next_state_imm = 6'd 25;
            end
            6'd 31: begin        //MAR <- MDR, after 29
                rom_ctrl_signals = 25'b 0100000010000000000000000;
                next_state_imm = 6'd 23;
            end
            6'd 25: begin        //MDR <- M[MAR], after 2, 6, 26
                rom_ctrl_signals = 25'b 0010000000000000000000010;
                next_state_imm = 6'd 25;
            end
            6'd 27: begin        //DR <- MDR setCC, after 25
                rom_ctrl_signals = 25'b 0000011010000000000000000;
                next_state_imm = 6'd 18;
            end
            6'd 23: begin        //MDR <- SR, after 3, 7, 31
                rom_ctrl_signals = 25'b 0010000001000000000001100;
                next_state_imm = 6'd 16;
            end
            6'd 16: begin        //M[MAR] <- MDR, after 23
                rom_ctrl_signals = 25'b 0000000000000000000000011;
                next_state_imm = 6'd 16;
            end
            6'd 21: begin        //PC <- PC + off11, after 4
                rom_ctrl_signals = 25'b 0000100000001001100000000;
                next_state_imm = 6'd 18;
            end
            6'd 20: begin        //PC <- BaseR, after 4
                rom_ctrl_signals = 25'b 0000100001000100000001100;
                next_state_imm = 6'd 18;
            end
            6'd 22: begin        //PC <- PC + off9, after 0
                rom_ctrl_signals = 25'b 0000100000001001000000000;
                next_state_imm = 6'd 18;
            end
            default : rom_ctrl_signals = 25'b0;             //undefined state
        endcase
    end

    //--instantiate microsequencer--
    microsequencer ctrl_microsequencer (
        .car(next_state_imm),
        .cond(rom_cond),
        .ir(ir),
        .ben(ben),
        .r(mem_r),
        .ird(ird),

        .j(next_state)                            //produces next_state from next_state_imm given conditions
    );

    //--assign outputs--
    assign ld_ben = rom_ctrl_signals[24];         //register load signals
    assign ld_mar = rom_ctrl_signals[23];
    assign ld_mdr = rom_ctrl_signals[22];
    assign ld_ir = rom_ctrl_signals[21];
    assign ld_pc = rom_ctrl_signals[20];
    assign ld_reg = rom_ctrl_signals[19];
    assign ld_cc = rom_ctrl_signals[18];

    assign gate_marmux = rom_ctrl_signals[17];    //bus driver signals
    assign gate_mdr = rom_ctrl_signals[16];
    assign gate_alu = rom_ctrl_signals[15];
    assign gate_pc = rom_ctrl_signals[14];

    assign marmux = rom_ctrl_signals[13];         //mux select signals
    assign pcmux = rom_ctrl_signals[12:11];
    assign addr1mux = rom_ctrl_signals[10];
    assign addr2mux = rom_ctrl_signals[9:8];
    assign drmux = rom_ctrl_signals[7:6];
    assign sr1mux = rom_ctrl_signals[5:4];

    assign aluk = rom_ctrl_signals[3:2];          //alu operation select

    assign mio_en = rom_ctrl_signals[1];          //memory signals
    assign r_w = rom_ctrl_signals[0];

endmodule


//--microsequencer (next state logic)--
module microsequencer (
    input logic [5:0] car,
    input logic [2:0] cond,
    input logic [15:0] ir,
    input logic ben, r, ird,

    output logic [5:0] j
);
    logic [7:0] y;
    //--handling combinational logic for next state based on conditions (j)-- 
    always_comb begin
        case(cond)
            3'b011 : y = 8'b00001000;
            3'b010 : y = 8'b00000100;
            3'b001 : y = 8'b00000010;
            default : y = 8'b00000000;
        endcase
        case(ird)
            1: j = {2'b00, ir[15:12]};
            0: j = {car[5:3], 
            (ben & y[2]) | car[2], 
            (r & y[1]) | car[1], 
            (ir[11] & y[3]) | car[0]};
        endcase
    end
endmodule


//--instruction register--
module ir (
    input clk, reset,
    input logic [15:0] bus,
    input logic ld,
    
    output logic [15:0] ir_out
);
    logic [15:0] instruction_register;
    //--synchronous load--
    always_ff @(posedge clk) begin
        if(reset) begin
            instruction_register <= 16'h0000;       //defaults to branching 0 from x3000 (nothing changes)
        end
        else if(ld) begin
            instruction_register <= bus;
        end
    end
    //--asynchronous ir_out--
    always_comb begin
        ir_out = instruction_register;
    end
endmodule


//--program counter--
module pc (
    input logic clk, reset,
    input logic [15:0] pc_in,
    input logic ld,

    output logic [15:0] pc_out
);
    logic [15:0] program_counter;
    //--synchronous load--
    always_ff @(posedge clk) begin
        if(reset) begin
            program_counter <= 16'h3000;                    //default reset to x3000
        end
        else if(ld) begin
            program_counter <= pc_in;
        end
    end
    //--asynchronous pc_out--
    always_comb begin
        pc_out = program_counter;
    end
endmodule


//--memory address register--
module mar (
    input logic clk, reset,
    input logic [15:0] bus,
    input logic ld,

    output logic [15:0] mar_out
);
    logic [15:0] memory_address_register;
    //--synchronous load--
    always_ff @(posedge clk) begin
        if(reset) begin
            memory_address_register <= 0;                    
        end
        else if(ld) begin
            memory_address_register <=  bus;
        end
    end
    //--asynchronous mar_out--
    always_comb begin
        mar_out = memory_address_register;
    end
endmodule


//--memory data register--
module mdr (
    input logic clk, reset,
    input logic [15:0] mdr_in,
    input logic ld,

    output logic [15:0] mdr_out
);
    logic [15:0] memory_data_register;
    //--synchronous load--
    always_ff @(posedge clk) begin
        if(reset) begin
            memory_data_register <= 0;                    
        end
        else if(ld) begin
            memory_data_register <=  mdr_in;
        end
    end
    //--asynchronous mdr_out--
    always_comb begin
        mdr_out = memory_data_register;
    end
endmodule


//--register file with registers R0 through R7 used in LC-3--
module reg_file (
    input logic clk, reset,
    input logic [15:0] bus,
    input logic [2:0] dr, sr1, sr2,
    input logic ld,

    output logic [15:0] sr1out, sr2out
);
    logic [15:0] regs [0:7];                    // these are R0 through R7
    //--synchronous load--
    always_ff @(posedge clk) begin
        if(reset) begin
            for(int i = 0; i < 8; i++) begin
                regs[i] <= 0;
            end
        end
        else begin
            if(ld) begin
                regs[dr] <= bus;
            end
        end
    end
    //--asynchronous sr1 and sr2 outputs--
    always_comb begin
        sr1out = regs[sr1];
        sr2out = regs[sr2];
    end
endmodule


//--arithmetic and logic unit (supports add, and, not, and pass)--
module alu (
    input logic [15:0] a, b, 
    input logic [1:0] aluk,

    output logic [15:0] alu_out
);
    always_comb begin
        case(aluk)
            2'b00: alu_out = a + b;
            2'b01: alu_out = a & b;
            2'b10: alu_out = ~a;
            2'b11: alu_out = a;
        endcase
    end
endmodule


//--memory module for the LC-3, featuring 16-bit addressibility and 2^16 address space [0:xFFFF]--
module memory (
    input logic clk, reset, mio_en, r_w,
    input logic [15:0] addr, data_in,
    output logic [15:0] read_data,
    output logic mem_r
);
    logic [15:0] main_memory [0:65535];

    always_ff @(posedge clk) begin
        if(reset) begin
            read_data <= 0;
            mem_r <= 0;
        end
        else if(mio_en) begin
            if(r_w) begin
                main_memory[addr] <= data_in;
                mem_r <= 1;
            end
            else begin
                read_data <= main_memory[addr];
                mem_r <= 1;
            end
        end
        else
            mem_r <= 0;
    end
endmodule


    



