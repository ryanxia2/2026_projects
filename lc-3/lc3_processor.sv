/* 
*
*   A fully-functioning LC-3 processor modeled in SystemVerilog.
*   LC-3 (Little Computer 3) is a relatively simple instruction set architecture built by professors Sanjay J. Patel (UIUC) and Yale N. Patt (UT-Austin).
*   For more info: https://en.wikipedia.org/wiki/Little_Computer_3
*   
*   @author Ryan Xia, ryanxia2@illinois.edu
*   @ver prototype1.0, 1/11/2026
*
*/

//--top module, represents one core of an LC-3 processor--


interface lc3io (
    input logic clk,
    input logic reset
);
    //--control signals (controller -> datapath)--
    logic ld_ben, ld_mar, ld_mdr, ld_ir, ld_pc, ld_reg, ld_cc;
    logic gate_marmux, gate_mdr, gate_alu, gate_pc;
    logic marmux, addr1mux;
    logic [1:0] pcmux, addr2mux, drmux, sr1mux;
    logic [1:0] aluk;
    logic mio_en, r_w;

    //--status signals (datapath -> controller)--
    logic [15:0] ir;
    logic ben;
    logic ird; 

    modport controller (
        output ld_ben, ld_mar, ld_mdr, ld_ir, ld_pc, ld_reg, ld_cc,
            gate_marmux, gate_mdr, gate_alu, gate_pc,
            marmux, addr1mux, pcmux, addr2mux, drmux, sr1mux,
            aluk, mio_en, r_w,
        input  ir, ben, ird, clk, reset
    );

    modport datapath (
        input  ld_ben, ld_mar, ld_mdr, ld_ir, ld_pc, ld_reg, ld_cc,
            gate_marmux, gate_mdr, gate_alu, gate_pc,
            marmux, addr1mux, pcmux, addr2mux, drmux, sr1mux,
            aluk, mio_en, r_w,
        output ir, ben,
        input  clk, reset
    );
endinterface


module lc3_processor (
    input logic clk,
    input logic reset
);
    lc3io io (clk, reset);

    logic [15:0] mar_wire;   
    logic [15:0] mdr_wire;   
    logic [15:0] mem_data_out;  
    logic mem_r, irpt;
    
    //--instantiate control unit--
    ctrl_unit lc3control (
        .io(io.controller),
        .mem_r(mem_r), .irpt(irpt)
    );

    //--instantiate datapath--
    data_path lc3datapath (
        .io(io.datapath),
        .mem_out(mem_data_out),
        .mar_to_mem(mar_wire),
        .mdr_to_mem(mdr_wire)
    );

    //--instantiate memory--
    memory lc3memory (
        //--input--
        .clk(clk),
        .reset(reset),
        .mio_en(io.mio_en), .r_w(io.r_w), .addr(mar_wire), .data_in(mdr_wire),

        //--output--
        .read_data(mem_data_out), .mem_r(mem_r)
    );

endmodule


//--main data path--
module data_path (
    lc3io.datapath io,
    input logic [15:0] mem_out;
    
    output logic [15:0] mar_to_mem,   
    output logic [15:0] mdr_to_mem 
);
    //--bus--
    wire [15:0] main_bus;

    //--status signal CC--
    logic [2:0] nzp;

    //--internal wires--
    logic [15:0] pc_out, mdr_out, mar_out, pcmux_out, marmux_out, addr1mux_out, addr2mux_out, sr1out, sr2out, alu_out, adder_out, sr2mux_out, miomux_out;
    logic [2:0] drmux_out, sr1mux_out;

    //--instantiating the IR--
    ir lc3_ir (
        .clk(io.clk), .reset(io.reset), .bus(main_bus), .ld(io.ld_ir),
        .ir_out(io.ir)
    );

    //--instantiating the PC--
    pc lc3_pc (
        .clk(io.clk), .reset(io.reset), .pc_in(pcmux_out), .ld(io.ld_pc),
        .pc_out(pc_out)
    );

    //--instantiating the CC--
    cc lc3_cc (
        .clk(io.clk), .reset(io.reset), .bus(main_bus), .ld(io.ld_cc),
        .stat_cc(nzp)
    );

    //--instantiating the MAR--
    mar lc3_mar (
        .clk(io.clk), .reset(io.reset), .bus(main_bus), .ld(io.ld_mar),
        .mar_out(mar_to_mem)
    );

    //--instantiating the MDR--
    mdr lc3_mdr (
        .clk(io.clk), .reset(io.reset), .mdr_in(miomux_out), .ld(io.ld_mdr), 
        .mdr_out(mdr_to_mem)
    );

    //--instantiating the reg file--
    reg_file lc3_regfile (
        .clk(io.clk), .reset(io.reset), .bus(main_bus), .dr(drmux_out), .sr1(sr1mux_out), .sr2(io.ir[2:0]), .ld(io.ld_reg),
        .sr1out(sr1out), .sr2out(sr2out)
    ); 

    //--instantiating the alu--
    alu lc3_alu (
        .a(sr1out), .b(sr2mux_out), .aluk(io.aluk),
        .alu_out(alu_out)
    );

    //--MUXES--

    //--marmux--
    always_comb begin
        case (io.marmux)
            1'b0: marmux_out = {8'b00000000, io.ir[7:0]};               //zero-extended
            1'b1: marmux_out = adder_out;
        endcase
    end

    //--addr1mux--
    always_comb begin
        case (io.addr1mux)
            1'b0: addr1mux_out = pc_out;
            1'b1: addr1mux_out = sr1out;
        endcase
    end

    //--addr2mux--
    always_comb begin
        case (io.addr2mux)
            2'b00: addr2mux_out = 0;
            2'b01: addr2mux_out = {{10{io.ir[5]}}, io.ir[5:0]};
            2'b10: addr2mux_out = {{7{io.ir[8]}}, io.ir[8:0]};
            2'b11: addr2mux_out = {{5{io.ir[10]}}, io.ir[10:0]};
        endcase
    end

    //--big adder--
    assign adder_out = addr1mux_out + addr2mux_out;

    //--pcmux--
    always_comb begin
        case (io.pcmux) 
            2'b00: pcmux_out = pc_out + 1;
            2'b01: pcmux_out = main_bus;
            2'b10: pcmux_out = adder_out;
            default: pcmux_out = 0;
        endcase
    end

    //--drmux--
    always_comb begin
        case(io.drmux)
            2'b00: drmux_out = io.ir[11:9];
            2'b01: drmux_out = 3'b111;
            2'b10: drmux_out = 3'b110;
            default: drmux_out = 0;
        endcase
    end

    //--sr1mux--
    always_comb begin
        case(io.sr1mux)
            2'b00: sr1mux_out = io.ir[11:9];
            2'b01: sr1mux_out = io.ir[8:6];
            2'b10: sr1mux_out = 3'b110;
            default: sr1mux_out = 0;
        endcase
    end

    //--sr2mux--
    always_comb begin
        case(io.ir[5])
            1'b0: sr2mux_out = sr2out;
            1'b1: sr2mux_out = {{11{io.ir[4]}}, io.ir[10:0]};
        endcase
    end

    //--mio_en--
    always_comb begin
        case(io.mio_en)
            1'b0: miomux_out = main_bus;
            1'b1: miomux_out = mem_out;
        endcase
    end

    //--clock synchronous logic for branch enable signal (BEN)--
    always_ff @(posedge clk) begin
        if(reset) begin
            io.ben <= 0;
        end
        else if(io.ld_ben) begin
            io.ben <= (io.ir[11] & nzp[2]) | (io.ir[10] & nzp[1]) | (io.ir[9] & nzp[0]);
        end
    end

    //--gate design with tri-state buffers--
    assign main_bus = (io.gate_alu) ? alu_out : 16'bz;
    assign main_bus = (io.gate_marmux) ? marmux_out : 16'bz;
    assign main_bus = (io.gate_mdr) ? mdr_out : 16'bz;
    assign main_bus = (io.gate_pc) ? pc_out : 16'bz;

endmodule


//--control unit fsm--
module ctrl_unit (
    lc3io.controller io,

    input logic mem_r,
    input logic irpt
);

    logic [24:0] rom_ctrl_signals;
    logic [2:0] rom_cond;
    logic [5:0] curr_state, next_state, next_state_imm;

    always_ff @(posedge io.clk) begin
        if(io.reset)
            curr_state <= 6'd18;                       //reset goes to fetch
        else
            curr_state <= next_state;
    end

    //--generate set of conditions for microsequencer--
    always_comb begin
        if(mem_r & ~next_state_imm[1])
            rom_cond = 3'b001;
        else if(io.ben & ~next_state_imm[2])
            rom_cond = 3'b010;
        else if(io.ir[11] & ~next_state_imm[0])
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
                io.ird = 0;
            end
            6'd 33: begin        //fetch 2 MDR <- M[MAR]
                rom_ctrl_signals = 25'b 0010000000000000000000010;
                next_state_imm = 6'd 33;
                io.ird = 0;
            end
            6'd 35: begin        //fetch 3 IR <- MDR
                rom_ctrl_signals = 25'b 0001000010000000000000000;
                next_state_imm = 6'd 32;
                io.ird = 0;
            end
            6'd 32: begin        //decode
                rom_ctrl_signals = 25'b 1000000000000000000000000;
                next_state_imm = 6'd 18;
                io.ird = 1;
            end
            6'd 1: begin         //ADD : DR <- SR1 + OP2, setCC 
                rom_ctrl_signals = 25'b 0000011001000000000010000;
                next_state_imm = 6'd 18;
                io.ird = 0;
            end
            6'd 5: begin         //AND : DR <- SR1 & OP2, setCC
                rom_ctrl_signals = 25'b 0000011001000000000010100;
                next_state_imm = 6'd 18;
                io.ird = 0;
            end
            6'd 9: begin         //NOT : DR <- NOT(SR), setCC
                rom_ctrl_signals = 25'b 0000011001000000000011000;
                next_state_imm = 6'd 18;
                io.ird = 0;
            end
            6'd 14: begin        //LEA : DR <- PC + off9, setCC
                rom_ctrl_signals = 25'b 0000010100010001000000000;
                next_state_imm = 6'd 18;
                io.ird = 0;
            end
            6'd 2: begin         //LD : MAR <- PC + off9
                rom_ctrl_signals = 25'b 0100000000101001000000000;
                next_state_imm = 6'd 25;
                io.ird = 0;
            end
            6'd 6: begin         //LDR : MAR <- BaseR + off6
                rom_ctrl_signals = 25'b 0100000100010010100000000;
                next_state_imm = 6'd 25;
                io.ird = 0;
            end
            6'd 10: begin        //LDI : MAR <- PC + off9
                rom_ctrl_signals = 25'b 0100000000101001000000000;
                next_state_imm = 6'd 24;
                io.ird = 0;
            end
            6'd 11: begin        //STI : MAR <- PC + off9
                rom_ctrl_signals = 25'b 0100000000101001000000000;
                next_state_imm = 6'd 29;
                io.ird = 0;
            end
            6'd 7: begin         //STR : MAR <- BaseR + off6
                rom_ctrl_signals = 25'b 0100000100010010100000000;
                next_state_imm = 6'd 23;
                io.ird = 0;
            end
            6'd 3: begin         //ST : MAR <- PC + off9
                rom_ctrl_signals = 25'b 0100000000101001000000000;
                next_state_imm = 6'd 23;
                io.ird = 0;
            end
            6'd 4: begin         //JSR : R7 <- PC , IR[11]
                rom_ctrl_signals = 25'b 0000010000100000001000000;
                next_state_imm = (io.ir[11]) ? 6'd 21 : 6'd 20;
                io.ird = 0;
            end
            6'd 12: begin        //JMP : PC <- BaseR
                rom_ctrl_signals = 25'b 0000100001000100000011100;
                next_state_imm = 6'd 18;
                io.ird = 0;
            end
            6'd 0: begin         //BR : [BEN]
                next_state_imm = (io.ben) ? 6'd 22 : 6'd 18;
                io.ird = 0;
            end
            6'd 28: begin        //MDR <- M[MAR], R7 <- PC, after 15
                rom_ctrl_signals = 25'b 0010010000100000001000010;
                next_state_imm = 6'd 28;
                io.ird = 0;
            end
            6'd 30: begin        //PC <- MDR, after 28
                rom_ctrl_signals = 25'b 0000100010000100000000000;
                next_state_imm = 6'd 18;
                io.ird = 0;
            end
            6'd 24: begin        //MDR <- M[MAR], after 10
                rom_ctrl_signals = 25'b 0010000000000000000000010;
                next_state_imm = 6'd 24;
                io.ird = 0;
            end
            6'd 29: begin        //MDR <- M[MAR], after 11
                rom_ctrl_signals = 25'b 0010000000000000000000010;
                next_state_imm = 6'd 29;
                io.ird = 0;
            end
            6'd 26: begin        //MAR <- MDR, after 24
                rom_ctrl_signals = 25'b 0100000010000000000000000;
                next_state_imm = 6'd 25;
                io.ird = 0;
            end
            6'd 31: begin        //MAR <- MDR, after 29
                rom_ctrl_signals = 25'b 0100000010000000000000000;
                next_state_imm = 6'd 23;
                io.ird = 0;
            end
            6'd 25: begin        //MDR <- M[MAR], after 2, 6, 26
                rom_ctrl_signals = 25'b 0010000000000000000000010;
                next_state_imm = 6'd 25;
                io.ird = 0;
            end
            6'd 27: begin        //DR <- MDR setCC, after 25
                rom_ctrl_signals = 25'b 0000011010000000000000000;
                next_state_imm = 6'd 18;
                io.ird = 0;
            end
            6'd 23: begin        //MDR <- SR, after 3, 7, 31
                rom_ctrl_signals = 25'b 0010000001000000000001100;
                next_state_imm = 6'd 16;
                io.ird = 0;
            end
            6'd 16: begin        //M[MAR] <- MDR, after 23
                rom_ctrl_signals = 25'b 0000000000000000000000011;
                next_state_imm = 6'd 16;
                io.ird = 0;
            end
            6'd 21: begin        //PC <- PC + off11, after 4
                rom_ctrl_signals = 25'b 0000100000001001100000000;
                next_state_imm = 6'd 18;
                io.ird = 0;
            end
            6'd 20: begin        //PC <- BaseR, after 4
                rom_ctrl_signals = 25'b 0000100001000100000001100;
                next_state_imm = 6'd 18;
                io.ird = 0;
            end
            6'd 22: begin        //PC <- PC + off9, after 0
                rom_ctrl_signals = 25'b 0000100000001001000000000;
                next_state_imm = 6'd 18;
                io.ird = 0;
            end
            default : rom_ctrl_signals = 25'b0;             //undefined state
        endcase
    end

    //--instantiate microsequencer--
    microsequencer ctrl_microsequencer (
        .car(next_state_imm),
        .cond(rom_cond),
        .ir(io.ir),
        .ben(io.ben),
        .r(mem_r),
        .ird(io.ird),

        .j(next_state)                            //produces next_state from next_state_imm given conditions
    );

    //--assign outputs--
    assign io.ld_ben = rom_ctrl_signals[24];         //register load signals
    assign io.ld_mar = rom_ctrl_signals[23];
    assign io.ld_mdr = rom_ctrl_signals[22];
    assign io.ld_ir = rom_ctrl_signals[21];
    assign io.ld_pc = rom_ctrl_signals[20];
    assign io.ld_reg = rom_ctrl_signals[19];
    assign io.ld_cc = rom_ctrl_signals[18];

    assign io.gate_marmux = rom_ctrl_signals[17];    //bus driver signals
    assign io.gate_mdr = rom_ctrl_signals[16];
    assign io.gate_alu = rom_ctrl_signals[15];
    assign io.gate_pc = rom_ctrl_signals[14];

    assign io.marmux = rom_ctrl_signals[13];         //mux select signals
    assign io.pcmux = rom_ctrl_signals[12:11];
    assign io.addr1mux = rom_ctrl_signals[10];
    assign io.addr2mux = rom_ctrl_signals[9:8];
    assign io.drmux = rom_ctrl_signals[7:6];
    assign io.sr1mux = rom_ctrl_signals[5:4];

    assign io.aluk = rom_ctrl_signals[3:2];          //alu operation select

    assign io.mio_en = rom_ctrl_signals[1];          //memory signals
    assign io.r_w = rom_ctrl_signals[0];

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

//--cc--
module cc (
    input logic clk, reset,
    input logic [15:0] bus,
    input logic ld,

    output logic [2:0] stat_cc
);
    logic n, z, p;
    //--combinational logic for nzp status signals--
    assign n = bus[15];
    assign z = (bus == 16'b0);
    assign p = !bus[15] && (bus != 16'b0);
    //--synchronous load--
    always_ff @(posedge clk) begin
        if(reset) begin
            stat_cc <= 3'b000;
        end
        else if(ld) begin
            stat_cc <= {n, z, p};
        end
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
    logic [15:0] regs [0:7];                    //these are R0 through R7
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
