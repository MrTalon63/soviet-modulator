module rrc_filter (
    input  wire        clk,       // Read clock (fast_clk)
    input  wire        write_clk, // Write clock (slow_clk)
    input  wire        rstn,
    
    // Baseband Input from Pico
    input  wire        bclk_rising,
    input  wire        in_data, // 1-bit baseband
    
    // Configuration from SPI
    input  wire [7:0]  rrc_L,
    input  wire [7:0]  rrc_span,
    input  wire [15:0] phase_ticks,
    
    // Coefficient memory programming (in write_clk domain)
    input  wire        coeff_we,
    input  wire [12:0] coeff_addr, // 13-bit address for 8192 coefficients
    input  wire [15:0] coeff_data,
    
    // Output to CIC
    output reg         out_valid,
    output reg signed [21:0] out_data // 22-bit to prevent overflow for span 64
);

    // =========================================================================
    // 1. Dual Coefficient Memories (8192 x 16-bit, dual-port for 2-tap/cycle)
    // =========================================================================
    reg signed [15:0] coeff_mem_A [0:8191];
    reg signed [15:0] coeff_mem_B [0:8191];
    reg signed [15:0] ram_out_A;
    reg signed [15:0] ram_out_B;

    wire ram_we = coeff_we;

    always @(posedge write_clk) begin
        if (ram_we) begin
            coeff_mem_A[coeff_addr] <= coeff_data;
            coeff_mem_B[coeff_addr] <= coeff_data;
        end
    end

    always @(posedge clk) begin
        ram_out_A <= coeff_mem_A[coeff_addr_read_A];
    end

    always @(posedge clk) begin
        ram_out_B <= coeff_mem_B[coeff_addr_read_B];
    end

    // =========================================================================
    // 2. Configuration Registers
    // =========================================================================
    reg [7:0]  rrc_L_reg;
    reg [7:0]  rrc_span_reg;
    reg [7:0]  rrc_span_minus_1;
    reg [15:0] phase_ticks_reg;
    reg [15:0] phase_ticks_minus_1;

    always @(posedge clk) begin
        if (!rstn) begin
            rrc_L_reg        <= 8'd8;
            rrc_span_reg     <= 8'd8;
            rrc_span_minus_1 <= 8'd7;
            phase_ticks_reg  <= 16'd100;
            phase_ticks_minus_1 <= 16'd99;
        end else begin
            rrc_L_reg        <= rrc_L;
            rrc_span_reg     <= rrc_span;
            rrc_span_minus_1 <= rrc_span - 8'd1;
            phase_ticks_reg  <= phase_ticks;
            phase_ticks_minus_1 <= phase_ticks - 16'd1;
        end
    end

    // =========================================================================
    // 3. Tick-Based Phase Counter (synchronized to bclk)
    // =========================================================================

    reg [15:0] tick_cnt;
    reg [7:0]  phase_cnt;
    reg [63:0] shift_reg;
    reg        in_data_reg;

    reg        start_calc;
    reg [7:0]  calc_phase;

    always @(posedge clk) begin
        in_data_reg <= in_data;
    end

    always @(posedge clk) begin
        if (!rstn) begin
            tick_cnt   <= 16'd0;
            phase_cnt  <= 8'd0;
            shift_reg  <= 64'd0;
            start_calc <= 1'b0;
            calc_phase <= 8'd0;
        end else begin
            start_calc <= 1'b0; // Auto-clear trigger pulse (1-cycle wide)
            
            if (bclk_rising) begin
                shift_reg <= {shift_reg[62:0], in_data_reg};
                tick_cnt  <= 16'd0;
                phase_cnt <= 8'd0;
                
                if (rrc_L_reg > 8'd1) begin
                    start_calc <= 1'b1;
                    calc_phase <= 8'd0;
                end
            end else if (phase_cnt < rrc_L_reg - 8'd1) begin
                // Wait phase_ticks cycles, then fire next phase
                tick_cnt <= tick_cnt + 16'd1;
                
                if (tick_cnt >= phase_ticks_minus_1) begin
                    tick_cnt   <= 16'd0;
                    phase_cnt  <= phase_cnt + 8'd1;
                    
                    // Fire next sample calculation
                    start_calc <= 1'b1;
                    calc_phase <= phase_cnt + 8'd1;
                end
            end
        end
    end

    // =========================================================================
    // 4. RRC Engine - 2 taps/cycle sequential FIR
    //
    // Coefficient address layout (matches firmware upload):
    //   addr[12:0] = {k[6:3], phase[6:0], k[2:0]}
    //   where k = tap index (0..span-1)
    //
    // On start_calc (t=0): read taps 0,1, set r_first=1
    // On t=2,4,6,...: read taps 2,3 then 4,5 etc.
    // On last pair: r_last=1, stop calc_running
    // =========================================================================

    reg [6:0]  calc_cnt;
    reg        calc_running;
    reg [12:0] coeff_addr_read_A;
    reg [12:0] coeff_addr_read_B;
    reg [63:0] shift_reg_copy;
    
    reg        r_valid_addr;
    reg        symbol_bit_addr_A;
    reg        symbol_bit_addr_B;
    reg        last_addr;
    reg        first_addr;         // 1-cycle pulse on start_calc
    
    reg        r_valid;
    reg        r_symbol_bit_A;
    reg        r_symbol_bit_B;
    reg        r_last;
    reg        r_first;            // Delayed copy of first_addr
    
    reg [7:0]  calc_phase_reg;

    wire [6:0] next_tap = calc_cnt + 7'd2;

    always @(posedge clk) begin
        if (!rstn) begin
            calc_running       <= 1'b0;
            calc_cnt           <= 7'd0;
            coeff_addr_read_A  <= 13'd0;
            coeff_addr_read_B  <= 13'd0;
            shift_reg_copy     <= 64'd0;
            r_valid_addr       <= 1'b0;
            symbol_bit_addr_A  <= 1'b0;
            symbol_bit_addr_B  <= 1'b0;
            last_addr          <= 1'b0;
            first_addr         <= 1'b0;
            calc_phase_reg     <= 8'd0;
        end else begin
            first_addr <= 1'b0;  // Default: single-cycle pulse
            
            if (start_calc) begin
                calc_running   <= 1'b1;
                calc_cnt       <= 7'd0;
                calc_phase_reg <= calc_phase;
                shift_reg_copy <= shift_reg;
                r_valid_addr   <= 1'b1;
                first_addr     <= 1'b1;
                last_addr      <= (rrc_span_reg <= 8'd2);
                
                // Taps 0,1: addr = {0, phase[6:0], 0} and {0, phase[6:0], 1}
                coeff_addr_read_A <= {4'd0, calc_phase[6:0], 3'd0};
                coeff_addr_read_B <= {4'd0, calc_phase[6:0], 3'd1};
                
                symbol_bit_addr_A <= shift_reg[0];
                symbol_bit_addr_B <= shift_reg[1];
            end else if (calc_running) begin
                calc_cnt <= next_tap;
                
                // Address: {k[6:3], phase[6:0], k[2:0]}
                coeff_addr_read_A <= {next_tap[6:3], calc_phase_reg[6:0], next_tap[2:0]};
                coeff_addr_read_B <= {next_tap[6:3], calc_phase_reg[6:0], next_tap[2:0] + 3'd1};
                
                symbol_bit_addr_A <= shift_reg_copy[next_tap];
                symbol_bit_addr_B <= shift_reg_copy[next_tap + 7'd1];
                
                last_addr <= (next_tap >= rrc_span_minus_1);
                if (next_tap >= rrc_span_minus_1) begin
                    calc_running <= 1'b0;
                end
            end else begin
                r_valid_addr <= 1'b0;
                last_addr    <= 1'b0;
            end
        end
    end

    // Delay address-domain control signals by 1 cycle to align with BSRAM readout
    always @(posedge clk) begin
        if (!rstn) begin
            r_valid        <= 1'b0;
            r_symbol_bit_A <= 1'b0;
            r_symbol_bit_B <= 1'b0;
            r_last         <= 1'b0;
            r_first        <= 1'b0;
        end else begin
            r_valid        <= r_valid_addr;
            r_symbol_bit_A <= symbol_bit_addr_A;
            r_symbol_bit_B <= symbol_bit_addr_B;
            r_last         <= last_addr;
            r_first        <= first_addr;
        end
    end

    // Stage 1: BSRAM output copy
    reg signed [15:0] ram_out_A_reg;
    reg signed [15:0] ram_out_B_reg;
    reg               r_symbol_bit_A_d1;
    reg               r_symbol_bit_B_d1;

    always @(posedge clk) begin
        if (!rstn) begin
            ram_out_A_reg     <= 16'sd0;
            ram_out_B_reg     <= 16'sd0;
            r_symbol_bit_A_d1 <= 1'b0;
            r_symbol_bit_B_d1 <= 1'b0;
        end else begin
            ram_out_A_reg     <= ram_out_A;
            ram_out_B_reg     <= ram_out_B;
            r_symbol_bit_A_d1 <= r_symbol_bit_A;
            r_symbol_bit_B_d1 <= r_symbol_bit_B;
        end
    end

    // Stage 2: BPSK negation (±coeff)
    reg signed [15:0] term_reg_A;
    reg signed [15:0] term_reg_B;

    always @(posedge clk) begin
        if (!rstn) begin
            term_reg_A <= 16'sd0;
            term_reg_B <= 16'sd0;
        end else begin
            term_reg_A <= r_symbol_bit_A_d1 ? ram_out_A_reg : -ram_out_A_reg;
            term_reg_B <= r_symbol_bit_B_d1 ? ram_out_B_reg : -ram_out_B_reg;
        end
    end

    // Stage 3: Term addition
    reg signed [21:0] sum_terms;

    always @(posedge clk) begin
        if (!rstn) begin
            sum_terms <= 22'sd0;
        end else begin
            sum_terms <= $signed(term_reg_A) + $signed(term_reg_B);
        end
    end

    // Aligned control pipeline (3 cycles: BSRAM + reg + neg + add)
    reg r_valid_d1, r_first_d1, r_last_d1;
    reg r_valid_d2, r_first_d2, r_last_d2;

    always @(posedge clk) begin
        if (!rstn) begin
            r_valid_d1 <= 1'b0; r_first_d1 <= 1'b0; r_last_d1 <= 1'b0;
            r_valid_d2 <= 1'b0; r_first_d2 <= 1'b0; r_last_d2 <= 1'b0;
        end else begin
            r_valid_d1 <= r_valid;
            r_first_d1 <= r_first;
            r_last_d1  <= r_last;
            
            r_valid_d2 <= r_valid_d1;
            r_first_d2 <= r_first_d1;
            r_last_d2  <= r_last_d1;
        end
    end

    // Stage 4: FIR accumulator
    reg signed [21:0] accum_reg;
    reg signed [21:0] final_output;
    reg               out_valid_reg;

    always @(posedge clk) begin
        if (!rstn) begin
            accum_reg     <= 22'sd0;
            final_output  <= 22'sd0;
            out_valid_reg <= 1'b0;
        end else begin
            out_valid_reg <= 1'b0;
            if (r_valid_d2) begin
                if (r_first_d2) begin
                    accum_reg <= sum_terms;
                end else begin
                    accum_reg <= accum_reg + sum_terms;
                end
                
                if (r_last_d2) begin
                    final_output  <= r_first_d2 ? sum_terms : (accum_reg + sum_terms);
                    out_valid_reg <= 1'b1;
                end
            end
        end
    end

    always @(posedge clk) begin
        if (!rstn) begin
            out_valid <= 1'b0;
            out_data  <= 22'sd0;
        end else begin
            out_valid <= out_valid_reg;
            if (out_valid_reg) begin
                out_data <= final_output;
            end
        end
    end

endmodule