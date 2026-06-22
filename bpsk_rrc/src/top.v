module top (
    input  wire       xtal_27mhz_pin,    // Onboard 27MHz crystal
    input  wire       hardware_reset_n,  // Physical reset button/pin
    
    // RP2350 SPI interface pins
    input  wire       spi_cs_n,
    input  wire       spi_sck,
    input  wire       spi_mosi,
    output wire       spi_miso,
    
    // RP2350 Baseband Data interface
    input  wire       mcu_bit_clk,
    input  wire       mcu_serial_data,   
    
    // Physical AD9708 Pins
    output wire [7:0] physical_ad9708_data,
    output wire       physical_ad9708_clk,
    
    // Status
    output wire       activity_led
);

    // =========================================================================
    // 1. CLOCK & RESET
    // =========================================================================
    wire fast_clk;   // 90 MHz - data path
    wire slow_clk;   // 45 MHz - SPI
    wire pll_lock;

    PLL_90_45MHz u_pll (
        .clkout(fast_clk),
        .clkoutd(slow_clk),
        .clkin(xtal_27mhz_pin),
        .lock(pll_lock)
    );

    // Reset for fast_clk domain
    reg [2:0] rst_sync_fast;
    always @(posedge fast_clk or negedge hardware_reset_n) begin
        if (!hardware_reset_n) begin
            rst_sync_fast <= 3'b000;
        end else if (!pll_lock) begin
            rst_sync_fast <= 3'b000;
        end else begin
            rst_sync_fast <= {rst_sync_fast[1:0], 1'b1};
        end
    end
    wire sys_rst_n_fast = rst_sync_fast[2];

    // Reset for slow_clk domain
    reg [2:0] rst_sync_slow;
    always @(posedge slow_clk or negedge hardware_reset_n) begin
        if (!hardware_reset_n) begin
            rst_sync_slow <= 3'b000;
        end else if (!pll_lock) begin
            rst_sync_slow <= 3'b000;
        end else begin
            rst_sync_slow <= {rst_sync_slow[1:0], 1'b1};
        end
    end
    wire sys_rst_n_slow = rst_sync_slow[2];

    // =========================================================================
    // 2. SPI SLAVE (API from RP2350) - runs on slow_clk (45 MHz)
    // =========================================================================
    wire [7:0]  rrc_L;
    wire [7:0]  rrc_span;
    wire [7:0]  cic_shift;
    wire        cic_bypass;
    wire [15:0] phase_ticks_slow;
    
    wire        coeff_we_slow;
    wire [12:0] coeff_addr_slow;
    wire [15:0] coeff_data_slow;

    spi_slave u_spi_slave (
        .clk(slow_clk),
        .rstn(sys_rst_n_slow),
        .spi_cs_n(spi_cs_n),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .rrc_L(rrc_L),
        .rrc_span(rrc_span),
        .cic_shift(cic_shift),
        .cic_bypass(cic_bypass),
        .phase_ticks(phase_ticks_slow),
        .coeff_we(coeff_we_slow),
        .coeff_addr(coeff_addr_slow),
        .coeff_data(coeff_data_slow)
    );

    // =========================================================================
    // 3. CLOCK DOMAIN CROSSING (slow_clk -> fast_clk)
    // =========================================================================
    // Dual-rank synchronizers for all SPI outputs crossing into fast_clk domain.
    reg [7:0]  rrc_L_sync_1, rrc_L_sync;
    reg [7:0]  rrc_span_sync_1, rrc_span_sync;
    reg [7:0]  cic_shift_sync_1, cic_shift_sync;
    reg        cic_bypass_sync_1, cic_bypass_sync;
    reg [15:0] phase_ticks_sync_1, phase_ticks_sync;
    reg        coeff_we_sync_1, coeff_we_sync;
    reg [12:0] coeff_addr_sync_1, coeff_addr_sync;
    reg [15:0] coeff_data_sync_1, coeff_data_sync;

    always @(posedge fast_clk or negedge sys_rst_n_fast) begin
        if (!sys_rst_n_fast) begin
            rrc_L_sync_1      <= 8'd8;  rrc_L_sync      <= 8'd8;
            rrc_span_sync_1   <= 8'd8;  rrc_span_sync   <= 8'd8;
            cic_shift_sync_1  <= 8'd12; cic_shift_sync  <= 8'd12;
            cic_bypass_sync_1 <= 1'b0;  cic_bypass_sync <= 1'b0;
            phase_ticks_sync_1 <= 16'd100; phase_ticks_sync <= 16'd100;
            coeff_we_sync_1   <= 1'b0;  coeff_we_sync   <= 1'b0;
            coeff_addr_sync_1 <= 13'd0; coeff_addr_sync <= 13'd0;
            coeff_data_sync_1 <= 16'd0; coeff_data_sync <= 16'd0;
        end else begin
            rrc_L_sync_1      <= rrc_L;               rrc_L_sync      <= rrc_L_sync_1;
            rrc_span_sync_1   <= rrc_span;            rrc_span_sync   <= rrc_span_sync_1;
            cic_shift_sync_1  <= cic_shift;           cic_shift_sync  <= cic_shift_sync_1;
            cic_bypass_sync_1 <= cic_bypass;          cic_bypass_sync <= cic_bypass_sync_1;
            phase_ticks_sync_1 <= phase_ticks_slow;   phase_ticks_sync <= phase_ticks_sync_1;
            coeff_we_sync_1   <= coeff_we_slow;       coeff_we_sync   <= coeff_we_sync_1;
            coeff_addr_sync_1 <= coeff_addr_slow;     coeff_addr_sync <= coeff_addr_sync_1;
            coeff_data_sync_1 <= coeff_data_slow;     coeff_data_sync <= coeff_data_sync_1;
        end
    end

    // =========================================================================
    // 4. BASEBAND DESERIALIZER & CLOCK SYNC (fast_clk domain, 90 MHz)
    // =========================================================================
    reg [2:0] mcu_bclk_sync;
    reg [2:0] mcu_data_sync;

    always @(posedge fast_clk or negedge sys_rst_n_fast) begin
        if (!sys_rst_n_fast) begin
            mcu_bclk_sync <= 3'b000;
            mcu_data_sync <= 3'b000;
        end else begin
            mcu_bclk_sync <= {mcu_bclk_sync[1:0], mcu_bit_clk};
            mcu_data_sync <= {mcu_data_sync[1:0], mcu_serial_data};
        end
    end

    reg bclk_rising;
    always @(posedge fast_clk or negedge sys_rst_n_fast) begin
        if (!sys_rst_n_fast) begin
            bclk_rising <= 1'b0;
        end else begin
            bclk_rising <= (mcu_bclk_sync[2] == 1'b0 && mcu_bclk_sync[1] == 1'b1);
        end
    end
    wire in_data = mcu_data_sync[1];

    // =========================================================================
    // 5. RRC FILTER (Polyphase FIR) - runs on fast_clk (90 MHz)
    // =========================================================================
    wire        rrc_out_valid;
    wire signed [21:0] rrc_out_data;

    rrc_filter u_rrc_filter (
        .clk(fast_clk),
        .write_clk(slow_clk),
        .rstn(sys_rst_n_fast),
        .bclk_rising(bclk_rising),
        .in_data(in_data),
        .rrc_L(rrc_L_sync),
        .rrc_span(rrc_span_sync),
        .phase_ticks(phase_ticks_sync),
        
        .coeff_we(coeff_we_slow),     
        .coeff_addr(coeff_addr_slow), 
        .coeff_data(coeff_data_slow), 
        
        .out_valid(rrc_out_valid),
        .out_data(rrc_out_data)
    );

    // =========================================================================
    // 6. RRC SCALING & CLIPPING FOR DAC (fast_clk domain)
    // =========================================================================
    reg signed [21:0] rrc_hold_data;
    always @(posedge fast_clk or negedge sys_rst_n_fast) begin
        if (!sys_rst_n_fast) begin
            rrc_hold_data <= 22'd0;
        end else begin
            if (cic_bypass_sync) begin
                rrc_hold_data <= in_data ? 22'sd32767 : -22'sd32767;
            end else if (rrc_out_valid) begin
                rrc_hold_data <= rrc_out_data;
            end
        end
    end

    reg [3:0] cic_shift_reg;
    always @(posedge fast_clk or negedge sys_rst_n_fast) begin
        if (!sys_rst_n_fast) begin
            cic_shift_reg <= 4'b0000;
        end else begin
            cic_shift_reg <= cic_shift_sync[3:0];
        end
    end

    reg signed [21:0] shift_stage1;
    always @(posedge fast_clk or negedge sys_rst_n_fast) begin
        if (!sys_rst_n_fast) begin
            shift_stage1 <= 22'd0;
        end else begin
            case (cic_shift_reg)
                4'd0:  shift_stage1 <= rrc_hold_data;
                4'd1:  shift_stage1 <= rrc_hold_data >>> 1;
                4'd2:  shift_stage1 <= rrc_hold_data >>> 2;
                4'd3:  shift_stage1 <= rrc_hold_data >>> 3;
                4'd4:  shift_stage1 <= rrc_hold_data >>> 4;
                4'd5:  shift_stage1 <= rrc_hold_data >>> 5;
                4'd6:  shift_stage1 <= rrc_hold_data >>> 6;
                4'd7:  shift_stage1 <= rrc_hold_data >>> 7;
                4'd8:  shift_stage1 <= rrc_hold_data >>> 8;
                4'd9:  shift_stage1 <= rrc_hold_data >>> 9;
                4'd10: shift_stage1 <= rrc_hold_data >>> 10;
                4'd11: shift_stage1 <= rrc_hold_data >>> 11;
                4'd12: shift_stage1 <= rrc_hold_data >>> 12;
                4'd13: shift_stage1 <= rrc_hold_data >>> 13;
                4'd14: shift_stage1 <= rrc_hold_data >>> 14;
                4'd15: shift_stage1 <= rrc_hold_data >>> 15;
            endcase
        end
    end

    // 22-bit to 8-bit scaling with clipping
    reg signed [7:0] clipped_rrc;
    always @(posedge fast_clk or negedge sys_rst_n_fast) begin
        if (!sys_rst_n_fast) begin
            clipped_rrc <= 8'sd0;
        end else begin
            if (shift_stage1[21]) begin
                if (!(&shift_stage1[20:7])) begin
                    clipped_rrc <= 8'sh80;
                end else begin
                    clipped_rrc <= shift_stage1[7:0];
                end
            end else begin
                if (|shift_stage1[20:7]) begin
                    clipped_rrc <= 8'sh7F;
                end else begin
                    clipped_rrc <= shift_stage1[7:0];
                end
            end
        end
    end

    // =========================================================================
    // 7. OUTPUT TO DAC (fast_clk domain, 90 MHz clock to AD9708)
    // =========================================================================
    reg [7:0] dac_data;
    always @(posedge fast_clk or negedge sys_rst_n_fast) begin
        if (!sys_rst_n_fast) begin
            dac_data <= 8'h80;
        end else begin
            dac_data <= {~clipped_rrc[7], clipped_rrc[6:0]};
        end
    end

    reg [7:0] ad9708_d_iob /* synthesis syn_useioff = 1 */;
    always @(posedge fast_clk or negedge sys_rst_n_fast) begin
        if (!sys_rst_n_fast)
            ad9708_d_iob <= 8'h80;
        else
            ad9708_d_iob <= dac_data;
    end

    assign physical_ad9708_data = ad9708_d_iob;
    // AD9708 clock = fast_clk (90 MHz) - within DAC spec
    assign physical_ad9708_clk  = fast_clk; 

    // =========================================================================
    // 8. ACTIVITY LED
    // =========================================================================
    reg [12:0] led_toggle_cnt;
    always @(posedge fast_clk or negedge sys_rst_n_fast) begin
        if (!sys_rst_n_fast)
            led_toggle_cnt <= 13'd0;
        else if (bclk_rising)
            led_toggle_cnt <= led_toggle_cnt + 1'b1;
    end

    assign activity_led = ~led_toggle_cnt[12];

endmodule