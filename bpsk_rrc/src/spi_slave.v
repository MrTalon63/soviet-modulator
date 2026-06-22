module spi_slave (
    input  wire        clk,
    input  wire        rstn,
    
    // SPI interface pins
    input  wire        spi_cs_n,
    input  wire        spi_sck,
    input  wire        spi_mosi,
    output wire        spi_miso,
    
    // Configuration outputs
    output reg  [7:0]  rrc_L,
    output reg  [7:0]  rrc_span,
    output reg  [7:0]  cic_shift,
    output reg         cic_bypass,
    output reg  [15:0] phase_ticks,
    
    // Coefficient programming interface
    output reg         coeff_we,
    output reg  [12:0] coeff_addr,
    output reg  [15:0] coeff_data
);

    // =========================================================================
    // Synchronizers for SPI inputs
    // =========================================================================
    reg [2:0] cs_sync   /* synthesis syn_preserve = 1 */;
    reg [2:0] cs_sync_a /* synthesis syn_preserve = 1 */;
    reg [2:0] cs_sync_b /* synthesis syn_preserve = 1 */;
    reg [2:0] sck_sync;
    reg [2:0] mosi_sync;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            cs_sync   <= 3'b111;
            cs_sync_a <= 3'b111;
            cs_sync_b <= 3'b111;
            sck_sync  <= 3'b000;
            mosi_sync <= 3'b000;
        end else begin
            cs_sync   <= {cs_sync[1:0], spi_cs_n};
            cs_sync_a <= {cs_sync_a[1:0], spi_cs_n};
            cs_sync_b <= {cs_sync_b[1:0], spi_cs_n};
            sck_sync  <= {sck_sync[1:0], spi_sck};
            mosi_sync <= {mosi_sync[1:0], spi_mosi};
        end
    end

    wire cs_val   = cs_sync[1];
    wire cs_val_a = cs_sync_a[1];
    wire cs_val_b = cs_sync_b[1];
    wire sck_val  = sck_sync[1];
    wire mosi_val = mosi_sync[1];

    // Edge detection for SCK and CS
    wire sck_rising  = (sck_sync[2] == 1'b0 && sck_sync[1] == 1'b1);
    wire sck_falling = (sck_sync[2] == 1'b1 && sck_sync[1] == 1'b0);
    wire cs_falling  = (cs_sync[2] == 1'b1 && cs_sync[1] == 1'b0);
    wire cs_rising   = (cs_sync[2] == 1'b0 && cs_sync[1] == 1'b1);

    // Registered edges to prevent timing violations on high-fanout control paths
    reg sck_rising_reg;
    reg sck_falling_reg;
    reg cs_falling_reg;
    reg cs_rising_reg;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            sck_rising_reg  <= 1'b0;
            sck_falling_reg <= 1'b0;
            cs_falling_reg  <= 1'b0;
            cs_rising_reg   <= 1'b0;
        end else begin
            sck_rising_reg  <= sck_rising;
            sck_falling_reg <= sck_falling;
            cs_falling_reg  <= cs_falling;
            cs_rising_reg   <= cs_rising;
        end
    end

    // =========================================================================
    // SPI State Machine & Registers
    // =========================================================================
    reg [1:0]  state;
    reg [3:0]  bit_cnt;
    reg [7:0]  shift_in;
    reg [7:0]  shift_in_d1;
    reg [6:0]  reg_addr;
    reg        is_read;
    reg [7:0]  shift_out;
    
    reg [7:0]  coeff_lsb;
    reg        coeff_byte_idx; // 0 = LSB, 1 = MSB
    reg        coeff_we_d1;

    // Pipelined read data path registers to prevent combinational paths to shift_out
    reg [6:0]  cmd_addr;
    reg [7:0]  next_read_byte;

    // Pipelined helper registers for timing closure
    reg        bit_cnt_is_8;
    reg        coeff_addr_rst;
    reg [2:0]  load_shift_out_delay;
    wire       load_shift_out_pulse;

    localparam STATE_IDLE    = 2'd0;
    localparam STATE_CMD     = 2'd1; // Reading 8-bit command (W/R_n + 7-bit addr)
    localparam STATE_DATA_RX = 2'd2; // Receiving data

    // Precomputed address match registers to isolate 7-bit comparisons
    reg reg_addr_is_00;
    reg reg_addr_is_01;
    reg reg_addr_is_02;
    reg reg_addr_is_04;
    reg reg_addr_is_05;
    reg reg_addr_is_06;
    reg reg_addr_is_10;

    // Re-registered write control signals to fully decouple control paths
    reg rrc_L_we;
    reg rrc_span_we;
    reg cic_shift_we;
    reg cic_bypass_we;
    reg phase_lsb_we;
    reg phase_msb_we;
    reg coeff_lsb_we;
    reg coeff_we_next;
    reg coeff_write_pulse_reg;
    reg reg_addr_load;
    reg reg_addr_inc;
    reg state_is_data_rx;

    reg write_active_rrc_L;
    reg write_active_rrc_span;
    reg write_active_cic_shift;
    reg write_active_cic_bypass;
    reg write_active_phase_lsb;
    reg write_active_phase_msb;
    reg write_active_coeff_lsb;
    reg write_active_coeff_we_next;
    reg write_active_coeff_pulse;
    reg write_active_inc;
    reg cmd_active;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            write_active_rrc_L         <= 1'b0;
            write_active_rrc_span      <= 1'b0;
            write_active_cic_shift     <= 1'b0;
            write_active_cic_bypass    <= 1'b0;
            write_active_phase_lsb     <= 1'b0;
            write_active_phase_msb     <= 1'b0;
            write_active_coeff_lsb     <= 1'b0;
            write_active_coeff_we_next <= 1'b0;
            write_active_coeff_pulse   <= 1'b0;
            write_active_inc           <= 1'b0;
            cmd_active                 <= 1'b0;
        end else begin
            write_active_rrc_L         <= !cs_val_a && !cs_falling_reg && state_is_data_rx && !is_read && reg_addr_is_00;
            write_active_rrc_span      <= !cs_val_a && !cs_falling_reg && state_is_data_rx && !is_read && reg_addr_is_01;
            write_active_cic_shift     <= !cs_val_a && !cs_falling_reg && state_is_data_rx && !is_read && reg_addr_is_02;
            write_active_cic_bypass    <= !cs_val_a && !cs_falling_reg && state_is_data_rx && !is_read && reg_addr_is_04;
            write_active_phase_lsb     <= !cs_val_a && !cs_falling_reg && state_is_data_rx && !is_read && reg_addr_is_05;
            write_active_phase_msb     <= !cs_val_a && !cs_falling_reg && state_is_data_rx && !is_read && reg_addr_is_06;
            write_active_coeff_lsb     <= !cs_val_a && !cs_falling_reg && state_is_data_rx && !is_read && reg_addr_is_10 && (coeff_byte_idx == 1'b0);
            write_active_coeff_we_next <= !cs_val_a && !cs_falling_reg && state_is_data_rx && !is_read && reg_addr_is_10 && (coeff_byte_idx == 1'b1);
            write_active_coeff_pulse   <= !cs_val_a && !cs_falling_reg && state_is_data_rx && !is_read && reg_addr_is_10;
            write_active_inc           <= !cs_val_a && !cs_falling_reg && state_is_data_rx && (is_read || !reg_addr_is_10);
            cmd_active                 <= !cs_val_b && !cs_falling_reg && (state == STATE_CMD);
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            rrc_L_we              <= 1'b0;
            rrc_span_we           <= 1'b0;
            cic_shift_we          <= 1'b0;
            cic_bypass_we         <= 1'b0;
            phase_lsb_we          <= 1'b0;
            phase_msb_we          <= 1'b0;
            coeff_lsb_we          <= 1'b0;
            coeff_we_next         <= 1'b0;
            coeff_write_pulse_reg <= 1'b0;
            reg_addr_load         <= 1'b0;
            reg_addr_inc          <= 1'b0;
            shift_in_d1           <= 8'd0;
        end else begin
            shift_in_d1           <= shift_in;
            
            rrc_L_we              <= write_active_rrc_L && sck_falling_reg && bit_cnt_is_8;
            rrc_span_we           <= write_active_rrc_span && sck_falling_reg && bit_cnt_is_8;
            cic_shift_we          <= write_active_cic_shift && sck_falling_reg && bit_cnt_is_8;
            cic_bypass_we         <= write_active_cic_bypass && sck_falling_reg && bit_cnt_is_8;
            phase_lsb_we          <= write_active_phase_lsb && sck_falling_reg && bit_cnt_is_8;
            phase_msb_we          <= write_active_phase_msb && sck_falling_reg && bit_cnt_is_8;
            coeff_lsb_we          <= write_active_coeff_lsb && sck_falling_reg && bit_cnt_is_8;
            coeff_we_next         <= write_active_coeff_we_next && sck_falling_reg && bit_cnt_is_8;
            coeff_write_pulse_reg <= write_active_coeff_pulse && sck_falling_reg && bit_cnt_is_8;
            reg_addr_load         <= cmd_active && sck_falling_reg && bit_cnt_is_8;
            reg_addr_inc          <= write_active_inc && sck_falling_reg && bit_cnt_is_8;
        end
    end

    // Configuration register updates (decoupled from state machine logic)
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            rrc_L      <= 8'd8;
            rrc_span   <= 8'd8;
            cic_shift  <= 8'd12;
            cic_bypass <= 1'b0;
            phase_ticks <= 16'd100;
        end else begin
            if (rrc_L_we)      rrc_L      <= shift_in_d1;
            if (rrc_span_we)   rrc_span   <= shift_in_d1;
            if (cic_shift_we)  cic_shift  <= shift_in_d1;
            if (cic_bypass_we) cic_bypass <= shift_in_d1[0];
            if (phase_lsb_we)  phase_ticks[7:0]  <= shift_in_d1;
            if (phase_msb_we)  phase_ticks[15:8] <= shift_in_d1;
        end
    end

    // Coefficient write control
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            coeff_lsb      <= 8'd0;
            coeff_byte_idx <= 1'b0;
            coeff_we       <= 1'b0;
        end else begin
            coeff_we <= coeff_we_next;
            if (cs_val) begin
                coeff_byte_idx <= 1'b0;
            end else begin
                if (coeff_lsb_we) begin
                    coeff_lsb <= shift_in_d1;
                end
                if (coeff_write_pulse_reg) begin
                    coeff_byte_idx <= ~coeff_byte_idx;
                end
            end
        end
    end

    // SPI read data load delay line (takes 3 cycles after address load/increment)
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            load_shift_out_delay <= 3'd0;
        end else begin
            load_shift_out_delay <= {load_shift_out_delay[1:0], (reg_addr_load && shift_in_d1[7]) || reg_addr_inc};
        end
    end
    assign load_shift_out_pulse = load_shift_out_delay[2];

    // Address counter, match registers, and status update
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            reg_addr       <= 7'd0;
            coeff_addr     <= 13'd0;
            coeff_we_d1    <= 1'b0;
            reg_addr_is_00 <= 1'b0;
            reg_addr_is_01 <= 1'b0;
            reg_addr_is_02 <= 1'b0;
            reg_addr_is_04 <= 1'b0;
            reg_addr_is_10 <= 1'b0;
        end else begin
            coeff_we_d1    <= coeff_we;
            
        if (reg_addr_load) begin
            reg_addr <= shift_in_d1[6:0];
            reg_addr_is_00 <= (shift_in_d1[6:0] == 7'h00);
            reg_addr_is_01 <= (shift_in_d1[6:0] == 7'h01);
            reg_addr_is_02 <= (shift_in_d1[6:0] == 7'h02);
            reg_addr_is_04 <= (shift_in_d1[6:0] == 7'h04);
            reg_addr_is_05 <= (shift_in_d1[6:0] == 7'h05);
            reg_addr_is_06 <= (shift_in_d1[6:0] == 7'h06);
            reg_addr_is_10 <= (shift_in_d1[6:0] == 7'h10);
        end else if (reg_addr_inc) begin
            reg_addr <= reg_addr + 1'b1;
            reg_addr_is_00 <= 1'b0;
            reg_addr_is_01 <= (reg_addr + 1'b1 == 7'h01);
            reg_addr_is_02 <= (reg_addr + 1'b1 == 7'h02);
            reg_addr_is_04 <= (reg_addr + 1'b1 == 7'h04);
            reg_addr_is_05 <= (reg_addr + 1'b1 == 7'h05);
            reg_addr_is_06 <= (reg_addr + 1'b1 == 7'h06);
            reg_addr_is_10 <= (reg_addr + 1'b1 == 7'h10);
        end

            if (coeff_addr_rst) begin
                coeff_addr <= 13'd0;
            end else if (coeff_we_d1) begin
                coeff_addr <= coeff_addr + 1'b1;
            end
        end
    end

    // Simplified address decoding to prevent long routing delays on read mux
    reg [7:0] next_read_byte_cmd;
    reg [7:0] next_read_byte_reg;

    always @(posedge clk) begin
        if (cmd_addr[6:3] == 4'b0000) begin
            case (cmd_addr[2:0])
                3'd0: next_read_byte_cmd <= rrc_L;
                3'd1: next_read_byte_cmd <= rrc_span;
                3'd2: next_read_byte_cmd <= cic_shift;
                3'd4: next_read_byte_cmd <= {7'd0, cic_bypass};
                3'd5: next_read_byte_cmd <= phase_ticks[7:0];
                3'd6: next_read_byte_cmd <= phase_ticks[15:8];
                default: next_read_byte_cmd <= 8'h00;
            endcase
        end else if (cmd_addr == 7'h7F) begin
            next_read_byte_cmd <= 8'hA5;
        end else begin
            next_read_byte_cmd <= 8'h00;
        end
    end

    always @(posedge clk) begin
        if (reg_addr_is_00) begin
            next_read_byte_reg <= rrc_L;
        end else if (reg_addr_is_01) begin
            next_read_byte_reg <= rrc_span;
        end else if (reg_addr_is_02) begin
            next_read_byte_reg <= cic_shift;
        end else if (reg_addr_is_04) begin
            next_read_byte_reg <= {7'd0, cic_bypass};
        end else if (reg_addr_is_05) begin
            next_read_byte_reg <= phase_ticks[7:0];
        end else if (reg_addr_is_06) begin
            next_read_byte_reg <= phase_ticks[15:8];
        end else begin
            next_read_byte_reg <= 8'h00;
        end
    end

    // main state machine
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state          <= STATE_IDLE;
            bit_cnt        <= 4'd0;
            shift_in       <= 8'd0;
            is_read        <= 1'b0;
            cmd_addr           <= 7'd0;
            bit_cnt_is_8       <= 1'b0;
            coeff_addr_rst     <= 1'b0;
            next_read_byte     <= 8'h00;
            state_is_data_rx   <= 1'b0;
        end else begin
            bit_cnt_is_8 <= (bit_cnt == 4'd8);
            next_read_byte <= (state == STATE_CMD) ? next_read_byte_cmd : next_read_byte_reg;
            
            if (coeff_addr_rst) begin
                coeff_addr_rst <= 1'b0;
            end
            
            // Pipelined capture of command address and read/write bit
            if (state == STATE_CMD && bit_cnt_is_8) begin
                cmd_addr <= shift_in[6:0];
                is_read  <= shift_in[7];
            end
            
            if (cs_val) begin
                state <= STATE_IDLE; // Reset when CS is high
                state_is_data_rx <= 1'b0;
            end else if (cs_falling_reg) begin
                state          <= STATE_CMD;
                bit_cnt        <= 4'd0;
                shift_in       <= 8'd0;
                state_is_data_rx <= 1'b0;
            end else begin
                case (state)
                    STATE_CMD: begin
                        if (sck_rising_reg) begin
                            shift_in <= {shift_in[6:0], mosi_val};
                            bit_cnt  <= bit_cnt + 1'b1;
                        end
                        if (sck_falling_reg && bit_cnt_is_8) begin
                            state    <= STATE_DATA_RX;
                            bit_cnt  <= 4'd0;
                            state_is_data_rx <= 1'b1;
                            
                            if (!shift_in[7]) begin
                                if (shift_in[6:0] == 7'h10) begin
                                    coeff_addr_rst <= 1'b1;
                                end
                            end
                        end
                    end
                    
                    STATE_DATA_RX: begin
                        if (sck_rising_reg) begin
                            shift_in <= {shift_in[6:0], mosi_val};
                            bit_cnt  <= bit_cnt + 1'b1;
                        end
                        if (sck_falling_reg) begin
                            if (bit_cnt_is_8) begin
                                bit_cnt <= 4'd0;
                            end
                        end
                    end
                endcase
            end
        end
    end

    // shift_out update logic (completely independent of state)
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            shift_out <= 8'h00;
        end else if (load_shift_out_pulse) begin
            shift_out <= next_read_byte;
        end else if (sck_falling_reg) begin
            if (is_read && !bit_cnt_is_8) begin
                shift_out <= {shift_out[6:0], 1'b0};
            end
        end
    end

    // MISO Output: Tri-state when CS is high or during write operations
    assign spi_miso = (cs_val || !is_read) ? 1'bZ : shift_out[7];

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            coeff_data <= 16'd0;
        end else begin
            coeff_data <= {shift_in, coeff_lsb};
        end
    end

endmodule
