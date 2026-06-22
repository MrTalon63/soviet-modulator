module cic_interpolator #(
    parameter integer WIDTH = 80  // 80-bit internal width prevents overflow
)(
    input  wire               clk,          // Fast system clock (60 MHz)
    input  wire               rstn,         // Active-low reset
    input  wire               in_valid,     // Input sample ready pulse (high for 1 clock)
    input  wire signed [21:0] in_data,      // Signed 22-bit input from RRC
    input  wire        [7:0]  shift_amount, // Dynamic gain shift value from SPI slave
    input  wire               cic_bypass,   // Dynamic mode select: 0 = CIC, 1 = ZOH
    output reg                out_valid,    // Output valid signal
    output reg         [7:0]  out_data      // Scaled 8-bit output sample to DAC
);

    // =========================================================================
    // 1. Comb Stages (running at input rate, on in_valid)
    // =========================================================================
    // Convert 22-bit signed input to 80-bit signed representation (sign extension)
    wire signed [WIDTH-1:0] c0 = {{ (WIDTH-22){in_data[21]} }, in_data};

    reg signed [WIDTH-1:0] c0_d;
    reg signed [WIDTH-1:0] c1_d;
    reg signed [WIDTH-1:0] c2_d;
    reg signed [WIDTH-1:0] c3_d;

    reg signed [WIDTH-1:0] c1;
    reg signed [WIDTH-1:0] c2;
    reg signed [WIDTH-1:0] c3;
    reg signed [WIDTH-1:0] c4;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            c0_d <= {WIDTH{1'b0}};
            c1_d <= {WIDTH{1'b0}};
            c2_d <= {WIDTH{1'b0}};
            c3_d <= {WIDTH{1'b0}};
            c1   <= {WIDTH{1'b0}};
            c2   <= {WIDTH{1'b0}};
            c3   <= {WIDTH{1'b0}};
            c4   <= {WIDTH{1'b0}};
        end else if (watchdog_timeout && !in_valid) begin
            c0_d <= {WIDTH{1'b0}};
            c1_d <= {WIDTH{1'b0}};
            c2_d <= {WIDTH{1'b0}};
            c3_d <= {WIDTH{1'b0}};
            c1   <= {WIDTH{1'b0}};
            c2   <= {WIDTH{1'b0}};
            c3   <= {WIDTH{1'b0}};
            c4   <= {WIDTH{1'b0}};
        end else if (in_valid) begin
            c1   <= c0 - c0_d;
            c0_d <= c0;

            c2   <= c1 - c1_d;
            c1_d <= c1;

            c3   <= c2 - c2_d;
            c2_d <= c2;

            c4   <= c3 - c3_d;
            c3_d <= c3;
        end
    end

    // =========================================================================
    // 2. Zero-Stuffing / Upsampling
    // =========================================================================
    wire signed [WIDTH-1:0] upsampled_val = (in_valid) ? c4 : {WIDTH{1'b0}};

    // =========================================================================
    // 3. Integrator Watchdog & Stages (running at clk output rate)
    // =========================================================================
    reg [19:0] watchdog_cnt;
    reg        watchdog_timeout;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            watchdog_cnt     <= 20'd0;
            watchdog_timeout <= 1'b1;
        end else if (in_valid) begin
            watchdog_cnt     <= 20'd0;
            watchdog_timeout <= 1'b0;
        end else begin
            if (watchdog_cnt == 20'hFFFFF) begin
                watchdog_timeout <= 1'b1;
            end else begin
                watchdog_cnt <= watchdog_cnt + 1'b1;
            end
        end
    end

    reg signed [WIDTH-1:0] i1;
    reg signed [WIDTH-1:0] i2;
    reg signed [WIDTH-1:0] i3;
    reg signed [WIDTH-1:0] i4;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            i1 <= {WIDTH{1'b0}};
            i2 <= {WIDTH{1'b0}};
            i3 <= {WIDTH{1'b0}};
            i4 <= {WIDTH{1'b0}};
        end else if (watchdog_timeout && !in_valid) begin
            i1 <= {WIDTH{1'b0}};
            i2 <= {WIDTH{1'b0}};
            i3 <= {WIDTH{1'b0}};
            i4 <= {WIDTH{1'b0}};
        end else begin
            i1 <= i1 + upsampled_val;
            i2 <= i2 + i1;
            i3 <= i3 + i2;
            i4 <= i4 + i3;
        end
    end

    // Parallel ZOH hold register (updated at RRC filter sample rate)
    reg signed [21:0] hold_data;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            hold_data <= 22'd0;
        end else if (in_valid) begin
            hold_data <= in_data;
        end
    end

    // =========================================================================
    // 4. Output Scaling & Bit-Shift Selection
    // =========================================================================
    // Multiplex between the 4-stage integrator output and the ZOH register
    wire signed [WIDTH-1:0] hold_extended = { { (WIDTH-22){hold_data[21]} }, hold_data };
    wire signed [WIDTH-1:0] interpolator_out = (cic_bypass) ? hold_extended : i4;

    // Perform arithmetic shift right to scale the internal signal to signed 8-bit range
    wire signed [WIDTH-1:0] scaled_i4 = interpolator_out >>> shift_amount;

    reg signed [7:0] clipped_i4;
    always @(*) begin
        if (scaled_i4[WIDTH-1] == 1'b0) begin
            // Positive: if any upper bits are 1, it exceeds 127
            if (|scaled_i4[WIDTH-2:7]) begin
                clipped_i4 = 8'h7F;
            end else begin
                clipped_i4 = scaled_i4[7:0];
            end
        end else begin
            // Negative: if any upper bits are 0, it is less than -128
            if (!(&scaled_i4[WIDTH-2:7])) begin
                clipped_i4 = 8'h80;
            end else begin
                clipped_i4 = scaled_i4[7:0];
            end
        end
    end

    // =========================================================================
    // 5. Output to DAC (Offset Binary)
    // =========================================================================
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            out_valid <= 1'b0;
            out_data  <= 8'h00;
        end else if (watchdog_timeout && !in_valid) begin
            out_valid <= 1'b1;
            out_data  <= 8'h80; // Output clean mid-scale DC bias when clock is stopped
        end else begin
            out_valid <= 1'b1;  // Filter is always generating outputs after reset
            
            // Convert 8-bit signed two's complement to unsigned offset binary
            // by inverting the MSB (sign bit)
            out_data  <= {~clipped_i4[7], clipped_i4[6:0]};
        end
    end

endmodule
