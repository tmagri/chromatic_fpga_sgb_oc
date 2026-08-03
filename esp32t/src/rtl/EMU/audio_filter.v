module audio_filter
(
	input        reset,
	input        clk,

	input [15:0] core_l,
	input [15:0] core_r,

	output [15:0] filter_l,
	output [15:0] filter_r
);

// Capture stable input (same as original)
reg [15:0] cl, cr;
reg [15:0] cl1, cl2, cr1, cr2;
always @(posedge clk) begin
	cl1 <= core_l; cl2 <= cl1;
	if(cl2 == cl1) cl <= cl2;

	cr1 <= core_r; cr2 <= cr1;
	if(cr2 == cr1) cr <= cr2;
end

// Output sample rate ≈ 65.5 kHz (CLK_RATE / 256)
reg sample_ce;
reg [7:0] div = 0;
always @(posedge clk) begin
	div <= div + 1'd1;
	sample_ce <= !div;
end

// Startup mute (~125 ms)
reg [14:0] dly = 0;
reg a_en = 0;
always @(posedge clk or posedge reset) begin
	if (reset) begin
		dly  <= 0;
		a_en <= 0;
	end else if (sample_ce) begin
		if (!dly[13]) dly <= dly + 1'd1;
		else a_en <= 1;
	end
end

// ----------------------------------------------------------------
// 2-stage cascaded 1-pole IIR low-pass running at full clock rate
// to perform anti-aliasing before downsampling.
// fc = fs * (1/256) / (2 * pi) = 16.777M / 1608 = ~10.4 kHz per pole
// Cascaded -3dB point: ~6.7 kHz
// ----------------------------------------------------------------

wire signed [31:0] cl_ext = {cl, 16'd0};
wire signed [31:0] cr_ext = {cr, 16'd0};

reg signed [31:0] lp1_l = 0, lp2_l = 0;
reg signed [31:0] lp1_r = 0, lp2_r = 0;

always @(posedge clk) begin
	if (reset) begin
		lp1_l <= 0; lp2_l <= 0;
		lp1_r <= 0; lp2_r <= 0;
	end else begin
		lp1_l <= lp1_l + ((cl_ext - lp1_l) >>> 8);
		lp1_r <= lp1_r + ((cr_ext - lp1_r) >>> 8);
		lp2_l <= lp2_l + ((lp1_l - lp2_l) >>> 8);
		lp2_r <= lp2_r + ((lp1_r - lp2_r) >>> 8);
	end
end

// Downsample and apply fractional scaling (0.625x) to match
// the overall gain profile of the original IIR filter (which was ~0.61x).
wire signed [15:0] ds_l = lp2_l[31:16];
wire signed [15:0] ds_r = lp2_r[31:16];

wire signed [15:0] scaled_l = (ds_l >>> 1) + (ds_l >>> 3);
wire signed [15:0] scaled_r = (ds_r >>> 1) + (ds_r >>> 3);

// ----------------------------------------------------------------
// Original high-precision DC Blocker module (0 DSPs)
// ----------------------------------------------------------------

DC_blocker dcb_l
(
	.clk(clk),
	.ce(sample_ce),
	.sample_rate(1'b0),
	.mute(~a_en),
	.din(scaled_l),
	.dout(filter_l)
);

DC_blocker dcb_r
(
	.clk(clk),
	.ce(sample_ce),
	.sample_rate(1'b0),
	.mute(~a_en),
	.din(scaled_r),
	.dout(filter_r)
);

endmodule
