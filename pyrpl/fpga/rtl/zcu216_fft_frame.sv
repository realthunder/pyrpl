/**
 * Chirp framing for one zcu216_fft_dit8 engine (lidar/docs/Zcu216ChirpFft.md).
 *
 * Off (en low): every beat goes in, frame after frame, as before. On: a sync
 * (the laser's chirp sync, one clock per edge) arms a countdown of OFF beats;
 * at zero the engine starts a frame if it is idle (else this chirp's frame is
 * skipped and `skip` pulses): ACQ beats of the input, then zeros up to the
 * frame's 2^SUB beats, all valid, so the realtime xfft cores see an unbroken
 * frame. Between frames nothing goes in (the cores accept gaps there). A new
 * sync restarts a countdown still running (the host keeps OFF below a period).
 *
 * The framer decides the beat to present NEXT clock (valid_d, zero_d: the
 * caller registers them beside the data, so the zeros cost no logic in front
 * of the cores) and counts the beats the engine ACCEPTED (acc: in_accept of the
 * beat presented now): a beat the engine drops (its cores' first-frame config)
 * is made up by the next one, so the engine's frame count and this one never
 * part. Mode changes wait for a frame boundary in that count, so the engine
 * never gets a partial frame.
 */
module zcu216_fft_frame #(
  parameter int  SUB = 13,                        // 2^SUB beats per frame
  parameter int  CW  = 21                         // countdown / window bits (beats)
) (
  input  logic          clk,
  input  logic          rstn,
  input  logic          en,                       // framing on (quasi-static)
  input  logic          sync,                     // one clock per chirp sync
  input  logic [CW-1:0] off,                      // sync -> frame start (beats)
  input  logic [CW-1:0] acq,                      // input beats per frame
  input  logic          acc,                      // the beat presented now was accepted
  output logic          valid_d,                  // present a beat next clock
  output logic          zero_d,                   // ... as zeros (padding)
  output logic          framed,                   // in framed mode
  output logic          skip                      // a frame skipped: engine busy
);

localparam logic [SUB:0] NB = (SUB+1)'(1) << SUB;

logic [SUB:0]  k, k_d;                           // beats accepted in this frame / free count
logic          running, running_d, framed_d;
logic [CW-1:0] cd;                                // countdown to the frame start
logic          pend;                              // countdown running

wire [SUB:0] k_acc = k + (SUB+1)'(acc);
wire         go    = pend && cd == '0;
wire [SUB:0] acq_c = (acq >= CW'(NB)) ? NB : (SUB+1)'(acq);

always_comb begin
  framed_d  = framed;
  running_d = running;
  k_d       = k;
  valid_d   = 1'b0;
  zero_d    = 1'b0;
  if (!framed) begin
    // free-running: every beat; switch at a frame boundary (the beat
    // presented next would be a frame's first)
    k_d = {1'b0, k_acc[SUB-1:0]};
    if (en && k_acc[SUB-1:0] == '0) begin
      framed_d = 1'b1;
      k_d      = '0;
    end else
      valid_d = 1'b1;
  end else if (running) begin
    // a frame: ACQ beats of input, then zeros, NB beats accepted in all
    if (k_acc == NB) begin
      running_d = 1'b0;
      k_d       = '0;
    end else begin
      k_d     = k_acc;
      valid_d = 1'b1;
      zero_d  = k_acc >= acq_c;
    end
  end else if (!en) begin
    // back to free-running, from a boundary
    framed_d = 1'b0;
    k_d      = '0;
    valid_d  = 1'b1;
  end else if (go) begin
    running_d = 1'b1;
    k_d       = '0;
    valid_d   = 1'b1;
    zero_d    = acq_c == '0;
  end
end

always_ff @(posedge clk)
  if (!rstn) begin
    framed  <= 1'b0;
    running <= 1'b0;
    k       <= '0;
    pend    <= 1'b0;
    cd      <= '0;
    skip    <= 1'b0;
  end else begin
    framed  <= framed_d;
    running <= running_d;
    k       <= k_d;
    // countdown from each sync
    if (sync && framed) begin
      pend <= 1'b1;
      cd   <= off;
    end else if (pend) begin
      if (cd == '0) pend <= 1'b0;
      else          cd   <= cd - 1'b1;
    end
    // a frame due while the engine is still busy with the previous one
    skip <= go && framed && running;
  end

endmodule
