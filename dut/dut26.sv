// ================================================================
// pcs_ref_model.sv
//
// Golden reference model for project PCS encoder.
//   - enc_in[8] == 0 : DATA byte on enc_in[7:0]
//   - enc_in[8] == 1 : IDLE / no active packet
//   - SSD, CSReset, ESD, etc. are derived internally from tx_enable
// ================================================================
class pcs_ref_model;

  // ------------------------------------------------------------
  // Four quinary symbols before/after sign randomization.
  // Each symbol is represented as signed value: -2,-1,0,+1,+2.
  // ------------------------------------------------------------
  typedef struct {
    logic signed [2:0] a;
    logic signed [2:0] b;
    logic signed [2:0] c;
    logic signed [2:0] d;
  } qsym_t;

  typedef enum int {
    ROW_NORMAL,
    ROW_XMT_ERR,
    ROW_CSRESET,
    ROW_CSEXTEND,
    ROW_CSEXTEND_ERR,
    ROW_SSD1,
    ROW_SSD2,
    ROW_ESD1,
    ROW_ESD2_EXT_0,
    ROW_ESD2_EXT_1,
    ROW_ESD2_EXT_2,
    ROW_ESD_EXT_ERR
  } row_sel_e;

  // ------------------------------------------------------------
  // Configuration of SEND_Z signal as mentioned
  // ------------------------------------------------------------
  bit send_z_mode;

  // ------------------------------------------------------------
  // Persistent encoder state
  // ------------------------------------------------------------
  logic [32:0] scr;       // side-stream scrambler state
  logic [2:0]  cs;        // convolutional encoder state

  // Delayed tx_enable history.
  // tx_enable_d1 = tx_enable_(n-1), etc.
  logic tx_enable_d1;
  logic tx_enable_d2;
  logic tx_enable_d3;
  logic tx_enable_d4;

  // Delayed tx_error history.
  logic tx_error_d1;
  logic tx_error_d2;
  logic tx_error_d3;

  // Delayed data, useful for ESD/carrier-extension cases.
  logic [7:0] txd_d1;
  logic [7:0] txd_d2;
  logic [7:0] txd_d3;
  logic [7:0] txd_d4;

  // Previous Syn value, used by Scn generation.
  logic [3:0] sy_d1;

  // Symbol counter used for simple alternating Scn behavior.
  int unsigned sym_count;

  // ------------------------------------------------------------
  // Constructor
  // ------------------------------------------------------------
  function new();
    reset();
  endfunction

  // ------------------------------------------------------------
  // Configure model from env config values.
  // You can call this from scoreboard build/connect phase.
  // ------------------------------------------------------------
  function void configure(
    input logic [32:0] seed,
    input bit          send_z    = 1'b0
  );
    send_z_mode = send_z;
    reset(seed);
  endfunction

  // ------------------------------------------------------------
  // Reset model state.
  // Scrambler seed must not be all zero.
  // ------------------------------------------------------------
  function void reset(input logic [32:0] seed = 33'h1);
    send_z_mode = 1'b0;

    if (seed == 33'h0)
      scr = 33'h1;
    else
      scr = seed;

    cs = 3'b000;

    tx_enable_d1 = 1'b0;
    tx_enable_d2 = 1'b0;
    tx_enable_d3 = 1'b0;
    tx_enable_d4 = 1'b0;

    tx_error_d1 = 1'b0;
    tx_error_d2 = 1'b0;
    tx_error_d3 = 1'b0;

    txd_d1 = 8'h00;
    txd_d2 = 8'h00;
    txd_d3 = 8'h00;
    txd_d4 = 8'h00;

    sy_d1 = 4'h0;
    sym_count = 0;
  endfunction

  // ------------------------------------------------------------
  // Top-level API used by scoreboard.
  //
  // enc_in[8] = 0 -> data byte
  // enc_in[8] = 1 -> idle/no-active-packet
  //
  // tx_error is optional for future error-injection support.
  // For your current pcs_env_config, tx_err_en is constrained to 0.
  // ------------------------------------------------------------
  function logic [11:0] encode(
    input logic [8:0] enc_in,
    input bit         tx_error = 1'b0
  );
    logic        tx_enable;
    logic [7:0]  txd;

    logic [3:0] sy;
    logic [3:0] sx;
    logic [3:0] sg;
    logic [7:0] scn;
    logic [8:0] sdn;
    logic [2:0] cs_next;

    row_sel_e row;
    qsym_t    t_symbols;
    qsym_t    signed_symbols;
    logic [11:0] packed_out;

    tx_enable = (enc_in[8] == 1'b0);
    txd       = enc_in[7:0];

    // Step 1/2: helper bits from scrambler state.
    helper_bits(scr, sy, sx, sg);

    // Step 3: build Scn[7:0].
    scn = build_scn(sy, sx);

    // Step 4: build Sdn[8:0] and next convolutional state.
    sdn = build_sdn(txd, tx_enable, scn, cs_next);

    // Step 5A: select which row family to use.
    row = select_row(tx_enable, tx_error, txd);

    // Step 5B: map into pre-sign quinary symbols.
    t_symbols = map_symbols(row, sdn, tx_enable);

    // Step 6: apply sign randomization.
    signed_symbols = apply_signs(t_symbols, sg);

    // Project output: pack four quinary symbols into 12 bits.
    packed_out = pack_symbols(signed_symbols);

    // State update happens after output prediction.
    update_state(tx_enable, tx_error, txd, sy, cs_next);

    return packed_out;
  endfunction

  // ------------------------------------------------------------
  // Step 1/2: generate Sy, Sx, Sg helper bits.
  // ------------------------------------------------------------
  function void helper_bits(
    input  logic [32:0] scrn,
    output logic [3:0]  sy,
    output logic [3:0]  sx,
    output logic [3:0]  sg
  );
    sy[0] = scrn[0];
    sy[1] = scrn[3] ^ scrn[8];
    sy[2] = scrn[6] ^ scrn[16];
    sy[3] = scrn[9] ^ scrn[14] ^ scrn[19] ^ scrn[24];

    sx[0] = scrn[4] ^ scrn[6];
    sx[1] = scrn[7] ^ scrn[9] ^ scrn[12] ^ scrn[14];
    sx[2] = scrn[10] ^ scrn[12] ^ scrn[20] ^ scrn[22];
    sx[3] = scrn[13] ^ scrn[15] ^ scrn[18] ^ scrn[20] ^
            scrn[23] ^ scrn[25] ^ scrn[28] ^ scrn[30];

    sg[0] = scrn[1] ^ scrn[5];
    sg[1] = scrn[4] ^ scrn[8] ^ scrn[9] ^ scrn[13];
    sg[2] = scrn[7] ^ scrn[11] ^ scrn[17] ^ scrn[21];
    sg[3] = scrn[10] ^ scrn[14] ^ scrn[15] ^ scrn[19] ^
            scrn[20] ^ scrn[24] ^ scrn[25] ^ scrn[29];
  endfunction

  // ------------------------------------------------------------
  // Step 3: build Scn[7:0].
  //
  // This keeps the Scn behavior isolated so we can refine it
  // directly against 40.3.1.3.3 if needed.
  // ------------------------------------------------------------
  function logic [7:0] build_scn(
    input logic [3:0] sy,
    input logic [3:0] sx
  );
    logic [7:0] scn;

    // Matches the common form:
    // upper nibble is active based on tx_enable_(n-2).
    scn[7:4] = tx_enable_d2 ? sx : 4'b0000;

    if (send_z_mode) begin
      scn[3:1] = 3'b000;
      scn[0]   = 1'b0;
    end
    else begin
      // Alternating Syn behavior. This is intentionally isolated.
      if (sym_count[0] == 1'b0)
        scn[3:1] = sy[3:1];
      else
        scn[3:1] = sy_d1[3:1] ^ 3'b111;

      scn[0] = sy[0];
    end

    return scn;
  endfunction

  // ------------------------------------------------------------
  // Step 4: build Sdn[8:0].
  //
  // For data cycles, Sdn[7:0] is the scrambled data byte.
  // Sdn[8] comes from convolutional state.
  // cs_next is returned separately and committed at end of cycle.
  // ------------------------------------------------------------
function logic [8:0] build_sdn(
  input  logic [7:0] txd,
  input  logic       tx_enable,
  input  logic [7:0] scn,
  output logic [2:0] cs_next
);
  logic [8:0] sdn;
  logic       csreset;

  csreset = tx_enable_d2 & ~tx_enable;

  // ------------------------------------------------------------
  // Sdn[7:6]
  //
  // Spec behavior:
  // If csreset = 0 and tx_enable_(n-2)=1:
  //   Sdn[7:6] = Scn[7:6] ^ TXD[7:6]
  //
  // If csreset = 1:
  //   Sdn[7] = cs(n-1)[1]
  //   Sdn[6] = cs(n-1)[0]
  //
  // Else:
  //   Sdn[7:6] = Scn[7:6]
  // ------------------------------------------------------------
  if ((csreset == 1'b0) && (tx_enable_d2 == 1'b1)) begin
    sdn[7] = scn[7] ^ txd[7];
    sdn[6] = scn[6] ^ txd[6];
  end
  else if (csreset == 1'b1) begin
    sdn[7] = cs[1];
    sdn[6] = cs[0];
  end
  else begin
    sdn[7] = scn[7];
    sdn[6] = scn[6];
  end

  // ------------------------------------------------------------
  // Sdn[5:4]
  // ------------------------------------------------------------
  if (tx_enable_d2)
    sdn[5:4] = scn[5:4] ^ txd[5:4];
  else
    sdn[5:4] = scn[5:4];

  // ------------------------------------------------------------
  // Simplified project handling for Sdn[3:0].
  //
  // The real spec gives separate definitions for Sdn[3],
  // Sdn[2], Sdn[1], and Sdn[0] depending on data/control fields.
  // For the project model, this is acceptable until we refine
  // loc_lpi_req, loc_rcvr_status, cext, etc.
  // ------------------------------------------------------------
  if (tx_enable_d2)
    sdn[3:0] = scn[3:0] ^ txd[3:0];
  else
    sdn[3:0] = scn[3:0];

  // ------------------------------------------------------------
  // Convolutional encoder state.
  //
  // This is the part you correctly noticed:
  // csn[0] = cs(n-1)[2] is unconditional.
  // ------------------------------------------------------------
  cs_next[0] = cs[2];

  if (tx_enable_d2) begin
    cs_next[1] = sdn[6] ^ cs[0];
    cs_next[2] = sdn[7] ^ cs[1];
  end
  else begin
    cs_next[1] = 1'b0;
    cs_next[2] = 1'b0;
  end

  // Sdn[8] comes from csn[0]
  sdn[8] = cs_next[0];

  return sdn;
endfunction

  // ------------------------------------------------------------
  // Step 5A: determine which table row should be selected.
  //
  // ------------------------------------------------------------
function row_sel_e select_row(
  input logic       tx_enable,
  input logic       tx_error,
  input logic [7:0] txd
);
  logic csreset;

  logic ssdn;
  logic ssd1_cond;
  logic ssd2_cond;

  logic esdn;
  logic esd1_cond;
  logic esd2_cond;

  logic carrier_ext_err_3;
  logic carrier_ext_err_4;
  logic esd_ext_err_now;

  // ------------------------------------------------------------
  // Convolutional encoder reset condition:
  // csresetn = tx_enable_(n-2) * !tx_enable_n
  // ------------------------------------------------------------
  csreset = tx_enable_d2 & ~tx_enable;

  // ------------------------------------------------------------
  // SSDn = tx_enable_n * !tx_enable_(n-2)
  // This covers both SSD1 and SSD2.
  // ------------------------------------------------------------
  ssdn = tx_enable & ~tx_enable_d2;

  ssd1_cond = tx_enable & ~tx_enable_d1;
  ssd2_cond = tx_enable_d1 & ~tx_enable_d2;

  // ------------------------------------------------------------
  // ESDn = !tx_enable_(n-2) * tx_enable_(n-4)
  // This covers the third and fourth symbol periods after the
  // last data octet.
  // ------------------------------------------------------------
  esdn = (~tx_enable_d2) & tx_enable_d4;

  esd1_cond = (~tx_enable_d2) & tx_enable_d3;
  esd2_cond = (~tx_enable_d3) & tx_enable_d4;

  // ------------------------------------------------------------
  // Carrier extension error conditions during ESD.
  // The spec gives two conditions:
  //   tx_error_n * tx_error_(n-1) * tx_error_(n-2) *
  //   (TXD_n != 0x0F)
  //
  // and:
  //   tx_error_n * tx_error_(n-1) * tx_error_(n-2) *
  //   tx_error_(n-3) * (TXD_n != 0x0F)
  // ------------------------------------------------------------
  carrier_ext_err_3 =
    tx_error & tx_error_d1 & tx_error_d2 & (txd != 8'h0F);

  carrier_ext_err_4 =
    tx_error & tx_error_d1 & tx_error_d2 & tx_error_d3 &
    (txd != 8'h0F);

  esd_ext_err_now = carrier_ext_err_3 | carrier_ext_err_4;

  // ------------------------------------------------------------
  // Active transmit error during real data.
  // tx_enable_d2 prevents SSD1/SSD2 cycles from being treated
  // as normal transmit-error data cycles.
  // ------------------------------------------------------------
  if (tx_error && tx_enable && tx_enable_d2)
    return ROW_XMT_ERR;

  // ------------------------------------------------------------
  // Start-of-Stream delimiter region.
  // ------------------------------------------------------------
  if (ssdn) begin
    if (ssd1_cond)
      return ROW_SSD1;

    if (ssd2_cond)
      return ROW_SSD2;
  end

  // ------------------------------------------------------------
  // Convolutional encoder reset / carrier extension region.
  // ------------------------------------------------------------
  if (csreset && !tx_error)
    return ROW_CSRESET;

  if (csreset && tx_error && (txd == 8'h0F))
    return ROW_CSEXTEND;

  if (csreset && tx_error && (txd != 8'h0F))
    return ROW_CSEXTEND_ERR;

  // ------------------------------------------------------------
  // End-of-Stream delimiter region.
  // ------------------------------------------------------------
  if (esdn) begin

    if (esd_ext_err_now)
      return ROW_ESD_EXT_ERR;

    if (esd1_cond)
      return ROW_ESD1;

    if (esd2_cond && (!tx_error) && (!tx_error_d1))
      return ROW_ESD2_EXT_0;

    if (esd2_cond &&
        (!tx_error) && tx_error_d1 && tx_error_d2 && tx_error_d3)
      return ROW_ESD2_EXT_1;

    if (esd2_cond &&
        tx_error && tx_error_d1 && tx_error_d2 && tx_error_d3 &&
        (txd == 8'h0F))
      return ROW_ESD2_EXT_2;
  end

  return ROW_NORMAL;
endfunction

  // ------------------------------------------------------------
  // Step 5B: map Sdn/row into pre-sign quinary symbols.
  // ------------------------------------------------------------
  function qsym_t map_symbols(
    input row_sel_e   row,
    input logic [8:0] sdn,
    input logic       tx_enable
  );
    logic [2:0] hbits = {<< {sdn[8:6]}};
    if (row != ROW_NORMAL)
      return map_special_row(row, hbits); //flipping the bits as table uses flipped bits Sdn[6:8]

    if (!tx_enable)
      return map_idle_carrier_extension(sdn[5:0]);

    return map_normal_data(sdn);
  endfunction

  // ------------------------------------------------------------
  // Special rows from Table 40-1 / Table 40-2.
  //
  // SSD/ESD rows are fixed.
  // xmt_err / CSReset / CSExtend rows vary by subset.
  // subset is modeled as sdn[6:8] in this project model.
  // ------------------------------------------------------------
  function qsym_t map_special_row(
    input row_sel_e   row,
    input logic [2:0] subset
  );

    case (row)

      ROW_SSD1:
        return make_qsym( 2,  2,  2,  2);

      ROW_SSD2:
        return make_qsym( 2,  2,  2, -2);

      ROW_ESD1:
        return make_qsym( 2,  2,  2,  2);

      ROW_ESD2_EXT_0:
        return make_qsym( 2,  2,  2, -2);

      ROW_ESD2_EXT_1:
        return make_qsym( 2,  2, -2,  2);

      ROW_ESD2_EXT_2:
        return make_qsym( 2, -2,  2,  2);

      ROW_ESD_EXT_ERR:
        return make_qsym(-2,  2,  2,  2);

      ROW_XMT_ERR: begin
        case (subset)
          // Table 40-1 even subsets
          3'b000: return make_qsym( 0,  2,  2,  0);
          3'b010: return make_qsym( 1,  1,  2,  2);
          3'b100: return make_qsym( 2,  1,  1,  2);
          3'b110: return make_qsym( 2,  1,  2,  1);

          // Table 40-2 odd subsets
          3'b001: return make_qsym( 2,  2,  0,  1);
          3'b011: return make_qsym( 0,  2,  1,  2);
          3'b101: return make_qsym( 1,  2,  2,  0);
          3'b111: return make_qsym( 2,  1,  2,  0);

          default: return make_qsym(0, 0, 0, 0);
        endcase
      end

      ROW_CSEXTEND_ERR: begin
        case (subset)
          // Table 40-1 even subsets
          3'b000: return make_qsym(-2,  2,  2, -2);
          3'b010: return make_qsym(-1, -1,  2,  2);
          3'b100: return make_qsym( 2, -1, -1,  2);
          3'b110: return make_qsym( 2, -1,  2, -1);

          // Table 40-2 odd subsets
          3'b001: return make_qsym( 2,  2, -2, -1);
          3'b011: return make_qsym(-2,  2, -1,  2);
          3'b101: return make_qsym(-1,  2,  2, -2);
          3'b111: return make_qsym( 2, -1,  2, -2);

          default: return make_qsym(0, 0, 0, 0);
        endcase
      end

      ROW_CSEXTEND: begin
        case (subset)
          // Table 40-1 even subsets
          3'b000: return make_qsym( 2,  0,  0,  2);
          3'b010: return make_qsym( 2,  2,  1,  1);
          3'b100: return make_qsym( 1,  2,  2,  1);
          3'b110: return make_qsym( 1,  2,  1,  2);

          // Table 40-2 odd subsets
          3'b001: return make_qsym( 2,  0,  2,  1);
          3'b011: return make_qsym( 2,  0,  1,  2);
          3'b101: return make_qsym( 1,  0,  2,  2);
          3'b111: return make_qsym( 2,  1,  0,  2);

          default: return make_qsym(0, 0, 0, 0);
        endcase
      end

      ROW_CSRESET: begin
        case (subset)
          // Table 40-1 even subsets
          3'b000: return make_qsym( 2, -2, -2,  2);
          3'b010: return make_qsym( 2,  2, -1, -1);
          3'b100: return make_qsym(-1,  2,  2, -1);
          3'b110: return make_qsym(-1,  2, -1,  2);

          // Table 40-2 odd subsets
          3'b001: return make_qsym( 2, -2,  2, -1);
          3'b011: return make_qsym( 2, -2, -1,  2);
          3'b101: return make_qsym(-1, -2,  2,  2);
          3'b111: return make_qsym( 2, -1, -2,  2);

          default: return make_qsym(0, 0, 0, 0);
        endcase
      end

      default:
        return make_qsym(0, 0, 0, 0);

    endcase
  endfunction

  // ------------------------------------------------------------
  // Idle / carrier-extension subset mapping.
  //
  // This covers the simple Idle/Carrier Extension rows visible
  // in Table 40-1 for subset 000.
  // ------------------------------------------------------------
  function qsym_t map_idle_carrier_extension(input logic [5:0] sdn_lsb);
    qsym_t q;

    q.a = sdn_lsb[0] ? sym(-2) : sym(0);
    q.b = sdn_lsb[1] ? sym(-2) : sym(0);
    q.c = sdn_lsb[2] ? sym(-2) : sym(0);
    q.d = sdn_lsb[3] ? sym(-2) : sym(0);

    return q;
  endfunction

  // ------------------------------------------------------------
  // Normal data mapper.
  // This is filled from Table 40-1 and Table 40-2 for
  // full spec-accurate checking.
  // ------------------------------------------------------------

  function qsym_t map_normal_data(input logic [8:0] sdn);
    //   case ({sdn[6:8], sdn[5:0]})
    //     9'b000_000000: return make_qsym(...);
    //     ...
    //   endcase

    case({sdn[8:6], sdn[5:0]})
              9'b000000000: return make_qsym(0, 0, 0, 0);
        9'b010000000: return make_qsym(0, 0, 1, 1);
        9'b001000000: return make_qsym(0, 1, 1, 0);
        9'b011000000: return make_qsym(0, 1, 0, 1);
        9'b000000001: return make_qsym(-2, 0, 0, 0);
        9'b010000001: return make_qsym(-2, 0, 1, 1);
        9'b001000001: return make_qsym(-2, 1, 1, 0);
        9'b011000001: return make_qsym(-2, 1, 0, 1);
        9'b000000010: return make_qsym(0, -2, 0, 0);
        9'b010000010: return make_qsym(0, -2, 1, 1);
        9'b001000010: return make_qsym(0, -1, 1, 0);
        9'b011000010: return make_qsym(0, -1, 0, 1);
        9'b000000011: return make_qsym(-2, -2, 0, 0);
        9'b010000011: return make_qsym(-2, -2, 1, 1);
        9'b001000011: return make_qsym(-2, -1, 1, 0);
        9'b011000011: return make_qsym(-2, -1, 0, 1);
        9'b000000100: return make_qsym(0, 0, -2, 0);
        9'b010000100: return make_qsym(0, 0, -1, 1);
        9'b001000100: return make_qsym(0, 1, -1, 0);
        9'b011000100: return make_qsym(0, 1, -2, 1);
        9'b000000101: return make_qsym(-2, 0, -2, 0);
        9'b010000101: return make_qsym(-2, 0, -1, 1);
        9'b001000101: return make_qsym(-2, 1, -1, 0);
        9'b011000101: return make_qsym(-2, 1, -2, 1);
        9'b000000110: return make_qsym(0, -2, -2, 0);
        9'b010000110: return make_qsym(0, -2, -1, 1);
        9'b001000110: return make_qsym(0, -1, -1, 0);
        9'b011000110: return make_qsym(0, -1, -2, 1);
        9'b000000111: return make_qsym(-2, -2, -2, 0);
        9'b010000111: return make_qsym(-2, -2, -1, 1);
        9'b001000111: return make_qsym(-2, -1, -1, 0);
        9'b011000111: return make_qsym(-2, -1, -2, 1);
        9'b000001000: return make_qsym(0, 0, 0, -2);
        9'b010001000: return make_qsym(0, 0, 1, -1);
        9'b001001000: return make_qsym(0, 1, 1, -2);
        9'b011001000: return make_qsym(0, 1, 0, -1);
        9'b000001001: return make_qsym(-2, 0, 0, -2);
        9'b010001001: return make_qsym(-2, 0, 1, -1);
        9'b001001001: return make_qsym(-2, 1, 1, -2);
        9'b011001001: return make_qsym(-2, 1, 0, -1);
        9'b000001010: return make_qsym(0, -2, 0, -2);
        9'b010001010: return make_qsym(0, -2, 1, -1);
        9'b001001010: return make_qsym(0, -1, 1, -2);
        9'b011001010: return make_qsym(0, -1, 0, -1);
        9'b000001011: return make_qsym(-2, -2, 0, -2);
        9'b010001011: return make_qsym(-2, -2, 1, -1);
        9'b001001011: return make_qsym(-2, -1, 1, -2);
        9'b011001011: return make_qsym(-2, -1, 0, -1);
        9'b000001100: return make_qsym(0, 0, -2, -2);
        9'b010001100: return make_qsym(0, 0, -1, -1);
        9'b001001100: return make_qsym(0, 1, -1, -2);
        9'b011001100: return make_qsym(0, 1, -2, -1);
        9'b000001101: return make_qsym(-2, 0, -2, -2);
        9'b010001101: return make_qsym(-2, 0, -1, -1);
        9'b001001101: return make_qsym(-2, 1, -1, -2);
        9'b011001101: return make_qsym(-2, 1, -2, -1);
        9'b000001110: return make_qsym(0, -2, -2, -2);
        9'b010001110: return make_qsym(0, -2, -1, -1);
        9'b001001110: return make_qsym(0, -1, -1, -2);
        9'b011001110: return make_qsym(0, -1, -2, -1);
        9'b000001111: return make_qsym(-2, -2, -2, -2);
        9'b010001111: return make_qsym(-2, -2, -1, -1);
        9'b001001111: return make_qsym(-2, -1, -1, -2);
        9'b011001111: return make_qsym(-2, -1, -2, -1);
        9'b000010000: return make_qsym(1, 1, 1, 1);
        9'b010010000: return make_qsym(1, 1, 0, 0);
        9'b001010000: return make_qsym(1, 0, 0, 1);
        9'b011010000: return make_qsym(1, 0, 1, 0);
        9'b000010001: return make_qsym(-1, 1, 1, 1);
        9'b010010001: return make_qsym(-1, 1, 0, 0);
        9'b001010001: return make_qsym(-1, 0, 0, 1);
        9'b011010001: return make_qsym(-1, 0, 1, 0);
        9'b000010010: return make_qsym(1, -1, 1, 1);
        9'b010010010: return make_qsym(1, -1, 0, 0);
        9'b001010010: return make_qsym(1, -2, 0, 1);
        9'b011010010: return make_qsym(1, -2, 1, 0);
        9'b000010011: return make_qsym(-1, -1, 1, 1);
        9'b010010011: return make_qsym(-1, -1, 0, 0);
        9'b001010011: return make_qsym(-1, -2, 0, 1);
        9'b011010011: return make_qsym(-1, -2, 1, 0);
        9'b000010100: return make_qsym(1, 1, -1, 1);
        9'b010010100: return make_qsym(1, 1, -2, 0);
        9'b001010100: return make_qsym(1, 0, -2, 1);
        9'b011010100: return make_qsym(1, 0, -1, 0);
        9'b000010101: return make_qsym(-1, 1, -1, 1);
        9'b010010101: return make_qsym(-1, 1, -2, 0);
        9'b001010101: return make_qsym(-1, 0, -2, 1);
        9'b011010101: return make_qsym(-1, 0, -1, 0);
        9'b000010110: return make_qsym(1, -1, -1, 1);
        9'b010010110: return make_qsym(1, -1, -2, 0);
        9'b001010110: return make_qsym(1, -2, -2, 1);
        9'b011010110: return make_qsym(1, -2, -1, 0);
        9'b000010111: return make_qsym(-1, -1, -1, 1);
        9'b010010111: return make_qsym(-1, -1, -2, 0);
        9'b001010111: return make_qsym(-1, -2, -2, 1);
        9'b011010111: return make_qsym(-1, -2, -1, 0);
        9'b000011000: return make_qsym(1, 1, 1, -1);
        9'b010011000: return make_qsym(1, 1, 0, -2);
        9'b001011000: return make_qsym(1, 0, 0, -1);
        9'b011011000: return make_qsym(1, 0, 1, -2);
        9'b000011001: return make_qsym(-1, 1, 1, -1);
        9'b010011001: return make_qsym(-1, 1, 0, -2);
        9'b001011001: return make_qsym(-1, 0, 0, -1);
        9'b011011001: return make_qsym(-1, 0, 1, -2);
        9'b000011010: return make_qsym(1, -1, 1, -1);
        9'b010011010: return make_qsym(1, -1, 0, -2);
        9'b001011010: return make_qsym(1, -2, 0, -1);
        9'b011011010: return make_qsym(1, -2, 1, -2);
        9'b000011011: return make_qsym(-1, -1, 1, -1);
        9'b010011011: return make_qsym(-1, -1, 0, -2);
        9'b001011011: return make_qsym(-1, -2, 0, -1);
        9'b011011011: return make_qsym(-1, -2, 1, -2);
        9'b000011100: return make_qsym(1, 1, -1, -1);
        9'b010011100: return make_qsym(1, 1, -2, -2);
        9'b001011100: return make_qsym(1, 0, -2, -1);
        9'b011011100: return make_qsym(1, 0, -1, -2);
        9'b000011101: return make_qsym(-1, 1, -1, -1);
        9'b010011101: return make_qsym(-1, 1, -2, -2);
        9'b001011101: return make_qsym(-1, 0, -2, -1);
        9'b011011101: return make_qsym(-1, 0, -1, -2);
        9'b000011110: return make_qsym(1, -1, -1, -1);
        9'b010011110: return make_qsym(1, -1, -2, -2);
        9'b001011110: return make_qsym(1, -2, -2, -1);
        9'b011011110: return make_qsym(1, -2, -1, -2);
        9'b000011111: return make_qsym(-1, -1, -1, -1);
        9'b010011111: return make_qsym(-1, -1, -2, -2);
        9'b001011111: return make_qsym(-1, -2, -2, -1);
        9'b011011111: return make_qsym(-1, -2, -1, -2);
        9'b000100000: return make_qsym(2, 0, 0, 0);
        9'b010100000: return make_qsym(2, 0, 1, 1);
        9'b001100000: return make_qsym(2, 1, 1, 0);
        9'b011100000: return make_qsym(2, 1, 0, 1);
        9'b000100001: return make_qsym(2, -2, 0, 0);
        9'b010100001: return make_qsym(2, -2, 1, 1);
        9'b001100001: return make_qsym(2, -1, 1, 0);
        9'b011100001: return make_qsym(2, -1, 0, 1);
        9'b000100010: return make_qsym(2, 0, -2, 0);
        9'b010100010: return make_qsym(2, 0, -1, 1);
        9'b001100010: return make_qsym(2, 1, -1, 0);
        9'b011100010: return make_qsym(2, 1, -2, 1);
        9'b000100011: return make_qsym(2, -2, -2, 0);
        9'b010100011: return make_qsym(2, -2, -1, 1);
        9'b001100011: return make_qsym(2, -1, -1, 0);
        9'b011100011: return make_qsym(2, -1, -2, 1);
        9'b000100100: return make_qsym(2, 0, 0, -2);
        9'b010100100: return make_qsym(2, 0, 1, -1);
        9'b001100100: return make_qsym(2, 1, 1, -2);
        9'b011100100: return make_qsym(2, 1, 0, -1);
        9'b000100101: return make_qsym(2, -2, 0, -2);
        9'b010100101: return make_qsym(2, -2, 1, -1);
        9'b001100101: return make_qsym(2, -1, 1, -2);
        9'b011100101: return make_qsym(2, -1, 0, -1);
        9'b000100110: return make_qsym(2, 0, -2, -2);
        9'b010100110: return make_qsym(2, 0, -1, -1);
        9'b001100110: return make_qsym(2, 1, -1, -2);
        9'b011100110: return make_qsym(2, 1, -2, -1);
        9'b000100111: return make_qsym(2, -2, -2, -2);
        9'b010100111: return make_qsym(2, -2, -1, -1);
        9'b001100111: return make_qsym(2, -1, -1, -2);
        9'b011100111: return make_qsym(2, -1, -2, -1);
        9'b000101000: return make_qsym(0, 0, 2, 0);
        9'b010101000: return make_qsym(1, 1, 2, 0);
        9'b001101000: return make_qsym(1, 0, 2, 1);
        9'b011101000: return make_qsym(0, 1, 2, 1);
        9'b000101001: return make_qsym(-2, 0, 2, 0);
        9'b010101001: return make_qsym(-1, 1, 2, 0);
        9'b001101001: return make_qsym(-1, 0, 2, 1);
        9'b011101001: return make_qsym(-2, 1, 2, 1);
        9'b000101010: return make_qsym(0, -2, 2, 0);
        9'b010101010: return make_qsym(1, -1, 2, 0);
        9'b001101010: return make_qsym(1, -2, 2, 1);
        9'b011101010: return make_qsym(0, -1, 2, 1);
        9'b000101011: return make_qsym(-2, -2, 2, 0);
        9'b010101011: return make_qsym(-1, -1, 2, 0);
        9'b001101011: return make_qsym(-1, -2, 2, 1);
        9'b011101011: return make_qsym(-2, -1, 2, 1);
        9'b000101100: return make_qsym(0, 0, 2, -2);
        9'b010101100: return make_qsym(1, 1, 2, -2);
        9'b001101100: return make_qsym(1, 0, 2, -1);
        9'b011101100: return make_qsym(0, 1, 2, -1);
        9'b000101101: return make_qsym(-2, 0, 2, -2);
        9'b010101101: return make_qsym(-1, 1, 2, -2);
        9'b001101101: return make_qsym(-1, 0, 2, -1);
        9'b011101101: return make_qsym(-2, 1, 2, -1);
        9'b000101110: return make_qsym(0, -2, 2, -2);
        9'b010101110: return make_qsym(1, -1, 2, -2);
        9'b001101110: return make_qsym(1, -2, 2, -1);
        9'b011101110: return make_qsym(0, -1, 2, -1);
        9'b000101111: return make_qsym(-2, -2, 2, -2);
        9'b010101111: return make_qsym(-1, -1, 2, -2);
        9'b001101111: return make_qsym(-1, -2, 2, -1);
        9'b011101111: return make_qsym(-2, -1, 2, -1);
        9'b000110000: return make_qsym(0, 2, 0, 0);
        9'b010110000: return make_qsym(0, 2, 1, 1);
        9'b001110000: return make_qsym(1, 2, 0, 1);
        9'b011110000: return make_qsym(1, 2, 1, 0);
        9'b000110001: return make_qsym(-2, 2, 0, 0);
        9'b010110001: return make_qsym(-2, 2, 1, 1);
        9'b001110001: return make_qsym(-1, 2, 0, 1);
        9'b011110001: return make_qsym(-1, 2, 1, 0);
        9'b000110010: return make_qsym(0, 2, -2, 0);
        9'b010110010: return make_qsym(0, 2, -1, 1);
        9'b001110010: return make_qsym(1, 2, -2, 1);
        9'b011110010: return make_qsym(1, 2, -1, 0);
        9'b000110011: return make_qsym(-2, 2, -2, 0);
        9'b010110011: return make_qsym(-2, 2, -1, 1);
        9'b001110011: return make_qsym(-1, 2, -2, 1);
        9'b011110011: return make_qsym(-1, 2, -1, 0);
        9'b000110100: return make_qsym(0, 2, 0, -2);
        9'b010110100: return make_qsym(0, 2, 1, -1);
        9'b001110100: return make_qsym(1, 2, 0, -1);
        9'b011110100: return make_qsym(1, 2, 1, -2);
        9'b000110101: return make_qsym(-2, 2, 0, -2);
        9'b010110101: return make_qsym(-2, 2, 1, -1);
        9'b001110101: return make_qsym(-1, 2, 0, -1);
        9'b011110101: return make_qsym(-1, 2, 1, -2);
        9'b000110110: return make_qsym(0, 2, -2, -2);
        9'b010110110: return make_qsym(0, 2, -1, -1);
        9'b001110110: return make_qsym(1, 2, -2, -1);
        9'b011110110: return make_qsym(1, 2, -1, -2);
        9'b000110111: return make_qsym(-2, 2, -2, -2);
        9'b010110111: return make_qsym(-2, 2, -1, -1);
        9'b001110111: return make_qsym(-1, 2, -2, -1);
        9'b011110111: return make_qsym(-1, 2, -1, -2);
        9'b000111000: return make_qsym(0, 0, 0, 2);
        9'b010111000: return make_qsym(1, 1, 0, 2);
        9'b001111000: return make_qsym(0, 1, 1, 2);
        9'b011111000: return make_qsym(1, 0, 1, 2);
        9'b000111001: return make_qsym(-2, 0, 0, 2);
        9'b010111001: return make_qsym(-1, 1, 0, 2);
        9'b001111001: return make_qsym(-2, 1, 1, 2);
        9'b011111001: return make_qsym(-1, 0, 1, 2);
        9'b000111010: return make_qsym(0, -2, 0, 2);
        9'b010111010: return make_qsym(1, -1, 0, 2);
        9'b001111010: return make_qsym(0, -1, 1, 2);
        9'b011111010: return make_qsym(1, -2, 1, 2);
        9'b000111011: return make_qsym(-2, -2, 0, 2);
        9'b010111011: return make_qsym(-1, -1, 0, 2);
        9'b001111011: return make_qsym(-2, -1, 1, 2);
        9'b011111011: return make_qsym(-1, -2, 1, 2);
        9'b000111100: return make_qsym(0, 0, -2, 2);
        9'b010111100: return make_qsym(1, 1, -2, 2);
        9'b001111100: return make_qsym(0, 1, -1, 2);
        9'b011111100: return make_qsym(1, 0, -1, 2);
        9'b000111101: return make_qsym(-2, 0, -2, 2);
        9'b010111101: return make_qsym(-1, 1, -2, 2);
        9'b001111101: return make_qsym(-2, 1, -1, 2);
        9'b011111101: return make_qsym(-1, 0, -1, 2);
        9'b000111110: return make_qsym(0, -2, -2, 2);
        9'b010111110: return make_qsym(1, -1, -2, 2);
        9'b001111110: return make_qsym(0, -1, -1, 2);
        9'b011111110: return make_qsym(1, -2, -1, 2);
        9'b000111111: return make_qsym(-2, -2, -2, 2);
        9'b010111111: return make_qsym(-1, -1, -2, 2);
        9'b001111111: return make_qsym(-2, -1, -1, 2);
        9'b011111111: return make_qsym(-1, -2, -1, 2);
        9'b100000000: return make_qsym(0, 0, 0, 1);
        9'b110000000: return make_qsym(0, 0, 1, 0);
        9'b101000000: return make_qsym(0, 1, 1, 1);
        9'b111000000: return make_qsym(0, 1, 0, 0);
        9'b100000001: return make_qsym(-2, 0, 0, 1);
        9'b110000001: return make_qsym(-2, 0, 1, 0);
        9'b101000001: return make_qsym(-2, 1, 1, 1);
        9'b111000001: return make_qsym(-2, 1, 0, 0);
        9'b100000010: return make_qsym(0, -2, 0, 1);
        9'b110000010: return make_qsym(0, -2, 1, 0);
        9'b101000010: return make_qsym(0, -1, 1, 1);
        9'b111000010: return make_qsym(0, -1, 0, 0);
        9'b100000011: return make_qsym(-2, -2, 0, 1);
        9'b110000011: return make_qsym(-2, -2, 1, 0);
        9'b101000011: return make_qsym(-2, -1, 1, 1);
        9'b111000011: return make_qsym(-2, -1, 0, 0);
        9'b100000100: return make_qsym(0, 0, -2, 1);
        9'b110000100: return make_qsym(0, 0, -1, 0);
        9'b101000100: return make_qsym(0, 1, -1, 1);
        9'b111000100: return make_qsym(0, 1, -2, 0);
        9'b100000101: return make_qsym(-2, 0, -2, 1);
        9'b110000101: return make_qsym(-2, 0, -1, 0);
        9'b101000101: return make_qsym(-2, 1, -1, 1);
        9'b111000101: return make_qsym(-2, 1, -2, 0);
        9'b100000110: return make_qsym(0, -2, -2, 1);
        9'b110000110: return make_qsym(0, -2, -1, 0);
        9'b101000110: return make_qsym(0, -1, -1, 1);
        9'b111000110: return make_qsym(0, -1, -2, 0);
        9'b100000111: return make_qsym(-2, -2, -2, 1);
        9'b110000111: return make_qsym(-2, -2, -1, 0);
        9'b101000111: return make_qsym(-2, -1, -1, 1);
        9'b111000111: return make_qsym(-2, -1, -2, 0);
        9'b100001000: return make_qsym(0, 0, 0, -1);
        9'b110001000: return make_qsym(0, 0, 1, -2);
        9'b101001000: return make_qsym(0, 1, 1, -1);
        9'b111001000: return make_qsym(0, 1, 0, -2);
        9'b100001001: return make_qsym(-2, 0, 0, -1);
        9'b110001001: return make_qsym(-2, 0, 1, -2);
        9'b101001001: return make_qsym(-2, 1, 1, -1);
        9'b111001001: return make_qsym(-2, 1, 0, -2);
        9'b100001010: return make_qsym(0, -2, 0, -1);
        9'b110001010: return make_qsym(0, -2, 1, -2);
        9'b101001010: return make_qsym(0, -1, 1, -1);
        9'b111001010: return make_qsym(0, -1, 0, -2);
        9'b100001011: return make_qsym(-2, -2, 0, -1);
        9'b110001011: return make_qsym(-2, -2, 1, -2);
        9'b101001011: return make_qsym(-2, -1, 1, -1);
        9'b111001011: return make_qsym(-2, -1, 0, -2);
        9'b100001100: return make_qsym(0, 0, -2, -1);
        9'b110001100: return make_qsym(0, 0, -1, -2);
        9'b101001100: return make_qsym(0, 1, -1, -1);
        9'b111001100: return make_qsym(0, 1, -2, -2);
        9'b100001101: return make_qsym(-2, 0, -2, -1);
        9'b110001101: return make_qsym(-2, 0, -1, -2);
        9'b101001101: return make_qsym(-2, 1, -1, -1);
        9'b111001101: return make_qsym(-2, 1, -2, -2);
        9'b100001110: return make_qsym(0, -2, -2, -1);
        9'b110001110: return make_qsym(0, -2, -1, -2);
        9'b101001110: return make_qsym(0, -1, -1, -1);
        9'b111001110: return make_qsym(0, -1, -2, -2);
        9'b100001111: return make_qsym(-2, -2, -2, -1);
        9'b110001111: return make_qsym(-2, -2, -1, -2);
        9'b101001111: return make_qsym(-2, -1, -1, -1);
        9'b111001111: return make_qsym(-2, -1, -2, -2);
        9'b100010000: return make_qsym(1, 1, 1, 0);
        9'b110010000: return make_qsym(1, 1, 0, 1);
        9'b101010000: return make_qsym(1, 0, 0, 0);
        9'b111010000: return make_qsym(1, 0, 1, 1);
        9'b100010001: return make_qsym(-1, 1, 1, 0);
        9'b110010001: return make_qsym(-1, 1, 0, 1);
        9'b101010001: return make_qsym(-1, 0, 0, 0);
        9'b111010001: return make_qsym(-1, 0, 1, 1);
        9'b100010010: return make_qsym(1, -1, 1, 0);
        9'b110010010: return make_qsym(1, -1, 0, 1);
        9'b101010010: return make_qsym(1, -2, 0, 0);
        9'b111010010: return make_qsym(1, -2, 1, 1);
        9'b100010011: return make_qsym(-1, -1, 1, 0);
        9'b110010011: return make_qsym(-1, -1, 0, 1);
        9'b101010011: return make_qsym(-1, -2, 0, 0);
        9'b111010011: return make_qsym(-1, -2, 1, 1);
        9'b100010100: return make_qsym(1, 1, -1, 0);
        9'b110010100: return make_qsym(1, 1, -2, 1);
        9'b101010100: return make_qsym(1, 0, -2, 0);
        9'b111010100: return make_qsym(1, 0, -1, 1);
        9'b100010101: return make_qsym(-1, 1, -1, 0);
        9'b110010101: return make_qsym(-1, 1, -2, 1);
        9'b101010101: return make_qsym(-1, 0, -2, 0);
        9'b111010101: return make_qsym(-1, 0, -1, 1);
        9'b100010110: return make_qsym(1, -1, -1, 0);
        9'b110010110: return make_qsym(1, -1, -2, 1);
        9'b101010110: return make_qsym(1, -2, -2, 0);
        9'b111010110: return make_qsym(1, -2, -1, 1);
        9'b100010111: return make_qsym(-1, -1, -1, 0);
        9'b110010111: return make_qsym(-1, -1, -2, 1);
        9'b101010111: return make_qsym(-1, -2, -2, 0);
        9'b111010111: return make_qsym(-1, -2, -1, 1);
        9'b100011000: return make_qsym(1, 1, 1, -2);
        9'b110011000: return make_qsym(1, 1, 0, -1);
        9'b101011000: return make_qsym(1, 0, 0, -2);
        9'b111011000: return make_qsym(1, 0, 1, -1);
        9'b100011001: return make_qsym(-1, 1, 1, -2);
        9'b110011001: return make_qsym(-1, 1, 0, -1);
        9'b101011001: return make_qsym(-1, 0, 0, -2);
        9'b111011001: return make_qsym(-1, 0, 1, -1);
        9'b100011010: return make_qsym(1, -1, 1, -2);
        9'b110011010: return make_qsym(1, -1, 0, -1);
        9'b101011010: return make_qsym(1, -2, 0, -2);
        9'b111011010: return make_qsym(1, -2, 1, -1);
        9'b100011011: return make_qsym(-1, -1, 1, -2);
        9'b110011011: return make_qsym(-1, -1, 0, -1);
        9'b101011011: return make_qsym(-1, -2, 0, -2);
        9'b111011011: return make_qsym(-1, -2, 1, -1);
        9'b100011100: return make_qsym(1, 1, -1, -2);
        9'b110011100: return make_qsym(1, 1, -2, -1);
        9'b101011100: return make_qsym(1, 0, -2, -2);
        9'b111011100: return make_qsym(1, 0, -1, -1);
        9'b100011101: return make_qsym(-1, 1, -1, -2);
        9'b110011101: return make_qsym(-1, 1, -2, -1);
        9'b101011101: return make_qsym(-1, 0, -2, -2);
        9'b111011101: return make_qsym(-1, 0, -1, -1);
        9'b100011110: return make_qsym(1, -1, -1, -2);
        9'b110011110: return make_qsym(1, -1, -2, -1);
        9'b101011110: return make_qsym(1, -2, -2, -2);
        9'b111011110: return make_qsym(1, -2, -1, -1);
        9'b100011111: return make_qsym(-1, -1, -1, -2);
        9'b110011111: return make_qsym(-1, -1, -2, -1);
        9'b101011111: return make_qsym(-1, -2, -2, -2);
        9'b111011111: return make_qsym(-1, -2, -1, -1);
        9'b100100000: return make_qsym(2, 0, 0, 1);
        9'b110100000: return make_qsym(2, 0, 1, 0);
        9'b101100000: return make_qsym(2, 1, 1, 1);
        9'b111100000: return make_qsym(2, 1, 0, 0);
        9'b100100001: return make_qsym(2, -2, 0, 1);
        9'b110100001: return make_qsym(2, -2, 1, 0);
        9'b101100001: return make_qsym(2, -1, 1, 1);
        9'b111100001: return make_qsym(2, -1, 0, 0);
        9'b100100010: return make_qsym(2, 0, -2, 1);
        9'b110100010: return make_qsym(2, 0, -1, 0);
        9'b101100010: return make_qsym(2, 1, -1, 1);
        9'b111100010: return make_qsym(2, 1, -2, 0);
        9'b100100011: return make_qsym(2, -2, -2, 1);
        9'b110100011: return make_qsym(2, -2, -1, 0);
        9'b101100011: return make_qsym(2, -1, -1, 1);
        9'b111100011: return make_qsym(2, -1, -2, 0);
        9'b100100100: return make_qsym(2, 0, 0, -1);
        9'b110100100: return make_qsym(2, 0, 1, -2);
        9'b101100100: return make_qsym(2, 1, 1, -1);
        9'b111100100: return make_qsym(2, 1, 0, -2);
        9'b100100101: return make_qsym(2, -2, 0, -1);
        9'b110100101: return make_qsym(2, -2, 1, -2);
        9'b101100101: return make_qsym(2, -1, 1, -1);
        9'b111100101: return make_qsym(2, -1, 0, -2);
        9'b100100110: return make_qsym(2, 0, -2, -1);
        9'b110100110: return make_qsym(2, 0, -1, -2);
        9'b101100110: return make_qsym(2, 1, -1, -1);
        9'b111100110: return make_qsym(2, 1, -2, -2);
        9'b100100111: return make_qsym(2, -2, -2, -1);
        9'b110100111: return make_qsym(2, -2, -1, -2);
        9'b101100111: return make_qsym(2, -1, -1, -1);
        9'b111100111: return make_qsym(2, -1, -2, -2);
        9'b100101000: return make_qsym(0, 0, 2, 1);
        9'b110101000: return make_qsym(1, 1, 2, 1);
        9'b101101000: return make_qsym(1, 0, 2, 0);
        9'b111101000: return make_qsym(0, 1, 2, 0);
        9'b100101001: return make_qsym(-2, 0, 2, 1);
        9'b110101001: return make_qsym(-1, 1, 2, 1);
        9'b101101001: return make_qsym(-1, 0, 2, 0);
        9'b111101001: return make_qsym(-2, 1, 2, 0);
        9'b100101010: return make_qsym(0, -2, 2, 1);
        9'b110101010: return make_qsym(1, -1, 2, 1);
        9'b101101010: return make_qsym(1, -2, 2, 0);
        9'b111101010: return make_qsym(0, -1, 2, 0);
        9'b100101011: return make_qsym(-2, -2, 2, 1);
        9'b110101011: return make_qsym(-1, -1, 2, 1);
        9'b101101011: return make_qsym(-1, -2, 2, 0);
        9'b111101011: return make_qsym(-2, -1, 2, 0);
        9'b100101100: return make_qsym(0, 0, 2, -1);
        9'b110101100: return make_qsym(1, 1, 2, -1);
        9'b101101100: return make_qsym(1, 0, 2, -2);
        9'b111101100: return make_qsym(0, 1, 2, -2);
        9'b100101101: return make_qsym(-2, 0, 2, -1);
        9'b110101101: return make_qsym(-1, 1, 2, -1);
        9'b101101101: return make_qsym(-1, 0, 2, -2);
        9'b111101101: return make_qsym(-2, 1, 2, -2);
        9'b100101110: return make_qsym(0, -2, 2, -1);
        9'b110101110: return make_qsym(1, -1, 2, -1);
        9'b101101110: return make_qsym(1, -2, 2, -2);
        9'b111101110: return make_qsym(0, -1, 2, -2);
        9'b100101111: return make_qsym(-2, -2, 2, -1);
        9'b110101111: return make_qsym(-1, -1, 2, -1);
        9'b101101111: return make_qsym(-1, -2, 2, -2);
        9'b111101111: return make_qsym(-2, -1, 2, -2);
        9'b100110000: return make_qsym(0, 2, 0, 1);
        9'b110110000: return make_qsym(0, 2, 1, 0);
        9'b101110000: return make_qsym(1, 2, 0, 0);
        9'b111110000: return make_qsym(1, 2, 1, 1);
        9'b100110001: return make_qsym(-2, 2, 0, 1);
        9'b110110001: return make_qsym(-2, 2, 1, 0);
        9'b101110001: return make_qsym(-1, 2, 0, 0);
        9'b111110001: return make_qsym(-1, 2, 1, 1);
        9'b100110010: return make_qsym(0, 2, -2, 1);
        9'b110110010: return make_qsym(0, 2, -1, 0);
        9'b101110010: return make_qsym(1, 2, -2, 0);
        9'b111110010: return make_qsym(1, 2, -1, 1);
        9'b100110011: return make_qsym(-2, 2, -2, 1);
        9'b110110011: return make_qsym(-2, 2, -1, 0);
        9'b101110011: return make_qsym(-1, 2, -2, 0);
        9'b111110011: return make_qsym(-1, 2, -1, 1);
        9'b100110100: return make_qsym(0, 2, 0, -1);
        9'b110110100: return make_qsym(0, 2, 1, -2);
        9'b101110100: return make_qsym(1, 2, 0, -2);
        9'b111110100: return make_qsym(1, 2, 1, -1);
        9'b100110101: return make_qsym(-2, 2, 0, -1);
        9'b110110101: return make_qsym(-2, 2, 1, -2);
        9'b101110101: return make_qsym(-1, 2, 0, -2);
        9'b111110101: return make_qsym(-1, 2, 1, -1);
        9'b100110110: return make_qsym(0, 2, -2, -1);
        9'b110110110: return make_qsym(0, 2, -1, -2);
        9'b101110110: return make_qsym(1, 2, -2, -2);
        9'b111110110: return make_qsym(1, 2, -1, -1);
        9'b100110111: return make_qsym(-2, 2, -2, -1);
        9'b110110111: return make_qsym(-2, 2, -1, -2);
        9'b101110111: return make_qsym(-1, 2, -2, -2);
        9'b111110111: return make_qsym(-1, 2, -1, -1);
        9'b100111000: return make_qsym(1, 1, 1, 2);
        9'b110111000: return make_qsym(0, 0, 1, 2);
        9'b101111000: return make_qsym(1, 0, 0, 2);
        9'b111111000: return make_qsym(0, 1, 0, 2);
        9'b100111001: return make_qsym(-1, 1, 1, 2);
        9'b110111001: return make_qsym(-2, 0, 1, 2);
        9'b101111001: return make_qsym(-1, 0, 0, 2);
        9'b111111001: return make_qsym(-2, 1, 0, 2);
        9'b100111010: return make_qsym(1, -1, 1, 2);
        9'b110111010: return make_qsym(0, -2, 1, 2);
        9'b101111010: return make_qsym(1, -2, 0, 2);
        9'b111111010: return make_qsym(0, -1, 0, 2);
        9'b100111011: return make_qsym(-1, -1, 1, 2);
        9'b110111011: return make_qsym(-2, -2, 1, 2);
        9'b101111011: return make_qsym(-1, -2, 0, 2);
        9'b111111011: return make_qsym(-2, -1, 0, 2);
        9'b100111100: return make_qsym(1, 1, -1, 2);
        9'b110111100: return make_qsym(0, 0, -1, 2);
        9'b101111100: return make_qsym(1, 0, -2, 2);
        9'b111111100: return make_qsym(0, 1, -2, 2);
        9'b100111101: return make_qsym(-1, 1, -1, 2);
        9'b110111101: return make_qsym(-2, 0, -1, 2);
        9'b101111101: return make_qsym(-1, 0, -2, 2);
        9'b111111101: return make_qsym(-2, 1, -2, 2);
        9'b100111110: return make_qsym(1, -1, -1, 2);
        9'b110111110: return make_qsym(0, -2, -1, 2);
        9'b101111110: return make_qsym(1, -2, -2, 2);
        9'b111111110: return make_qsym(0, -1, -2, 2);
        9'b100111111: return make_qsym(-1, -1, -1, 2);
        9'b110111111: return make_qsym(-2, -2, -1, 2);
        9'b101111111: return make_qsym(-1, -2, -2, 2);
        9'b111111111: return make_qsym(-2, -1, -2, 2);
    endcase

  endfunction


  // ------------------------------------------------------------
  // Step 6: apply sign randomization.
  // Srev modeled as OR of tx_enable_(n-2) and tx_enable_(n-4).
  // Sign rule:
  //   sign = +1 if (Sgn[i] ^ Srev) == 0
  //   sign = -1 otherwise
  // ------------------------------------------------------------
  function qsym_t apply_signs(
    input qsym_t      t,
    input logic [3:0] sg
  );
    qsym_t q;
    logic srev;

    srev = tx_enable_d2 | tx_enable_d4;

    q.a = apply_one_sign(t.a, sg[0] ^ srev);
    q.b = apply_one_sign(t.b, sg[1] ^ srev);
    q.c = apply_one_sign(t.c, sg[2] ^ srev);
    q.d = apply_one_sign(t.d, sg[3] ^ srev);

    return q;
  endfunction

  function logic signed [2:0] apply_one_sign(
    input logic signed [2:0] mag,
    input logic              negate
  );
    if (mag == 0)
      return sym(0);

    if (negate)
      return -mag;

    return mag;
  endfunction

  // ------------------------------------------------------------
  // Output packing.
  //
  // Project-local representation:
  //   -2 -> 3'b000
  //   -1 -> 3'b001
  //    0 -> 3'b010
  //   +1 -> 3'b011
  //   +2 -> 3'b100
  //
  // Change this if your DUT uses a different 3-bit symbol packing.
  // ------------------------------------------------------------
  function logic [11:0] pack_symbols(input qsym_t q);
    logic [11:0] out;

    out[11:9] = pack_one(q.a);
    out[8:6]  = pack_one(q.b);
    out[5:3]  = pack_one(q.c);
    out[2:0]  = pack_one(q.d);

    return out;
  endfunction

//   000= 0
//   001=+1
//   010=+2
//   111=-1
//   110=-2

  function logic [2:0] pack_one(input logic signed [2:0] v);
    case (v)
      -3'sd2: return 3'b110;
      -3'sd1: return 3'b111;
       3'sd0: return 3'b000;
       3'sd1: return 3'b001;
       3'sd2: return 3'b010;
      default: return 3'b111;
    endcase
  endfunction

  // ------------------------------------------------------------
  // State update.
  // Happens after current output has been predicted.
  // ------------------------------------------------------------
  function void update_state(
    input logic       tx_enable,
    input logic       tx_error,
    input logic [7:0] txd,
    input logic [3:0] sy,
    input logic [2:0] cs_next
  );
    update_scrambler();

    cs = cs_next;

    tx_enable_d4 = tx_enable_d3;
    tx_enable_d3 = tx_enable_d2;
    tx_enable_d2 = tx_enable_d1;
    tx_enable_d1 = tx_enable;

    tx_error_d3 = tx_error_d2;
    tx_error_d2 = tx_error_d1;
    tx_error_d1 = tx_error;

    txd_d4 = txd_d3;
    txd_d3 = txd_d2;
    txd_d2 = txd_d1;
    txd_d1 = txd;

    sy_d1 = sy;
    sym_count++;
  endfunction

  // ------------------------------------------------------------
  // Scrambler update.
  // MASTER: gM(x) = 1 + x^13 + x^33
  //
  // The polynomial constant does not mean XOR with constant 1.
  // It defines the recurrence taps.
  // ------------------------------------------------------------
  function void update_scrambler();
    logic feedback;

    feedback = scr[12] ^ scr[32];

    scr = {scr[31:0], feedback};

    if (scr == 33'h0)
      scr = 33'h1;
  endfunction

  // ------------------------------------------------------------
  // Helpers
  // ------------------------------------------------------------
  function qsym_t make_qsym(input int ia, input int ib, input int ic, input int id);
    qsym_t q;

    q.a = sym(ia);
    q.b = sym(ib);
    q.c = sym(ic);
    q.d = sym(id);

    return q;
  endfunction

  function logic signed [2:0] sym(input int v);
    case (v)
      -2: return -3'sd2;
      -1: return -3'sd1;
       0: return  3'sd0;
       1: return  3'sd1;
       2: return  3'sd2;
      default: return 3'sd0;
    endcase
  endfunction


endclass : pcs_ref_model

module encoder(
  input  logic        clk,
  input  logic        rst,
  input  logic	      TX_EN,
  input  logic [7:0]  enc_in,
  output logic [11:0] enc_out
);
pcs_ref_model refm;


  int unsigned cycle_count;
  logic [11:0] expected_out;
  logic [11:0] dut_out_next;

  initial begin
    refm = new();
    refm.reset(33'b1);
    cycle_count = 0;
  end

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      refm.reset(33'b1);
      enc_out     <= 12'h000;
      cycle_count <= 0;
    end
    else begin
      expected_out = refm.encode({~TX_EN,enc_in}, 1'b0);
      dut_out_next = expected_out;

      enc_out <= dut_out_next;
      cycle_count <= cycle_count + 1;
    end
  end

endmodule : encoder
