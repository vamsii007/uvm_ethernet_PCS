/*
 * pcs_tx_rm.c - 1000BASE-T PCS Transmit reference model
 *
 * This model follows IEEE 802.3-2012 Clause 40 PCS Transmit encoding:
 *   - side-stream scrambler
 *   - Sx/Sy/Sg generation
 *   - Sc/Sd generation
 *   - convolutional encoder state
 *   - Table 40-1 / Table 40-2 bit-to-symbol mapping
 *   - Srev polarity randomization
 *
 * It intentionally does NOT verify reset behavior.  Call pcs_tx_init_state()
 * at the verification start point to align the reference model to the DUT's
 * post-reset/start state.
 *
 * For the EE273 project 9-bit input wrapper:
 *   input[8]   = 0 -> data byte, decoded as tx_enable=1, tx_error=0, TXD=input[7:0]
 *   input[8]   = 1 -> command byte, decoded through a configurable command table
 *
 * The IEEE spec itself does not define this 9-bit project command encoding.
 * Keep the PCS core model spec-level; only modify the command table/adapter
 * when the grading DUT defines a specific command code convention.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PCS_TX_RM_API
#define PCS_TX_RM_API

typedef enum {
    PCS_ROLE_MASTER = 0,
    PCS_ROLE_SLAVE  = 1
} pcs_role_t;

typedef enum {
    PCS_TX_MODE_SEND_N = 0,
    PCS_TX_MODE_SEND_I = 1,
    PCS_TX_MODE_SEND_Z = 2
} pcs_tx_mode_t;

typedef enum {
    PCS_RCVR_NOT_OK = 0,
    PCS_RCVR_OK     = 1
} pcs_rcvr_status_t;

typedef enum {
    PCS_COND_NORMAL = 0,
    PCS_COND_IDLE_CARR_EXT,
    PCS_COND_XMT_ERR,
    PCS_COND_CSEXTEND_ERR,
    PCS_COND_CSEXTEND,
    PCS_COND_CSRESET,
    PCS_COND_SSD1,
    PCS_COND_SSD2,
    PCS_COND_ESD1,
    PCS_COND_ESD2_EXT_0,
    PCS_COND_ESD2_EXT_1,
    PCS_COND_ESD2_EXT_2,
    PCS_COND_ESD_EXT_ERR
} pcs_symbol_condition_t;

typedef enum {
    PCS_DUT0_ST_RESET = 0,
    PCS_DUT0_ST_SEND_IDLE,
    PCS_DUT0_ST_SDD1,
    PCS_DUT0_ST_SDD2,
    PCS_DUT0_ST_TRANSMIT_DATA,
    PCS_DUT0_ST_CSR1,
    PCS_DUT0_ST_CSR2,
    PCS_DUT0_ST_ESD1,
    PCS_DUT0_ST_ESD2
} pcs_dut0_state_t;

typedef struct {
    bool tx_enable;
    bool tx_error;
    uint8_t txd;
    pcs_tx_mode_t tx_mode;
    bool loc_lpi_req;
    pcs_rcvr_status_t loc_rcvr_status;
    bool loc_update_done;
} pcs_tx_in_t;

typedef struct {
    int8_t ta, tb, tc, td;     /* table symbols before polarity randomization */
    int8_t a, b, c, d;         /* final PAM5 symbols after polarity randomization */
    uint16_t packed12_tc3;     /* default packing: A/B/C/D as 3-bit two's complement */

    uint8_t sx, sy, sg;
    uint8_t sc;
    uint16_t sd;               /* 9-bit Sd[8:0] */
    uint8_t cs;                /* cs_n */
    bool csreset;
    bool srev;
    pcs_symbol_condition_t condition;
} pcs_tx_out_t;

typedef struct {
    /* Initial values that usually need to match the DUT. */
    pcs_role_t role;
    uint64_t scr_init_33;      /* lower 33 bits valid, must not be zero */
    uint8_t cs_init;           /* 3 bits: cs_{n-1} at verification start */
    uint16_t sd_prev_init;     /* 9 bits: Sd_{n-1} at verification start */
    uint8_t sy_prev_init;      /* 4 bits: Sy_{n-1} at verification start */

    bool tx_enable_d1_init;
    bool tx_enable_d2_init;
    bool tx_enable_d3_init;
    bool tx_enable_d4_init;

    bool tx_error_d1_init;
    bool tx_error_d2_init;
    bool tx_error_d3_init;

    uint64_t n_init;           /* current time index n at first pcs_tx_step() */
    uint64_t n0_init;          /* time index of last TX side-stream scrambler reset */

    /* Spec notation uses Scr_n for current-cycle generation. In this implementation
     * state.scr holds Scr_{n-1} at function entry when this is true, so we advance
     * once and use the advanced value. If your DUT uses the stored scrambler value
     * before advancing, set this false or equivalently adjust scr_init_33 by one step.
     */
    bool advance_scrambler_before_use;
} pcs_tx_init_cfg_t;

typedef struct {
    pcs_role_t role;
    uint64_t scr;              /* lower 33 bits valid */
    uint8_t cs;                /* 3 bits, previous cs at step entry */
    uint16_t sd_prev;          /* previous Sd[8:0] */
    uint8_t sy_prev;           /* previous Sy[3:0] */

    bool tx_enable_d1;
    bool tx_enable_d2;
    bool tx_enable_d3;
    bool tx_enable_d4;

    bool tx_error_d1;
    bool tx_error_d2;
    bool tx_error_d3;

    uint64_t n;
    uint64_t n0;
    bool advance_scrambler_before_use;

    /* DUTS26_0 compatibility state.  These fields model the professor DUT's
     * wrapper/FSM behavior, not only the IEEE core equations.
     */
    uint8_t dut0_state;        /* pcs_dut0_state_t stored as byte for C ABI stability */
    uint8_t tx_enable_pipe;    /* DUT tx_enable[4:0], bit0 is newest registered sample */
    bool oe;                   /* DUT OE toggle used by Sc[3:1] generation */
} pcs_tx_state_t;

typedef struct {
    bool valid[256];
    pcs_tx_in_t entry[256];
} pcs_cmd_decode_table_t;

#endif /* PCS_TX_RM_API */

#define PCS_SCR_MASK_33 ((1ULL << 33) - 1ULL)

static inline uint8_t bit_u64(uint64_t x, unsigned b) { return (uint8_t)((x >> b) & 1ULL); }
static inline uint8_t bit_u16(uint16_t x, unsigned b) { return (uint8_t)((x >> b) & 1U); }
static inline uint8_t bit_u8 (uint8_t  x, unsigned b) { return (uint8_t)((x >> b) & 1U); }

static const int8_t pcs_tbl40_1_normal[64][4][4] = {
  {
    {0,0,0,0},
    {0,0,+1,+1},
    {0,+1,+1,0},
    {0,+1,0,+1}
  },
  {
    {-2,0,0,0},
    {-2,0,+1,+1},
    {-2,+1,+1,0},
    {-2,+1,0,+1}
  },
  {
    {0,-2,0,0},
    {0,-2,+1,+1},
    {0,-1,+1,0},
    {0,-1,0,+1}
  },
  {
    {-2,-2,0,0},
    {-2,-2,+1,+1},
    {-2,-1,+1,0},
    {-2,-1,0,+1}
  },
  {
    {0,0,-2,0},
    {0,0,-1,+1},
    {0,+1,-1,0},
    {0,+1,-2,+1}
  },
  {
    {-2,0,-2,0},
    {-2,0,-1,+1},
    {-2,+1,-1,0},
    {-2,+1,-2,+1}
  },
  {
    {0,-2,-2,0},
    {0,-2,-1,+1},
    {0,-1,-1,0},
    {0,-1,-2,+1}
  },
  {
    {-2,-2,-2,0},
    {-2,-2,-1,+1},
    {-2,-1,-1,0},
    {-2,-1,-2,+1}
  },
  {
    {0,0,0,-2},
    {0,0,+1,-1},
    {0,+1,+1,-2},
    {0,+1,0,-1}
  },
  {
    {-2,0,0,-2},
    {-2,0,+1,-1},
    {-2,+1,+1,-2},
    {-2,+1,0,-1}
  },
  {
    {0,-2,0,-2},
    {0,-2,+1,-1},
    {0,-1,+1,-2},
    {0,-1,0,-1}
  },
  {
    {-2,-2,0,-2},
    {-2,-2,+1,-1},
    {-2,-1,+1,-2},
    {-2,-1,0,-1}
  },
  {
    {0,0,-2,-2},
    {0,0,-1,-1},
    {0,+1,-1,-2},
    {0,+1,-2,-1}
  },
  {
    {-2,0,-2,-2},
    {-2,0,-1,-1},
    {-2,+1,-1,-2},
    {-2,+1,-2,-1}
  },
  {
    {0,-2,-2,-2},
    {0,-2,-1,-1},
    {0,-1,-1,-2},
    {0,-1,-2,-1}
  },
  {
    {-2,-2,-2,-2},
    {-2,-2,-1,-1},
    {-2,-1,-1,-2},
    {-2,-1,-2,-1}
  },
  {
    {+1,+1,+1,+1},
    {+1,+1,0,0},
    {+1,0,0,+1},
    {+1,0,+1,0}
  },
  {
    {-1,+1,+1,+1},
    {-1,+1,0,0},
    {-1,0,0,+1},
    {-1,0,+1,0}
  },
  {
    {+1,-1,+1,+1},
    {+1,-1,0,0},
    {+1,-2,0,+1},
    {+1,-2,+1,0}
  },
  {
    {-1,-1,+1,+1},
    {-1,-1,0,0},
    {-1,-2,0,+1},
    {-1,-2,+1,0}
  },
  {
    {+1,+1,-1,+1},
    {+1,+1,-2,0},
    {+1,0,-2,+1},
    {+1,0,-1,0}
  },
  {
    {-1,+1,-1,+1},
    {-1,+1,-2,0},
    {-1,0,-2,+1},
    {-1,0,-1,0}
  },
  {
    {+1,-1,-1,+1},
    {+1,-1,-2,0},
    {+1,-2,-2,+1},
    {+1,-2,-1,0}
  },
  {
    {-1,-1,-1,+1},
    {-1,-1,-2,0},
    {-1,-2,-2,+1},
    {-1,-2,-1,0}
  },
  {
    {+1,+1,+1,-1},
    {+1,+1,0,-2},
    {+1,0,0,-1},
    {+1,0,+1,-2}
  },
  {
    {-1,+1,+1,-1},
    {-1,+1,0,-2},
    {-1,0,0,-1},
    {-1,0,+1,-2}
  },
  {
    {+1,-1,+1,-1},
    {+1,-1,0,-2},
    {+1,-2,0,-1},
    {+1,-2,+1,-2}
  },
  {
    {-1,-1,+1,-1},
    {-1,-1,0,-2},
    {-1,-2,0,-1},
    {-1,-2,+1,-2}
  },
  {
    {+1,+1,-1,-1},
    {+1,+1,-2,-2},
    {+1,0,-2,-1},
    {+1,0,-1,-2}
  },
  {
    {-1,+1,-1,-1},
    {-1,+1,-2,-2},
    {-1,0,-2,-1},
    {-1,0,-1,-2}
  },
  {
    {+1,-1,-1,-1},
    {+1,-1,-2,-2},
    {+1,-2,-2,-1},
    {+1,-2,-1,-2}
  },
  {
    {-1,-1,-1,-1},
    {-1,-1,-2,-2},
    {-1,-2,-2,-1},
    {-1,-2,-1,-2}
  },
  {
    {+2,0,0,0},
    {+2,0,+1,+1},
    {+2,+1,+1,0},
    {+2,+1,0,+1}
  },
  {
    {+2,-2,0,0},
    {+2,-2,+1,+1},
    {+2,-1,+1,0},
    {+2,-1,0,+1}
  },
  {
    {+2,0,-2,0},
    {+2,0,-1,+1},
    {+2,+1,-1,0},
    {+2,+1,-2,+1}
  },
  {
    {+2,-2,-2,0},
    {+2,-2,-1,+1},
    {+2,-1,-1,0},
    {+2,-1,-2,+1}
  },
  {
    {+2,0,0,-2},
    {+2,0,+1,-1},
    {+2,+1,+1,-2},
    {+2,+1,0,-1}
  },
  {
    {+2,-2,0,-2},
    {+2,-2,+1,-1},
    {+2,-1,+1,-2},
    {+2,-1,0,-1}
  },
  {
    {+2,0,-2,-2},
    {+2,0,-1,-1},
    {+2,+1,-1,-2},
    {+2,+1,-2,-1}
  },
  {
    {+2,-2,-2,-2},
    {+2,-2,-1,-1},
    {+2,-1,-1,-2},
    {+2,-1,-2,-1}
  },
  {
    {0,0,+2,0},
    {+1,+1,+2,0},
    {+1,0,+2,+1},
    {0,+1,+2,+1}
  },
  {
    {-2,0,+2,0},
    {-1,+1,+2,0},
    {-1,0,+2,+1},
    {-2,+1,+2,+1}
  },
  {
    {0,-2,+2,0},
    {+1,-1,+2,0},
    {+1,-2,+2,+1},
    {0,-1,+2,+1}
  },
  {
    {-2,-2,+2,0},
    {-1,-1,+2,0},
    {-1,-2,+2,+1},
    {-2,-1,+2,+1}
  },
  {
    {0,0,+2,-2},
    {+1,+1,+2,-2},
    {+1,0,+2,-1},
    {0,+1,+2,-1}
  },
  {
    {-2,0,+2,-2},
    {-1,+1,+2,-2},
    {-1,0,+2,-1},
    {-2,+1,+2,-1}
  },
  {
    {0,-2,+2,-2},
    {+1,-1,+2,-2},
    {+1,-2,+2,-1},
    {0,-1,+2,-1}
  },
  {
    {-2,-2,+2,-2},
    {-1,-1,+2,-2},
    {-1,-2,+2,-1},
    {-2,-1,+2,-1}
  },
  {
    {0,+2,0,0},
    {0,+2,+1,+1},
    {+1,+2,0,+1},
    {+1,+2,+1,0}
  },
  {
    {-2,+2,0,0},
    {-2,+2,+1,+1},
    {-1,+2,0,+1},
    {-1,+2,+1,0}
  },
  {
    {0,+2,-2,0},
    {0,+2,-1,+1},
    {+1,+2,-2,+1},
    {+1,+2,-1,0}
  },
  {
    {-2,+2,-2,0},
    {-2,+2,-1,+1},
    {-1,+2,-2,+1},
    {-1,+2,-1,0}
  },
  {
    {0,+2,0,-2},
    {0,+2,+1,-1},
    {+1,+2,0,-1},
    {+1,+2,+1,-2}
  },
  {
    {-2,+2,0,-2},
    {-2,+2,+1,-1},
    {-1,+2,0,-1},
    {-1,+2,+1,-2}
  },
  {
    {0,+2,-2,-2},
    {0,+2,-1,-1},
    {+1,+2,-2,-1},
    {+1,+2,-1,-2}
  },
  {
    {-2,+2,-2,-2},
    {-2,+2,-1,-1},
    {-1,+2,-2,-1},
    {-1,+2,-1,-2}
  },
  {
    {0,0,0,+2},
    {+1,+1,0,+2},
    {0,+1,+1,+2},
    {+1,0,+1,+2}
  },
  {
    {-2,0,0,+2},
    {-1,+1,0,+2},
    {-2,+1,+1,+2},
    {-1,0,+1,+2}
  },
  {
    {0,-2,0,+2},
    {+1,-1,0,+2},
    {0,-1,+1,+2},
    {+1,-2,+1,+2}
  },
  {
    {-2,-2,0,+2},
    {-1,-1,0,+2},
    {-2,-1,+1,+2},
    {-1,-2,+1,+2}
  },
  {
    {0,0,-2,+2},
    {+1,+1,-2,+2},
    {0,+1,-1,+2},
    {+1,0,-1,+2}
  },
  {
    {-2,0,-2,+2},
    {-1,+1,-2,+2},
    {-2,+1,-1,+2},
    {-1,0,-1,+2}
  },
  {
    {0,-2,-2,+2},
    {+1,-1,-2,+2},
    {0,-1,-1,+2},
    {+1,-2,-1,+2}
  },
  {
    {-2,-2,-2,+2},
    {-1,-1,-2,+2},
    {-2,-1,-1,+2},
    {-1,-2,-1,+2}
  }
};

static const int8_t pcs_tbl40_2_normal[64][4][4] = {
  {
    {0,0,0,+1},
    {0,0,+1,0},
    {0,+1,+1,+1},
    {0,+1,0,0}
  },
  {
    {-2,0,0,+1},
    {-2,0,+1,0},
    {-2,+1,+1,+1},
    {-2,+1,0,0}
  },
  {
    {0,-2,0,+1},
    {0,-2,+1,0},
    {0,-1,+1,+1},
    {0,-1,0,0}
  },
  {
    {-2,-2,0,+1},
    {-2,-2,+1,0},
    {-2,-1,+1,+1},
    {-2,-1,0,0}
  },
  {
    {0,0,-2,+1},
    {0,0,-1,0},
    {0,+1,-1,+1},
    {0,+1,-2,0}
  },
  {
    {-2,0,-2,+1},
    {-2,0,-1,0},
    {-2,+1,-1,+1},
    {-2,+1,-2,0}
  },
  {
    {0,-2,-2,+1},
    {0,-2,-1,0},
    {0,-1,-1,+1},
    {0,-1,-2,0}
  },
  {
    {-2,-2,-2,+1},
    {-2,-2,-1,0},
    {-2,-1,-1,+1},
    {-2,-1,-2,0}
  },
  {
    {0,0,0,-1},
    {0,0,+1,-2},
    {0,+1,+1,-1},
    {0,+1,0,-2}
  },
  {
    {-2,0,0,-1},
    {-2,0,+1,-2},
    {-2,+1,+1,-1},
    {-2,+1,0,-2}
  },
  {
    {0,-2,0,-1},
    {0,-2,+1,-2},
    {0,-1,+1,-1},
    {0,-1,0,-2}
  },
  {
    {-2,-2,0,-1},
    {-2,-2,+1,-2},
    {-2,-1,+1,-1},
    {-2,-1,0,-2}
  },
  {
    {0,0,-2,-1},
    {0,0,-1,-2},
    {0,+1,-1,-1},
    {0,+1,-2,-2}
  },
  {
    {-2,0,-2,-1},
    {-2,0,-1,-2},
    {-2,+1,-1,-1},
    {-2,+1,-2,-2}
  },
  {
    {0,-2,-2,-1},
    {0,-2,-1,-2},
    {0,-1,-1,-1},
    {0,-1,-2,-2}
  },
  {
    {-2,-2,-2,-1},
    {-2,-2,-1,-2},
    {-2,-1,-1,-1},
    {-2,-1,-2,-2}
  },
  {
    {+1,+1,+1,0},
    {+1,+1,0,+1},
    {+1,0,0,0},
    {+1,0,+1,+1}
  },
  {
    {-1,+1,+1,0},
    {-1,+1,0,+1},
    {-1,0,0,0},
    {-1,0,+1,+1}
  },
  {
    {+1,-1,+1,0},
    {+1,-1,0,+1},
    {+1,-2,0,0},
    {+1,-2,+1,+1}
  },
  {
    {-1,-1,+1,0},
    {-1,-1,0,+1},
    {-1,-2,0,0},
    {-1,-2,+1,+1}
  },
  {
    {+1,+1,-1,0},
    {+1,+1,-2,+1},
    {+1,0,-2,0},
    {+1,0,-1,+1}
  },
  {
    {-1,+1,-1,0},
    {-1,+1,-2,+1},
    {-1,0,-2,0},
    {-1,0,-1,+1}
  },
  {
    {+1,-1,-1,0},
    {+1,-1,-2,+1},
    {+1,-2,-2,0},
    {+1,-2,-1,+1}
  },
  {
    {-1,-1,-1,0},
    {-1,-1,-2,+1},
    {-1,-2,-2,0},
    {-1,-2,-1,+1}
  },
  {
    {+1,+1,+1,-2},
    {+1,+1,0,-1},
    {+1,0,0,-2},
    {+1,0,+1,-1}
  },
  {
    {-1,+1,+1,-2},
    {-1,+1,0,-1},
    {-1,0,0,-2},
    {-1,0,+1,-1}
  },
  {
    {+1,-1,+1,-2},
    {+1,-1,0,-1},
    {+1,-2,0,-2},
    {+1,-2,+1,-1}
  },
  {
    {-1,-1,+1,-2},
    {-1,-1,0,-1},
    {-1,-2,0,-2},
    {-1,-2,+1,-1}
  },
  {
    {+1,+1,-1,-2},
    {+1,+1,-2,-1},
    {+1,0,-2,-2},
    {+1,0,-1,-1}
  },
  {
    {-1,+1,-1,-2},
    {-1,+1,-2,-1},
    {-1,0,-2,-2},
    {-1,0,-1,-1}
  },
  {
    {+1,-1,-1,-2},
    {+1,-1,-2,-1},
    {+1,-2,-2,-2},
    {+1,-2,-1,-1}
  },
  {
    {-1,-1,-1,-2},
    {-1,-1,-2,-1},
    {-1,-2,-2,-2},
    {-1,-2,-1,-1}
  },
  {
    {+2,0,0,+1},
    {+2,0,+1,0},
    {+2,+1,+1,+1},
    {+2,+1,0,0}
  },
  {
    {+2,-2,0,+1},
    {+2,-2,+1,0},
    {+2,-1,+1,+1},
    {+2,-1,0,0}
  },
  {
    {+2,0,-2,+1},
    {+2,0,-1,0},
    {+2,+1,-1,+1},
    {+2,+1,-2,0}
  },
  {
    {+2,-2,-2,+1},
    {+2,-2,-1,0},
    {+2,-1,-1,+1},
    {+2,-1,-2,0}
  },
  {
    {+2,0,0,-1},
    {+2,0,+1,-2},
    {+2,+1,+1,-1},
    {+2,+1,0,-2}
  },
  {
    {+2,-2,0,-1},
    {+2,-2,+1,-2},
    {+2,-1,+1,-1},
    {+2,-1,0,-2}
  },
  {
    {+2,0,-2,-1},
    {+2,0,-1,-2},
    {+2,+1,-1,-1},
    {+2,+1,-2,-2}
  },
  {
    {+2,-2,-2,-1},
    {+2,-2,-1,-2},
    {+2,-1,-1,-1},
    {+2,-1,-2,-2}
  },
  {
    {0,0,+2,+1},
    {+1,+1,+2,+1},
    {+1,0,+2,0},
    {0,+1,+2,0}
  },
  {
    {-2,0,+2,+1},
    {-1,+1,+2,+1},
    {-1,0,+2,0},
    {-2,+1,+2,0}
  },
  {
    {0,-2,+2,+1},
    {+1,-1,+2,+1},
    {+1,-2,+2,0},
    {0,-1,+2,0}
  },
  {
    {-2,-2,+2,+1},
    {-1,-1,+2,+1},
    {-1,-2,+2,0},
    {-2,-1,+2,0}
  },
  {
    {0,0,+2,-1},
    {+1,+1,+2,-1},
    {+1,0,+2,-2},
    {0,+1,+2,-2}
  },
  {
    {-2,0,+2,-1},
    {-1,+1,+2,-1},
    {-1,0,+2,-2},
    {-2,+1,+2,-2}
  },
  {
    {0,-2,+2,-1},
    {+1,-1,+2,-1},
    {+1,-2,+2,-2},
    {0,-1,+2,-2}
  },
  {
    {-2,-2,+2,-1},
    {-1,-1,+2,-1},
    {-1,-2,+2,-2},
    {-2,-1,+2,-2}
  },
  {
    {0,+2,0,+1},
    {0,+2,+1,0},
    {+1,+2,0,0},
    {+1,+2,+1,+1}
  },
  {
    {-2,+2,0,+1},
    {-2,+2,+1,0},
    {-1,+2,0,0},
    {-1,+2,+1,+1}
  },
  {
    {0,+2,-2,+1},
    {0,+2,-1,0},
    {+1,+2,-2,0},
    {+1,+2,-1,+1}
  },
  {
    {-2,+2,-2,+1},
    {-2,+2,-1,0},
    {-1,+2,-2,0},
    {-1,+2,-1,+1}
  },
  {
    {0,+2,0,-1},
    {0,+2,+1,-2},
    {+1,+2,0,-2},
    {+1,+2,+1,-1}
  },
  {
    {-2,+2,0,-1},
    {-2,+2,+1,-2},
    {-1,+2,0,-2},
    {-1,+2,+1,-1}
  },
  {
    {0,+2,-2,-1},
    {0,+2,-1,-2},
    {+1,+2,-2,-2},
    {+1,+2,-1,-1}
  },
  {
    {-2,+2,-2,-1},
    {-2,+2,-1,-2},
    {-1,+2,-2,-2},
    {-1,+2,-1,-1}
  },
  {
    {+1,+1,+1,+2},
    {0,0,+1,+2},
    {+1,0,0,+2},
    {0,+1,0,+2}
  },
  {
    {-1,+1,+1,+2},
    {-2,0,+1,+2},
    {-1,0,0,+2},
    {-2,+1,0,+2}
  },
  {
    {+1,-1,+1,+2},
    {0,-2,+1,+2},
    {+1,-2,0,+2},
    {0,-1,0,+2}
  },
  {
    {-1,-1,+1,+2},
    {-2,-2,+1,+2},
    {-1,-2,0,+2},
    {-2,-1,0,+2}
  },
  {
    {+1,+1,-1,+2},
    {0,0,-1,+2},
    {+1,0,-2,+2},
    {0,+1,-2,+2}
  },
  {
    {-1,+1,-1,+2},
    {-2,0,-1,+2},
    {-1,0,-2,+2},
    {-2,+1,-2,+2}
  },
  {
    {+1,-1,-1,+2},
    {0,-2,-1,+2},
    {+1,-2,-2,+2},
    {0,-1,-2,+2}
  },
  {
    {-1,-1,-1,+2},
    {-2,-2,-1,+2},
    {-1,-2,-2,+2},
    {-2,-1,-2,+2}
  }
};

static const int8_t pcs_tbl40_1_idle_carr_ext[16][4] = {
  {0,0,0,0},
  {-2,0,0,0},
  {0,-2,0,0},
  {-2,-2,0,0},
  {0,0,-2,0},
  {-2,0,-2,0},
  {0,-2,-2,0},
  {-2,-2,-2,0},
  {0,0,0,-2},
  {-2,0,0,-2},
  {0,-2,0,-2},
  {-2,-2,0,-2},
  {0,0,-2,-2},
  {-2,0,-2,-2},
  {0,-2,-2,-2},
  {-2,-2,-2,-2}
};

/* special4 order: xmt_err, CSExtend_Err, CSExtend, CSReset */
static const int8_t pcs_tbl40_1_special4[4][4][4] = {
  {
    {0,+2,+2,0},
    {+1,+1,+2,+2},
    {+2,+1,+1,+2},
    {+2,+1,+2,+1}
  },
  {
    {-2,+2,+2,-2},
    {-1,-1,+2,+2},
    {+2,-1,-1,+2},
    {+2,-1,+2,-1}
  },
  {
    {+2,0,0,+2},
    {+2,+2,+1,+1},
    {+1,+2,+2,+1},
    {+1,+2,+1,+2}
  },
  {
    {+2,-2,-2,+2},
    {+2,+2,-1,-1},
    {-1,+2,+2,-1},
    {-1,+2,-1,+2}
  }
};

static const int8_t pcs_tbl40_2_special4[4][4][4] = {
  {
    {+2,+2,0,+1},
    {0,+2,+1,+2},
    {+1,+2,+2,0},
    {+2,+1,+2,0}
  },
  {
    {+2,+2,-2,-1},
    {-2,+2,-1,+2},
    {-1,+2,+2,-2},
    {+2,-1,+2,-2}
  },
  {
    {+2,0,+2,+1},
    {+2,0,+1,+2},
    {+1,0,+2,+2},
    {+2,+1,0,+2}
  },
  {
    {+2,-2,+2,-1},
    {+2,-2,-1,+2},
    {-1,-2,+2,+2},
    {+2,-1,-2,+2}
  }
};

/* special1 order: SSD1, SSD2, ESD1, ESD2_Ext_0, ESD2_Ext_1, ESD2_Ext_2, ESD_Ext_Err */
static const int8_t pcs_tbl40_1_special1[7][4] = {
  {+2,+2,+2,+2},
  {+2,+2,+2,-2},
  {+2,+2,+2,+2},
  {+2,+2,+2,-2},
  {+2,+2,-2,+2},
  {+2,-2,+2,+2},
  {-2,+2,+2,+2}
};


static uint64_t pcs_scrambler_advance(uint64_t scr, pcs_role_t role)
{
    scr &= PCS_SCR_MASK_33;
    uint8_t fb;
    if (role == PCS_ROLE_MASTER) {
        fb = bit_u64(scr, 12) ^ bit_u64(scr, 32);  /* g_M(x)=1+x^13+x^33 */
    } else {
        fb = bit_u64(scr, 19) ^ bit_u64(scr, 32);  /* g_S(x)=1+x^20+x^33 */
    }
    return (((scr << 1) & PCS_SCR_MASK_33) | fb);
}

static uint8_t pcs_gen_sy(uint64_t scr)
{
    uint8_t sy = 0;
    sy |= (bit_u64(scr, 0)) << 0;
    sy |= (bit_u64(scr, 3) ^ bit_u64(scr, 8)) << 1;
    sy |= (bit_u64(scr, 6) ^ bit_u64(scr, 16)) << 2;
    sy |= (bit_u64(scr, 9) ^ bit_u64(scr, 14) ^ bit_u64(scr, 19) ^ bit_u64(scr, 24)) << 3;
    return sy & 0xF;
}

static uint8_t pcs_gen_sx(uint64_t scr)
{
    uint8_t sx = 0;
    sx |= (bit_u64(scr, 4) ^ bit_u64(scr, 6)) << 0;
    sx |= (bit_u64(scr, 7) ^ bit_u64(scr, 9) ^ bit_u64(scr, 12) ^ bit_u64(scr, 14)) << 1;
    sx |= (bit_u64(scr, 10) ^ bit_u64(scr, 12) ^ bit_u64(scr, 20) ^ bit_u64(scr, 22)) << 2;
    sx |= (bit_u64(scr, 13) ^ bit_u64(scr, 15) ^ bit_u64(scr, 18) ^ bit_u64(scr, 20) ^
           bit_u64(scr, 23) ^ bit_u64(scr, 25) ^ bit_u64(scr, 28) ^ bit_u64(scr, 30)) << 3;
    return sx & 0xF;
}

static uint8_t pcs_gen_sg(uint64_t scr)
{
    uint8_t sg = 0;
    sg |= (bit_u64(scr, 1) ^ bit_u64(scr, 5)) << 0;
    sg |= (bit_u64(scr, 4) ^ bit_u64(scr, 8) ^ bit_u64(scr, 9) ^ bit_u64(scr, 13)) << 1;
    sg |= (bit_u64(scr, 7) ^ bit_u64(scr, 11) ^ bit_u64(scr, 17) ^ bit_u64(scr, 21)) << 2;
    sg |= (bit_u64(scr, 10) ^ bit_u64(scr, 14) ^ bit_u64(scr, 15) ^ bit_u64(scr, 19) ^
           bit_u64(scr, 20) ^ bit_u64(scr, 24) ^ bit_u64(scr, 25) ^ bit_u64(scr, 29)) << 3;
    return sg & 0xF;
}

static uint8_t pcs_gen_sc(const pcs_tx_state_t *s, const pcs_tx_in_t *in, uint8_t sx, uint8_t sy)
{
    uint8_t sc = 0;

    if (s->tx_enable_d2) {
        sc |= (uint8_t)((sx & 0xF) << 4);
    }

    if (in->tx_mode == PCS_TX_MODE_SEND_Z) {
        /* Sc[3:1]=000 and Sc[0]=0 */
    } else {
        uint8_t sc31;
        if (((s->n - s->n0) & 1ULL) == 0ULL) {
            sc31 = sy & 0xE;                         /* Sy_n[3:1] */
        } else {
            sc31 = (uint8_t)((s->sy_prev ^ 0xE) & 0xE); /* Sy_{n-1}[3:1] xor 111 */
        }
        sc |= sc31;
        sc |= (sy & 0x1);
    }

    return sc;
}

static uint8_t pcs_gen_cs(const pcs_tx_state_t *s)
{
    uint8_t old = s->cs & 0x7;
    uint8_t cs = 0;
    cs |= bit_u8(old, 2) << 0;
    if (s->tx_enable_d2) {
        cs |= (bit_u16(s->sd_prev, 6) ^ bit_u8(old, 0)) << 1;
        cs |= (bit_u16(s->sd_prev, 7) ^ bit_u8(old, 1)) << 2;
    }
    return cs & 0x7;
}

static uint16_t pcs_gen_sd(const pcs_tx_state_t *s, const pcs_tx_in_t *in,
                          uint8_t sc, uint8_t cs_n, bool csreset)
{
    uint16_t sd = 0;
    uint8_t old_cs = s->cs & 0x7;
    bool cext     = (!in->tx_enable && in->tx_error && (in->txd == 0x0F));
    bool cext_err = (!in->tx_enable && in->tx_error && (in->txd != 0x0F) && !in->loc_lpi_req);

    sd |= (uint16_t)bit_u8(cs_n, 0) << 8;

    if (!csreset && s->tx_enable_d2) sd |= (uint16_t)(bit_u8(sc, 7) ^ bit_u8(in->txd, 7)) << 7;
    else if (csreset)                sd |= (uint16_t)bit_u8(old_cs, 1) << 7;
    else                             sd |= (uint16_t)bit_u8(sc, 7) << 7;

    if (!csreset && s->tx_enable_d2) sd |= (uint16_t)(bit_u8(sc, 6) ^ bit_u8(in->txd, 6)) << 6;
    else if (csreset)                sd |= (uint16_t)bit_u8(old_cs, 0) << 6;
    else                             sd |= (uint16_t)bit_u8(sc, 6) << 6;

    for (unsigned b = 4; b <= 5; ++b) {
        uint8_t v = s->tx_enable_d2 ? (bit_u8(sc, b) ^ bit_u8(in->txd, b)) : bit_u8(sc, b);
        sd |= (uint16_t)v << b;
    }

    if (s->tx_enable_d2) sd |= (uint16_t)(bit_u8(sc, 3) ^ bit_u8(in->txd, 3)) << 3;
    else if (in->loc_lpi_req && in->tx_mode != PCS_TX_MODE_SEND_Z) sd |= (uint16_t)(bit_u8(sc, 3) ^ 1U) << 3;
    else sd |= (uint16_t)bit_u8(sc, 3) << 3;

    if (s->tx_enable_d2) sd |= (uint16_t)(bit_u8(sc, 2) ^ bit_u8(in->txd, 2)) << 2;
    else if (in->loc_rcvr_status == PCS_RCVR_OK && in->tx_mode != PCS_TX_MODE_SEND_Z) sd |= (uint16_t)(bit_u8(sc, 2) ^ 1U) << 2;
    else sd |= (uint16_t)bit_u8(sc, 2) << 2;

    if (s->tx_enable_d2) sd |= (uint16_t)(bit_u8(sc, 1) ^ bit_u8(in->txd, 1)) << 1;
    else if (in->loc_update_done && in->tx_mode != PCS_TX_MODE_SEND_Z) sd |= (uint16_t)(bit_u8(sc, 1) ^ 1U) << 1;
    else sd |= (uint16_t)(bit_u8(sc, 1) ^ (uint8_t)cext_err) << 1;

    if (s->tx_enable_d2) sd |= (uint16_t)(bit_u8(sc, 0) ^ bit_u8(in->txd, 0)) << 0;
    else sd |= (uint16_t)(bit_u8(sc, 0) ^ (uint8_t)cext) << 0;

    return sd & 0x1FF;
}

static unsigned pcs_subset_col(uint16_t sd)
{
    /* Table column index uses vector Sd[6:8] = {Sd[6], Sd[7], Sd[8]}. */
    return ((unsigned)bit_u16(sd, 6) << 2) | ((unsigned)bit_u16(sd, 7) << 1) | bit_u16(sd, 8);
}

static pcs_symbol_condition_t pcs_select_condition(const pcs_tx_state_t *s,
                                                   const pcs_tx_in_t *in,
                                                   uint16_t sd,
                                                   bool csreset)
{
    (void)sd;

    bool carrier_ext_err_ind =
        (in->tx_error && s->tx_error_d1 && s->tx_error_d2 && (in->txd != 0x0F)) ||
        (in->tx_error && s->tx_error_d1 && s->tx_error_d2 && s->tx_error_d3 && (in->txd != 0x0F));

    if (s->tx_error_d1 && in->tx_enable && s->tx_enable_d2) {
        return PCS_COND_XMT_ERR;
    }

    if (csreset) {
        if (in->tx_error) {
            return (in->txd == 0x0F) ? PCS_COND_CSEXTEND : PCS_COND_CSEXTEND_ERR;
        }
        return PCS_COND_CSRESET;
    }

    if (in->tx_enable && !s->tx_enable_d1) {
        return PCS_COND_SSD1;
    }
    if (s->tx_enable_d1 && !s->tx_enable_d2) {
        return PCS_COND_SSD2;
    }

    if (!s->tx_enable_d2 && s->tx_enable_d3) {
        return carrier_ext_err_ind ? PCS_COND_ESD_EXT_ERR : PCS_COND_ESD1;
    }

    if (!s->tx_enable_d3 && s->tx_enable_d4) {
        if (carrier_ext_err_ind) {
            return PCS_COND_ESD_EXT_ERR;
        }
        if (!in->tx_error && !s->tx_error_d1) {
            return PCS_COND_ESD2_EXT_0;
        }
        if (!in->tx_error && s->tx_error_d1 && s->tx_error_d2 && s->tx_error_d3) {
            return PCS_COND_ESD2_EXT_1;
        }
        if (in->tx_error && s->tx_error_d1 && s->tx_error_d2 && s->tx_error_d3 && in->txd == 0x0F) {
            return PCS_COND_ESD2_EXT_2;
        }
        /* Conservative fallback for the fourth ESD symbol if no extension subcase matched. */
        return PCS_COND_ESD2_EXT_0;
    }

    if (!s->tx_enable_d2) {
        return PCS_COND_IDLE_CARR_EXT;
    }

    return PCS_COND_NORMAL;
}

static void pcs_lookup_base_symbols(pcs_symbol_condition_t cond, uint16_t sd,
                                    int8_t *ta, int8_t *tb, int8_t *tc, int8_t *td)
{
    unsigned subset = pcs_subset_col(sd);              /* 0..7 representing Sd[6:8] */
    unsigned row = sd & 0x3F;
    const int8_t *q = 0;

    switch (cond) {
    case PCS_COND_NORMAL:
        if ((subset & 1U) == 0U) q = pcs_tbl40_1_normal[row][subset >> 1];
        else                     q = pcs_tbl40_2_normal[row][subset >> 1];
        break;

    case PCS_COND_IDLE_CARR_EXT:
        q = pcs_tbl40_1_idle_carr_ext[row & 0x0F];
        break;

    case PCS_COND_XMT_ERR:
    case PCS_COND_CSEXTEND_ERR:
    case PCS_COND_CSEXTEND:
    case PCS_COND_CSRESET: {
        unsigned idx = 0;
        if (cond == PCS_COND_XMT_ERR) idx = 0;
        else if (cond == PCS_COND_CSEXTEND_ERR) idx = 1;
        else if (cond == PCS_COND_CSEXTEND) idx = 2;
        else idx = 3;
        if ((subset & 1U) == 0U) q = pcs_tbl40_1_special4[idx][subset >> 1];
        else                     q = pcs_tbl40_2_special4[idx][subset >> 1];
        break;
    }

    case PCS_COND_SSD1:         q = pcs_tbl40_1_special1[0]; break;
    case PCS_COND_SSD2:         q = pcs_tbl40_1_special1[1]; break;
    case PCS_COND_ESD1:         q = pcs_tbl40_1_special1[2]; break;
    case PCS_COND_ESD2_EXT_0:   q = pcs_tbl40_1_special1[3]; break;
    case PCS_COND_ESD2_EXT_1:   q = pcs_tbl40_1_special1[4]; break;
    case PCS_COND_ESD2_EXT_2:   q = pcs_tbl40_1_special1[5]; break;
    case PCS_COND_ESD_EXT_ERR:  q = pcs_tbl40_1_special1[6]; break;
    default:                    q = pcs_tbl40_1_idle_carr_ext[0]; break;
    }

    *ta = q[0]; *tb = q[1]; *tc = q[2]; *td = q[3];
}

static uint8_t pcs_pam5_to_tc3(int8_t v)
{
    /* Default 3-bit two's-complement packing. Modify this if the DUT uses another output encoding. */
    switch (v) {
    case -2: return 0x6;
    case -1: return 0x7;
    case  0: return 0x0;
    case +1: return 0x1;
    case +2: return 0x2;
    default: return 0x0;
    }
}

static uint16_t pcs_pack12_tc3(int8_t a, int8_t b, int8_t c, int8_t d)
{
    return (uint16_t)((pcs_pam5_to_tc3(a) << 9) |
                      (pcs_pam5_to_tc3(b) << 6) |
                      (pcs_pam5_to_tc3(c) << 3) |
                       pcs_pam5_to_tc3(d));
}

void pcs_tx_default_init_cfg(pcs_tx_init_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->role = PCS_ROLE_MASTER;

    /* These are deliberately centralized because they must match the DUT.
     * The IEEE text requires a non-zero 33-bit scrambler state, but the exact
     * implementation seed/phase is implementation-dependent in the excerpt.
     */
    cfg->scr_init_33 = 0x1ULL;       /* MODIFY for DUT/spec reset seed */
    cfg->cs_init = 0;
    cfg->sd_prev_init = 0;
    cfg->sy_prev_init = 0;
    cfg->tx_enable_d1_init = false;
    cfg->tx_enable_d2_init = false;
    cfg->tx_enable_d3_init = false;
    cfg->tx_enable_d4_init = false;
    cfg->tx_error_d1_init = false;
    cfg->tx_error_d2_init = false;
    cfg->tx_error_d3_init = false;
    cfg->n_init = 0;
    cfg->n0_init = 0;
    cfg->advance_scrambler_before_use = false;
}

void pcs_tx_init_state(pcs_tx_state_t *s, const pcs_tx_init_cfg_t *cfg)
{
    memset(s, 0, sizeof(*s));
    s->role = cfg->role;
    s->scr = cfg->scr_init_33 & PCS_SCR_MASK_33;
    s->cs = cfg->cs_init & 0x7;
    s->sd_prev = cfg->sd_prev_init & 0x1FF;
    s->sy_prev = cfg->sy_prev_init & 0xF;
    s->tx_enable_d1 = cfg->tx_enable_d1_init;
    s->tx_enable_d2 = cfg->tx_enable_d2_init;
    s->tx_enable_d3 = cfg->tx_enable_d3_init;
    s->tx_enable_d4 = cfg->tx_enable_d4_init;
    s->tx_error_d1 = cfg->tx_error_d1_init;
    s->tx_error_d2 = cfg->tx_error_d2_init;
    s->tx_error_d3 = cfg->tx_error_d3_init;
    s->n = cfg->n_init;
    s->n0 = cfg->n0_init;
    s->advance_scrambler_before_use = cfg->advance_scrambler_before_use;

    s->dut0_state = PCS_DUT0_ST_RESET;
    s->tx_enable_pipe = ((cfg->tx_enable_d4_init ? 1U : 0U) << 4) |
                        ((cfg->tx_enable_d3_init ? 1U : 0U) << 3) |
                        ((cfg->tx_enable_d2_init ? 1U : 0U) << 2) |
                        ((cfg->tx_enable_d1_init ? 1U : 0U) << 1);
    s->oe = true;
}

void pcs_tx_step(pcs_tx_state_t *s, const pcs_tx_in_t *in, pcs_tx_out_t *out)
{
    memset(out, 0, sizeof(*out));

    uint64_t scr_for_use;
    uint64_t scr_next;
    if (s->advance_scrambler_before_use) {
        scr_for_use = pcs_scrambler_advance(s->scr, s->role);
        scr_next = scr_for_use;
    } else {
        scr_for_use = s->scr & PCS_SCR_MASK_33;
        scr_next = pcs_scrambler_advance(s->scr, s->role);
    }

    uint8_t sy = pcs_gen_sy(scr_for_use);
    uint8_t sx = pcs_gen_sx(scr_for_use);
    uint8_t sg = pcs_gen_sg(scr_for_use);
    uint8_t sc = pcs_gen_sc(s, in, sx, sy);
    uint8_t cs_n = pcs_gen_cs(s);
    bool csreset = (s->tx_enable_d2 && !in->tx_enable);
    uint16_t sd = pcs_gen_sd(s, in, sc, cs_n, csreset);
    pcs_symbol_condition_t cond = pcs_select_condition(s, in, sd, csreset);

    int8_t ta, tb, tc, td;
    pcs_lookup_base_symbols(cond, sd, &ta, &tb, &tc, &td);

    bool srev = (s->tx_enable_d2 ^ s->tx_enable_d4);
    int8_t sna = ((bit_u8(sg,0) ^ (uint8_t)srev) == 0) ? +1 : -1;
    int8_t snb = ((bit_u8(sg,1) ^ (uint8_t)srev) == 0) ? +1 : -1;
    int8_t snc = ((bit_u8(sg,2) ^ (uint8_t)srev) == 0) ? +1 : -1;
    int8_t snd = ((bit_u8(sg,3) ^ (uint8_t)srev) == 0) ? +1 : -1;

    out->ta = ta; out->tb = tb; out->tc = tc; out->td = td;
    out->a = (int8_t)(ta * sna);
    out->b = (int8_t)(tb * snb);
    out->c = (int8_t)(tc * snc);
    out->d = (int8_t)(td * snd);
    out->packed12_tc3 = pcs_pack12_tc3(out->a, out->b, out->c, out->d);
    out->sx = sx; out->sy = sy; out->sg = sg;
    out->sc = sc;
    out->sd = sd;
    out->cs = cs_n;
    out->csreset = csreset;
    out->srev = srev;
    out->condition = cond;

    /* End-of-cycle state update. */
    s->scr = scr_next & PCS_SCR_MASK_33;
    s->cs = cs_n & 0x7;
    s->sd_prev = sd & 0x1FF;
    s->sy_prev = sy & 0xF;

    s->tx_enable_d4 = s->tx_enable_d3;
    s->tx_enable_d3 = s->tx_enable_d2;
    s->tx_enable_d2 = s->tx_enable_d1;
    s->tx_enable_d1 = in->tx_enable;

    s->tx_error_d3 = s->tx_error_d2;
    s->tx_error_d2 = s->tx_error_d1;
    s->tx_error_d1 = in->tx_error;

    s->n++;
}


/* -------------------------------------------------------------------------
 * DUTS26_0-compatible wrapper model
 *
 * The professor DUT is not a pure Clause-40 PCS core.  It has only Din[7:0]
 * and TX_EN at the top level, has a small internal delimiter FSM, uses the
 * MASTER scrambler seeded to 33'h1, uses the stored Scr value before advancing,
 * toggles OE for Sc[3:1], and packs each PAM5 symbol as 3-bit two's complement.
 *
 * This function models behavior, but it is intentionally written independently
 * from the DUT source structure so the reference model remains original work.
 * ------------------------------------------------------------------------- */

static uint8_t pcs_dut0_make_sc(const pcs_tx_state_t *s, uint8_t sx, uint8_t sy)
{
    bool txe2 = ((s->tx_enable_pipe >> 2) & 1U) != 0;
    uint8_t sc = 0;

    if (txe2) {
        sc |= (uint8_t)((sx & 0xF) << 4);
    }

    if (!s->oe) {
        sc |= (uint8_t)(sy & 0xE);
    } else {
        sc |= (uint8_t)((~s->sy_prev) & 0xE);
    }

    sc |= (uint8_t)(sy & 0x1);
    return sc;
}

static uint16_t pcs_dut0_make_sd_and_cs(const pcs_tx_state_t *s,
                                        uint8_t din,
                                        uint8_t sc,
                                        bool csreset,
                                        uint8_t *cs_next)
{
    bool txe2 = ((s->tx_enable_pipe >> 2) & 1U) != 0;
    uint8_t old_cs = s->cs & 0x7;
    uint16_t sd = 0;

    uint8_t sd7;
    uint8_t sd6;

    if (!csreset && txe2) {
        sd7 = bit_u8(sc, 7) ^ bit_u8(din, 7);
        sd6 = bit_u8(sc, 6) ^ bit_u8(din, 6);
    } else if (csreset) {
        /* Match DUTS26_0 behavior: both Sd[7] and Sd[6] use CSnm1[1]. */
        sd7 = bit_u8(old_cs, 1);
        sd6 = bit_u8(old_cs, 1);
    } else {
        sd7 = bit_u8(sc, 7);
        sd6 = bit_u8(sc, 6);
    }

    uint8_t cs = 0;
    cs |= (uint8_t)(bit_u8(old_cs, 2) << 0);
    if (txe2) {
        cs |= (uint8_t)((sd6 ^ bit_u8(old_cs, 1)) << 1);
        cs |= (uint8_t)((sd7 ^ bit_u8(din, 7)) << 2);
    }
    cs &= 0x7;

    sd |= (uint16_t)bit_u8(cs, 0) << 8;
    sd |= (uint16_t)sd7 << 7;
    sd |= (uint16_t)sd6 << 6;

    if (txe2) {
        sd |= (uint16_t)(bit_u8(sc, 5) ^ bit_u8(din, 5)) << 5;
        sd |= (uint16_t)(bit_u8(sc, 4) ^ bit_u8(din, 4)) << 4;
        sd |= (uint16_t)(bit_u8(sc, 3) ^ bit_u8(din, 3)) << 3;
        sd |= (uint16_t)(bit_u8(sc, 2) ^ bit_u8(din, 2)) << 2;
        sd |= (uint16_t)(bit_u8(sc, 1) ^ bit_u8(din, 1)) << 1;
    } else {
        sd |= (uint16_t)bit_u8(sc, 5) << 5;
        sd |= (uint16_t)bit_u8(sc, 4) << 4;
        sd |= (uint16_t)bit_u8(sc, 3) << 3;
        sd |= (uint16_t)bit_u8(sc, 2) << 2;
        sd |= (uint16_t)(bit_u8(sc, 1) ^ 1U) << 1;
    }

    if ((bit_u8(sc, 0) ^ (uint8_t)txe2) != 0U) {
        sd |= (uint16_t)bit_u8(din, 0);
    }

    *cs_next = cs;
    return sd & 0x1FF;
}

static pcs_symbol_condition_t pcs_dut0_select_condition(const pcs_tx_state_t *s,
                                                        bool tx_en,
                                                        bool *csreset,
                                                        uint8_t *next_state)
{
    *csreset = false;
    *next_state = s->dut0_state;

    switch ((pcs_dut0_state_t)s->dut0_state) {
    case PCS_DUT0_ST_RESET:
        *next_state = PCS_DUT0_ST_SEND_IDLE;
        return PCS_COND_IDLE_CARR_EXT;

    case PCS_DUT0_ST_SEND_IDLE:
        if (tx_en) {
            *next_state = PCS_DUT0_ST_SDD2;
            return PCS_COND_SSD1;
        }
        return PCS_COND_IDLE_CARR_EXT;

    case PCS_DUT0_ST_SDD1:
        *next_state = PCS_DUT0_ST_SDD2;
        return PCS_COND_SSD1;

    case PCS_DUT0_ST_SDD2:
        *next_state = PCS_DUT0_ST_TRANSMIT_DATA;
        return PCS_COND_SSD2;

    case PCS_DUT0_ST_TRANSMIT_DATA:
        if (tx_en) {
            return PCS_COND_NORMAL;
        }
        *csreset = true;
        *next_state = PCS_DUT0_ST_CSR2;
        return PCS_COND_CSRESET;

    case PCS_DUT0_ST_CSR2:
        *csreset = true;
        *next_state = PCS_DUT0_ST_ESD1;
        return PCS_COND_CSRESET;

    case PCS_DUT0_ST_ESD1:
        *next_state = PCS_DUT0_ST_ESD2;
        return PCS_COND_ESD1;

    case PCS_DUT0_ST_ESD2:
        *next_state = PCS_DUT0_ST_SEND_IDLE;
        return PCS_COND_ESD2_EXT_0;

    case PCS_DUT0_ST_CSR1:
    default:
        *next_state = PCS_DUT0_ST_SEND_IDLE;
        return PCS_COND_IDLE_CARR_EXT;
    }
}

void pcs_dut0_reset(pcs_tx_state_t *s)
{
    pcs_tx_init_cfg_t cfg;
    pcs_tx_default_init_cfg(&cfg);
    cfg.role = PCS_ROLE_MASTER;
    cfg.scr_init_33 = 0x1ULL;
    cfg.cs_init = 0;
    cfg.sd_prev_init = 0;
    cfg.sy_prev_init = 0;
    cfg.tx_enable_d1_init = false;
    cfg.tx_enable_d2_init = false;
    cfg.tx_enable_d3_init = false;
    cfg.tx_enable_d4_init = false;
    cfg.tx_error_d1_init = false;
    cfg.tx_error_d2_init = false;
    cfg.tx_error_d3_init = false;
    cfg.n_init = 0;
    cfg.n0_init = 0;
    cfg.advance_scrambler_before_use = false;
    pcs_tx_init_state(s, &cfg);

    s->dut0_state = PCS_DUT0_ST_RESET;
    s->tx_enable_pipe = 0;
    s->oe = true;
}

void pcs_dut0_step(pcs_tx_state_t *s, uint8_t din, bool tx_en, pcs_tx_out_t *out)
{
    memset(out, 0, sizeof(*out));

    uint64_t scr_for_use = s->scr & PCS_SCR_MASK_33;
    uint8_t sy = pcs_gen_sy(scr_for_use);
    uint8_t sx = pcs_gen_sx(scr_for_use);
    uint8_t sg = pcs_gen_sg(scr_for_use);
    uint8_t sc = pcs_dut0_make_sc(s, sx, sy);

    bool csreset;
    uint8_t next_state;
    pcs_symbol_condition_t cond = pcs_dut0_select_condition(s, tx_en, &csreset, &next_state);

    uint8_t cs_next;
    uint16_t sd = pcs_dut0_make_sd_and_cs(s, din, sc, csreset, &cs_next);

    int8_t ta, tb, tc, td;
    pcs_lookup_base_symbols(cond, sd, &ta, &tb, &tc, &td);

    bool srev = ((((s->tx_enable_pipe >> 4) ^ (s->tx_enable_pipe >> 2)) & 1U) != 0U);
    int8_t sna = ((bit_u8(sg, 0) ^ (uint8_t)srev) == 0U) ? +1 : -1;
    int8_t snb = ((bit_u8(sg, 1) ^ (uint8_t)srev) == 0U) ? +1 : -1;
    int8_t snc = ((bit_u8(sg, 2) ^ (uint8_t)srev) == 0U) ? +1 : -1;
    int8_t snd = ((bit_u8(sg, 3) ^ (uint8_t)srev) == 0U) ? +1 : -1;

    out->ta = ta; out->tb = tb; out->tc = tc; out->td = td;
    out->a = (int8_t)(ta * sna);
    out->b = (int8_t)(tb * snb);
    out->c = (int8_t)(tc * snc);
    out->d = (int8_t)(td * snd);
    out->packed12_tc3 = pcs_pack12_tc3(out->a, out->b, out->c, out->d);
    out->sx = sx; out->sy = sy; out->sg = sg;
    out->sc = sc;
    out->sd = sd;
    out->cs = cs_next;
    out->csreset = csreset;
    out->srev = srev;
    out->condition = cond;

    s->scr = pcs_scrambler_advance(s->scr, PCS_ROLE_MASTER);
    s->sy_prev = sy & 0xF;
    s->cs = cs_next & 0x7;
    s->sd_prev = sd & 0x1FF;
    s->dut0_state = next_state;
    s->tx_enable_pipe = (uint8_t)(((s->tx_enable_pipe << 1) | (tx_en ? 1U : 0U)) & 0x1F);
    s->oe = !s->oe;

    s->tx_enable_d1 = (s->tx_enable_pipe & 0x01U) != 0;
    s->tx_enable_d2 = (s->tx_enable_pipe & 0x02U) != 0;
    s->tx_enable_d3 = (s->tx_enable_pipe & 0x04U) != 0;
    s->tx_enable_d4 = (s->tx_enable_pipe & 0x08U) != 0;
    s->n++;
}

/* DPI-C convenience API for a single-scoreboard/single-DUT use case. */
static pcs_tx_state_t pcs_dpi_dut0_state;
static bool pcs_dpi_dut0_is_initialized = false;

void pcs_dpi_dut0_reset(void)
{
    pcs_dut0_reset(&pcs_dpi_dut0_state);
    pcs_dpi_dut0_is_initialized = true;
}

uint32_t pcs_dpi_dut0_step(uint32_t din, uint32_t tx_en)
{
    if (!pcs_dpi_dut0_is_initialized) {
        pcs_dpi_dut0_reset();
    }

    pcs_tx_out_t out;
    pcs_dut0_step(&pcs_dpi_dut0_state, (uint8_t)(din & 0xFFU), tx_en != 0U, &out);
    return (uint32_t)(out.packed12_tc3 & 0x0FFFU);
}

void pcs_dpi_dut0_step_debug(uint32_t din, uint32_t tx_en,
                             uint32_t *dout12,
                             uint32_t *sc,
                             uint32_t *sd,
                             uint32_t *condition,
                             uint32_t *state)
{
    if (!pcs_dpi_dut0_is_initialized) {
        pcs_dpi_dut0_reset();
    }

    pcs_tx_out_t out;
    pcs_dut0_step(&pcs_dpi_dut0_state, (uint8_t)(din & 0xFFU), tx_en != 0U, &out);

    if (dout12)    *dout12 = (uint32_t)(out.packed12_tc3 & 0x0FFFU);
    if (sc)        *sc = (uint32_t)(out.sc & 0xFFU);
    if (sd)        *sd = (uint32_t)(out.sd & 0x1FFU);
    if (condition) *condition = (uint32_t)out.condition;
    if (state)     *state = (uint32_t)pcs_dpi_dut0_state.dut0_state;
}

void pcs_cmd_decode_table_init_default(pcs_cmd_decode_table_t *t)
{
    memset(t, 0, sizeof(*t));
    for (unsigned i = 0; i < 256; ++i) {
        t->valid[i] = true;
        t->entry[i].tx_enable = false;
        t->entry[i].tx_error = false;
        t->entry[i].txd = (uint8_t)i;
        t->entry[i].tx_mode = PCS_TX_MODE_SEND_N;
        t->entry[i].loc_lpi_req = false;
        t->entry[i].loc_rcvr_status = PCS_RCVR_NOT_OK;
        t->entry[i].loc_update_done = false;
    }
}

void pcs_cmd_decode_table_set(pcs_cmd_decode_table_t *t, uint8_t cmd, const pcs_tx_in_t *meaning)
{
    t->valid[cmd] = true;
    t->entry[cmd] = *meaning;
}

bool pcs_project9_decode(uint16_t in9, const pcs_cmd_decode_table_t *cmd_table, pcs_tx_in_t *out)
{
    if ((in9 & 0x100U) == 0U) {
        out->tx_enable = true;
        out->tx_error = false;
        out->txd = (uint8_t)(in9 & 0xFFU);
        out->tx_mode = PCS_TX_MODE_SEND_N;
        out->loc_lpi_req = false;
        out->loc_rcvr_status = PCS_RCVR_NOT_OK;
        out->loc_update_done = false;
        return true;
    }

    uint8_t cmd = (uint8_t)(in9 & 0xFFU);
    if (!cmd_table || !cmd_table->valid[cmd]) {
        return false;
    }
    *out = cmd_table->entry[cmd];
    return true;
}

bool pcs_project9_step(pcs_tx_state_t *s, uint16_t in9,
                       const pcs_cmd_decode_table_t *cmd_table,
                       pcs_tx_out_t *out)
{
    pcs_tx_in_t spec_in;
    if (!pcs_project9_decode(in9, cmd_table, &spec_in)) {
        return false;
    }
    pcs_tx_step(s, &spec_in, out);
    return true;
}

#ifdef __cplusplus
}
#endif
