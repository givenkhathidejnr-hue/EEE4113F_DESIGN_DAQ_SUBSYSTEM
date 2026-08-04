#ifndef DAQ_H
#define DAQ_H

#include <stdint.h>
#include <stddef.h>

#include "stm32l4xx_hal.h"

/* ═══════════════════════════════════════════════════════════════════════
 *  CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════════ */

#define DETECTOR_THRESHOLD      50u          /* Min valid ADC delta (raw counts)   */
#define FAULT_RETRY_LIMIT        3u          /* Max retries before aborting cycle  */
#define RAM_BUFFER_DEPTH        16u          /* Records buffered before SD flush   */
#define MCP9808_I2C_ADDR        (0x18 << 1) /* 7-bit addr 0x18, shifted for HAL  */
#define TEMP_REF_CELSIUS        25.0f        /* Calibration reference temperature  */
#define TEMP_ALPHA              -0.015f      /* Fluorescence sensitivity coeff.    */

/*
 * Acquisition window parameters — chosen to respect the Sallen-Key
 * filter time constant (τ = RC ≈ 150 ms, 3τ ≈ 450 ms settling time).
 *
 * SAMPLE_HALF_PERIOD_MS: time the system waits in each excitation state
 *   (ON or OFF) before triggering the ADC. Must be >= 3τ ≈ 450 ms.
 *   Set to 500 ms to give full filter settling before each sample.
 *
 * ACQUISITION_PAIRS: on/off pairs per 30-second window.
 *   At 1 pair/second → 30 pairs.
 *   ENOB gain = ½ × log₂(30) ≈ 2.45 bits → effective ~14.45 bits.
 *
 * IWDG_TIMEOUT_MS: must exceed worst-case active-cycle time.
 *   30 pairs × 1 s + init/logging overhead ≈ 45 s → 90 s timeout.
 */
#define SAMPLE_HALF_PERIOD_MS   500u         /* ms per half-cycle (on or off)     */
#define ACQUISITION_PAIRS       30u          /* on/off pairs per measurement      */
#define IWDG_TIMEOUT_MS         90000u       /* 90 s IWDG window                  */
#define FAILED_LOG_BUFFER		1024u		/* Cyclic buffer to store failed logs */


/* ── GPIO pin assignments ───────────────────────────────────────────────
 *
 *  EN  pins: active-high  (SET  = domain ON)
 *  FLT pins: active-low   (RESET = fault asserted), internal pull-up assumed
 */

/* Detection domain */
#define DETECT_EN_PORT      GPIOA
#define DETECT_EN_PIN       GPIO_PIN_8
#define DETECT_FLT_PORT     GPIOA
#define DETECT_FLT_PIN      GPIO_PIN_9

/* Excitation domain */
#define EXCITE_EN_PORT      GPIOC
#define EXCITE_EN_PIN       GPIO_PIN_7
#define EXCITE_FLT_PORT     GPIOB
#define EXCITE_FLT_PIN      GPIO_PIN_6

/* SD card domain */
#define SDCARD_EN_PORT      GPIOA
#define SDCARD_EN_PIN       GPIO_PIN_6
#define SDCARD_FLT_PORT     GPIOA
#define SDCARD_FLT_PIN      GPIO_PIN_7

/* ── Timing constants (ms) ──────────────────────────────────────────────
 *
 *  DETECT/EXCITE settle: 3× Tau of the Sallen-Key filter (RC ≈ 150 ms)
 *  to ensure >95% settling before the first ADC sample.
 */
#define DELAY_DETECT_SETTLE_MS      500u
#define DELAY_EXCITE_SETTLE_MS      500u
#define DELAY_SDCARD_SETTLE_MS      500u
#define DELAY_DOMAIN_GAP_MS           2u


/* ── Subsystem identifiers ─────────────────────────────────────────────── */

typedef enum {
    SUBSYS_PROCESSOR  = 0,
    SUBSYS_DETECTION,
    SUBSYS_EXCITATION,
    SUBSYS_SDCARD,
    SUBSYS_TEMP_SENSOR
} SubSystem;

/* ── Fault types ───────────────────────────────────────────────────────── */

/**
 * Used as the primary return type for all DAQ functions.
 * FAULT_NONE (== 0) signals success, any other value is a self-describing
 * error — callers never need a separate status enum to understand what
 * went wrong.
 *
 * Fault sources mirror §1.6.3 and DAQ-ATP-06.
 */
typedef enum {
    FAULT_NONE                      = 0,   /* Success / no fault                    */
    FAULT_HEALTH_CHECK,                    /* Failed post-wake health verification  */
    FAULT_INVALID_DETECTOR_RESPONSE,       /* ADC delta below DETECTOR_THRESHOLD    */
    FAULT_STORAGE_FAILURE,                 /* SD card write or mount error          */
    FAULT_LOAD_SWITCH_DETECTION,           /* Detection domain load switch asserted */
    FAULT_LOAD_SWITCH_EXCITATION,          /* Excitation domain load switch asserted*/
    FAULT_LOAD_SWITCH_SDCARD,             /* SD card domain load switch asserted   */
    FAULT_DAC_ERROR,                       /* DAC configuration or start failure    */
    FAULT_ADC_TIMEOUT,                     /* ADC poll timed out mid-window         */
    FAULT_TEMP_SENSOR,                     /* MCP9808 I2C read failure              */
	FAULT_TEMP_ALERT,                     /* MCP9808 ALERT pin — out-of-range temp */
	FAULT_CRC_MISMATCH,                   /* Record CRC failed on verify           */
	FAULT_NULL_POINTER,                    /* NULL passed to a function that needs  */
    FAULT_BUFFER_FULL                      /* RAM log buffer exhausted              */
} FaultType;

/* ── System state machine ──────────────────────────────────────────────── */
typedef enum {
    SYS_STOP        = 0,   /* Low-power Stop 2 mode; RTC alarm pending     */
    SYS_INIT,              /* Post-wake initialisation and health check     */
    SYS_POWER_SEQ,         /* Enable detection domain; await stabilisation  */
    SYS_BASELINE,          /* Record baseline with excitation off           */
    SYS_ACQUIRE,           /* Apply excitation; sample ADC window           */
    SYS_PROCESS,           /* Average samples; form measurement record      */
    SYS_LOG,               /* Append record to RAM buffer; flush if due     */
    SYS_FAULT              /* Fault detected; log, retry or abort           */
} SystemState;

/* ═══════════════════════════════════════════════════════════════════════
 *  TIMER-DRIVEN ACQUISITION STATE
 *
 *  The hardware timer fires every SAMPLE_HALF_PERIOD_MS (500 ms).
 *  On each tick the ISR advances through OFF-settle → OFF-sample →
 *  ON-settle → ON-sample, toggling the excitation GPIO and starting
 *  one ADC conversion per sample phase.
 * ═══════════════════════════════════════════════════════════════════════ */

typedef enum {
    ACQ_IDLE       = 0,
    ACQ_WAIT_OFF,      /* Excitation OFF — waiting SAMPLE_HALF_PERIOD_MS   */
    ACQ_SAMPLE_OFF,    /* Trigger ADC with excitation off                  */
    ACQ_WAIT_ON,       /* Excitation ON  — waiting SAMPLE_HALF_PERIOD_MS   */
    ACQ_SAMPLE_ON,     /* Trigger ADC with excitation on                   */
} AcqPhase;

typedef struct {
    volatile AcqPhase  phase;
    volatile uint16_t  pair_count;           /* Completed pairs so far     */
    volatile uint8_t   complete;             /* 1 when window finished     */
    volatile FaultType fault;                /* Non-NONE if ISR hit error  */
    uint16_t           buf_on [ACQUISITION_PAIRS];
    uint16_t           buf_off[ACQUISITION_PAIRS];
} AcqState;

/* ── Pending fault (set from ISR, consumed by state machine) ───────────── */
typedef struct {
    volatile FaultType  type;
    volatile SubSystem  subsystem;
} PendingFault;

/* ── Runtime context ───────────────────────────────────────────────────── */
typedef struct {
    SystemState     state;
    uint32_t        cycle_count;    /* Total completed sleep/wake cycles         */
} DaqContext;

/* ── Fault manager ─────────────────────────────────────────────────────── */

/**
 * Persists fault tracking across Stop 2 sleep cycles.
 * Placed in .noinit so values survive sleep without re-zeroing.
 */
typedef struct {
    FaultType   last_fault;
    SubSystem   last_subsystem;
    uint8_t     retry_count;        /* Resets to 0 after a clean cycle           */
    uint32_t    total_faults;       /* Lifetime fault counter                    */
} FaultManager;

/* ── Measurement record ────────────────────────────────────────────────── */

/**
 * One log entry written to the SD card. 
 */
typedef struct __attribute__((aligned(4))){
    uint32_t    timestamp;          /* RTC epoch at time of acquisition          */
    float       adc_differential;  /* mean(on) - mean(off), temp-compensated    */
    float       adc_stddev;        /* Std-dev of excitation-on samples          */
    float       temperature_c;     /* MCP9808 reading logged at end of window   */
    uint8_t 	cycle_count;
    FaultType   op_status;         /* FAULT_NONE = clean record, else degraded  */
    uint8_t     _padding[2];        /* Manual padding to ensure 32-bit alignment */
    uint32_t    crc32;              /* Always the last member */
} MeasurementRecord;

/* ── Fault log entry ───────────────────────────────────────────────────── */

typedef struct {
    uint32_t    timestamp;
    SubSystem   subsystem;
    SystemState state_at_fault;
    FaultType   fault;
    uint8_t     retry_count;
} FaultLog;

/* ── Global state (defined in daq.c) ──────────────────────────────────── */

extern DaqContext    	g_daq;
extern FaultManager  	g_fault_mgr;
extern PendingFault  	g_pending_fault;   /* Written by ISR, read by health check  */
extern AcqState      	g_acq;

/* ── Initialisation / power sequencing ────────────────────────────────── */
FaultType init_power_domain(GPIO_TypeDef *en_port, uint16_t en_pin,
                            GPIO_TypeDef *flt_port, uint16_t flt_pin,
                            uint32_t settle_delay_ms, FaultType expected_fault);

/* ── Health and fault monitoring ───────────────────────────────────────── */

/**
 * @brief Poll ISR latch and all fault GPIO lines.
 * @return FAULT_NONE if all domains healthy, else specific fault type.
 */
FaultType   check_power_health(void);

/**
 * @brief Validate a raw ADC delta against DETECTOR_THRESHOLD.
 * @return FAULT_INVALID_DETECTOR_RESPONSE or FAULT_NONE.
 */
FaultType   check_detector_response(uint16_t adc_delta);

/**
 * @brief Central fault handler: logs, retries or aborts, always safe-states.
 */
void        fault_handler(FaultType fault, SubSystem subsystem);

/* ── EXTI callbacks (implement in daq.c, called from stm32xx_it.c) ────── */

void        DAQ_EXTI_Detection_Callback(void);
void        DAQ_EXTI_Excitation_Callback(void);
void        DAQ_EXTI_SDCard_Callback(void);
void        DAQ_EXTI_TempAlert_Callback(void);
void        DAQ_TIM_PeriodElapsed_Callback(void);
void        DAQ_ADC_ConvCplt_Callback(void);

/* ── Acquisition ───────────────────────────────────────────────────────── */
FaultType   start_acquisition(void);
FaultType   measure_baseline(uint16_t *buf, uint16_t window_size);
FaultType   excite_system(uint16_t dac_value);
FaultType   sample_data(uint16_t *buffer, uint16_t window_size);

/* ── Processing ────────────────────────────────────────────────────────── */

/**
 * @brief Compute arithmetic mean and stddev of a uint16 sample buffer.
 */
void compute_stats(const uint16_t *buf, size_t n, float *mean, float *stddev);

/**
 * @brief Differential processing + temperature compensation.
 * @return FAULT_NONE on success, FAULT_TEMP_SENSOR if MCP9808 fails
 */
FaultType   process_measurement(const uint16_t *on_buf,
                                const uint16_t *off_buf,
                                size_t          n,
                                MeasurementRecord *out);

/* ── Temperature sensor ────────────────────────────────────────────────── */

/**
 * @brief Wake MCP9808, read temperature, return to shutdown.
 * @param[out] temp_c  Measured temperature in °C.
 * @return FAULT_NONE or FAULT_TEMP_SENSOR.
 */
FaultType   read_mcp9808_temp(float *temp_c);
FaultType   configure_mcp9808_alert(void);

/* CRC */
uint32_t    crc32_record(const MeasurementRecord *rec);
FaultType   verify_record_crc(const MeasurementRecord *rec);

/* ── Logging ───────────────────────────────────────────────────────────── */

FaultType   log_measurement(const MeasurementRecord *record);
FaultType  	log_fault_to_sd(const FaultLog *entry);
FaultType  	log_fault_to_buffer(const FaultLog *entry);
FaultType   flush_buffer_to_sd(void);

/* ── Power management ──────────────────────────────────────────────────── */

void        enter_stop_mode(void);
void        disable_all_domains(void);

/* Watchdog */
void        iwdg_kick(void);

/* Debug */
void debug_dump_sd(void);

#endif /* DAQ_H */
