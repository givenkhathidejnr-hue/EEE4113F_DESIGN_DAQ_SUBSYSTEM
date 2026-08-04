/*
 * mcp9808.h
 *
 *  Created on: May 8, 2026
 *      Author: simph
 */

#ifndef INC_MCP9808_H_
#define INC_MCP9808_H_

#define MCP9808_I2C_ADDR    (0x18 << 1)  /* Shifted for STM32 HAL compatibility */

/* ── MCP9808 Register Map ─────────────────────────────────────────────── */

#define MCP9808_REG_RESERVED    0x00u   /* Reserved                         */
#define MCP9808_REG_CONFIG      0x01u   /* Configuration Register           */
#define MCP9808_REG_TUPPER      0x02u   /* Upper Temperature Limit          */
#define MCP9808_REG_TLOWER      0x03u   /* Lower Temperature Limit          */
#define MCP9808_REG_TCRIT       0x04u   /* Critical Temperature Limit       */
#define MCP9808_REG_TAMBIENT    0x05u   /* Ambient Temperature Data         */
#define MCP9808_REG_MANUF_ID    0x06u   /* Manufacturer ID (0x0054)         */
#define MCP9808_REG_DEVICE_ID   0x07u   /* Device ID/Revision (0x0400)      */
#define MCP9808_REG_RESOLUTION  0x08u   /* Temperature Resolution           */

/* ── Configuration Register Bit Definitions ────────────────────────────── */

#define MCP9808_CONFIG_DEFAULT    0x0000u
#define MCP9808_CONFIG_SHUTDOWN   (1u << 8)   /* Bit 8: 1 = Low Power Mode  */
#define MCP9808_CONFIG_CRIT_ONLY  (1u << 2)   /* Bit 2: 1 = TCRIT only      */
#define MCP9808_CONFIG_ALERT_POL  (1u << 1)   /* Bit 1: 1 = Active High     */
#define MCP9808_CONFIG_ALERT_EN   (1u << 0)   /* Bit 0: 1 = Alert Enabled   */

/* ── Temperature Alert Thresholds (Celsius) ────────────────────────────── */

#define TEMP_ALERT_UPPER_C        35.0f   /* Typical max marine operational temp */
#define TEMP_ALERT_LOWER_C         2.0f   /* Typical min marine operational temp */
#define TEMP_ALERT_CRIT_C         45.0f   /* Safety limit for electronics        */

/* ── Resolution Settings ───────────────────────────────────────────────── */

#define MCP9808_RES_0_5_C         0x00u   /* 30 ms conversion time  */
#define MCP9808_RES_0_25_C        0x01u   /* 65 ms conversion time  */
#define MCP9808_RES_0_125_C       0x02u   /* 130 ms conversion time */
#define MCP9808_RES_0_0625_C      0x03u   /* 250 ms conversion time (Default) */

#endif /* INC_MCP9808_H_ */
