/*
 * debug_strings.h
 *
 *  Created on: May 12, 2026
 *      Author: simph
 */

#ifndef DEBUG_STRINGS_H_
#define DEBUG_STRINGS_H_

#include "daq.h"

/* Allows for UART data transmission during debugging */
#define DAQ_DEBUG 1u

#if DAQ_DEBUG
  	  #define DAQ_LOG(fmt, ...) \
		printf("[%lu] " fmt "\r\n", (unsigned long)rtc_now(), ##__VA_ARGS__)
#else
  #define DAQ_LOG(fmt, ...) /* nothing */
#endif

/* Definitions live in daq.c, the only .c file that includes this header
 * today -- declared extern here (rather than defined directly in the
 * header, as before) so a second translation unit can include this header
 * without a "multiple definition" link error. */
extern const char *const FAULT_NAMES[];
extern const char *const SUBSYS_NAMES[];
extern const char *const STATE_NAMES[];

/* sizeof() here only works in a translation unit that can also see the
 * array's real definition (daq.c, after the point it defines them) --
 * an `extern`-only declaration has no known size. Not usable from a
 * second .c file without also seeing the definitions directly. */
#define FAULT_NAMES_COUNT  (sizeof(FAULT_NAMES)  / sizeof(FAULT_NAMES[0]))
#define SUBSYS_NAMES_COUNT (sizeof(SUBSYS_NAMES) / sizeof(SUBSYS_NAMES[0]))
#define STATE_NAMES_COUNT  (sizeof(STATE_NAMES)  / sizeof(STATE_NAMES[0]))

#endif /* DEBUG_STRINGS_H_ */
