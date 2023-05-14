
#ifndef _IR_REMOTE_HPP
#define _IR_REMOTE_HPP

#define VERSION_IRREMOTE "4.0.0"
#define VERSION_IRREMOTE_MAJOR 4
#define VERSION_IRREMOTE_MINOR 0
#define VERSION_IRREMOTE_PATCH 0

#define VERSION_HEX_VALUE(major, minor, patch) ((major << 16) | (minor << 8) | (patch))
#define VERSION_IRREMOTE_HEX  VERSION_HEX_VALUE(VERSION_IRREMOTE_MAJOR, VERSION_IRREMOTE_MINOR, VERSION_IRREMOTE_PATCH)

#define DECODE_NEC          // Includes Apple and Onkyo

#if defined(DECODE_NEC) && !(~(~DECODE_NEC + 0) == 0 && ~(~DECODE_NEC + 1) == 1)
#warning "The macros DECODE_XXX no longer require a value. Decoding is now switched by defining / non defining the macro."
#endif

//#define DEBUG // Activate this for lots of lovely debug output from the IRremote core.

/****************************************************
 *                    RECEIVING
 ****************************************************/
/**
 * MARK_EXCESS_MICROS is subtracted from all marks and added to all spaces before decoding,
 * to compensate for the signal forming of different IR receiver modules
 * For Vishay TSOP*, marks tend to be too long and spaces tend to be too short.
 * If you set MARK_EXCESS_MICROS to approx. 50us then the TSOP4838 works best.
 * At 100us it also worked, but not as well.
 * Set MARK_EXCESS to 100us and the VS1838 doesn't work at all.
 *
 * The right value is critical for IR codes using short pulses like Denon / Sharp / Lego
 *
 *  Observed values:
 *  Delta of each signal type is around 50 up to 100 and at low signals up to 200. TSOP is better, especially at low IR signal level.
 *  VS1838      Mark Excess -50 at low intensity to +50 us at high intensity
 *  TSOP31238   Mark Excess 0 to +50
 */
#if !defined(MARK_EXCESS_MICROS)
// To change this value, you simply can add a line #define "MARK_EXCESS_MICROS <My_new_value>" in your ino file before the line "#include <IRremote.hpp>"
#define MARK_EXCESS_MICROS    20
#endif

/**
 * Minimum gap between IR transmissions, to detect the end of a protocol.
 * Must be greater than any space of a protocol e.g. the NEC header space of 4500 us.
 * Must be smaller than any gap between a command and a repeat; e.g. the retransmission gap for Sony is around 15 ms for Sony20 protocol.
 * Keep in mind, that this is the delay between the end of the received command and the start of decoding.
 */

#if !defined(RECORD_GAP_MICROS)
// To change this value, you simply can add a line #define "RECORD_GAP_MICROS <My_new_value>" in your ino file before the line "#include <IRremote.hpp>"
#define RECORD_GAP_MICROS   5000 // FREDRICH28AC / LG2 header space is 9700, NEC header space is 4500
#endif


#if !defined(RECORD_GAP_MICROS_WARNING_THRESHOLD)
// To change this value, you simply can add a line #define "RECORD_GAP_MICROS_WARNING_THRESHOLD <My_new_value>" in your ino file before the line "#include <IRremote.hpp>"
#define RECORD_GAP_MICROS_WARNING_THRESHOLD   15000
#endif

/** Minimum gap between IR transmissions, in MICROS_PER_TICK */
#define RECORD_GAP_TICKS    (RECORD_GAP_MICROS / MICROS_PER_TICK) // 100

/*
 * Activate this line if your receiver has an external output driver transistor / "inverted" output
 */
//#define IR_INPUT_IS_ACTIVE_HIGH
#if defined(IR_INPUT_IS_ACTIVE_HIGH)
// IR detector output is active high
#define INPUT_MARK   1 ///< Sensor output for a mark ("flash")
#else
// IR detector output is active low
#define INPUT_MARK   0 ///< Sensor output for a mark ("flash")
#endif

/*

#include "IRremoteInt.h"


#include "IRFeedbackLED.hpp"

#include "LongUnion.h" // used in most decoders

#include "IRProtocol.hpp" // must be first, it includes definition for PrintULL (unsigned long long)

#include "IRReceive.hpp"

#include "ir_NEC.hpp"
*/

/*
#define RAWBUF  101  // Maximum length of raw duration buffer
#define REPEAT 0xFFFFFFFF
#define USECPERTICK MICROS_PER_TICK
#define MARK_EXCESS MARK_EXCESS_MICROS
*/

