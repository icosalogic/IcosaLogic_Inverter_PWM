/*
 * This file contains definitions for fine-grained logging of inverter events.
 * This gives the ability to some insight into the behavior of the interrupt
 * handlers, which should not be perturbed by print statements or other crude
 * debugging techniques that might prevent the inverter from behaving normally.
 * 
 * We use the CPU's debug/trace counter to timestamp log entries.
 * This implementation is SAMD specific.
 */

#ifndef _I20InverterLogging
#define _I20InverterLogging

// Comment out this line to turn off logging and remove logging code

#define I20DoLogging 1

typedef enum {
  I20LogTypeUnused,
  I20LogTypeTcc,
  I20LogTypeAdcHandler,
  I20LogTypeAdcStart,
  I20LogTypeAdcNext,
  I20LogTypePwm,
  I20LogTypeExtension,  
} I20LogType;

const char *logTypeNames[] = {"unused", "TCC", "ADCH", "ADCS", "ADCN", "PWM", "ext"};

typedef struct {
  uint32_t start;              // ticks at IRQ start of IRQ call
  uint32_t stop;               // ticks at end
  uint8_t logType;             // number of IRQ calls
  uint8_t vu8;                 // unsigned 8-bit value
  uint16_t vu16;               // unsigned 16-bit value
} LogEntry;                    //                        struct length == 12 bytes


const int logSize = 30 * 1000;

#ifdef I20DoLogging
const uint32_t numLogEntries = logSize / sizeof(LogEntry);
#else
const uint32_t numLogEntries = 10;
#endif

LogEntry eventLog[numLogEntries];
LogEntry* const firstLogEntry = &(eventLog[0]);
LogEntry* const wrapLogEntry  = &(eventLog[numLogEntries]);
LogEntry* curLogEntry = firstLogEntry;
LogEntry* nxtLogEntry = firstLogEntry + 1;
LogEntry* dmpLogEntry = firstLogEntry;
LogEntry zeroEntry;

#ifdef I20DoLogging

/*
 * Sets curLogEntry to the next entry in the log buffer.
 * Wraps from the end of the buffer back to the front.
 */
inline void logNextEntry() {
  curLogEntry = nxtLogEntry;
  nxtLogEntry += 1;
  if (nxtLogEntry >= wrapLogEntry) {
    nxtLogEntry = firstLogEntry;
  }
}

/*
 * Gets the current clock ticks, and saves it as the start time of
 * the next log buffer entry.  We are careful to call logNextEntry()
 * after getting the start tick count, so it is included in the
 * overhead.
 */
inline void logEntryStart(uint8_t logType) {
  uint32_t start = DWT->CYCCNT;
  logNextEntry();
  curLogEntry->start = start;
  curLogEntry->logType = logType;
}

/*
 * Sets the stop time of the current log entry to the current tick count.
 */
inline void logEntryStop() {
  curLogEntry->stop = DWT->CYCCNT;
}

/*
 * Set the 8-bit unsigned log entry value.
 */
inline void logEntryVu8(uint8_t val) {
  curLogEntry->vu8 = val;
}

/*
 * Set the 16-bit unsigned log entry value.
 */
inline void logEntryVu16(uint16_t val) {
  curLogEntry->vu16 = val;
}

/*
 * Set both the 8-bit and 16-bit unsigned log entry values.
 */
inline void logEntryExt(uint8_t val1, uint16_t val2) {
  uint32_t start = DWT->CYCCNT;
  logNextEntry();
  curLogEntry->start = start;
  curLogEntry->logType = I20LogTypeExtension;
  curLogEntry->vu8  = val1;
  curLogEntry->vu16 = val2;
  curLogEntry->stop = DWT->CYCCNT;
}

inline void setupLog() {
  memset(&zeroEntry, 0, sizeof(LogEntry));
  memset(firstLogEntry, 0, sizeof(eventLog));
  curLogEntry = firstLogEntry;
  
  // Do 11 timestamp reads in a row to see how performant it is
  uint32_t s1  = DWT->CYCCNT;
  uint32_t s2  = DWT->CYCCNT;
  uint32_t s3  = DWT->CYCCNT;
  uint32_t s4  = DWT->CYCCNT;
  uint32_t s5  = DWT->CYCCNT;
  uint32_t s6  = DWT->CYCCNT;
  uint32_t s7  = DWT->CYCCNT;
  uint32_t s8  = DWT->CYCCNT;
  uint32_t s9  = DWT->CYCCNT;
  uint32_t s10 = DWT->CYCCNT;
  uint32_t s11 = DWT->CYCCNT;
  logNextEntry();
  uint32_t s12 = DWT->CYCCNT;
  
  Serial.printf("    timestamp total for 10 samples is %d ticks, nextEntry is %d ticks\n",
		s11 - s1, s12 - s11);

  curLogEntry = firstLogEntry;        // undo effect of logNextEntry() test above
  nxtLogEntry = firstLogEntry + 1;
}

#else // I20DoLogging

inline void setupLog() { }

inline void logNextEntry() { }

inline void logEntryStart(uint8_t logType) { }

inline void logEntryStop() { }

inline void logEntryVu8(uint8_t val) { }

inline void logEntryVu16(uint16_t val) { }

#endif // I20DoLogging
  
#endif // _I20InverterLogging
