#include "GoertzelJS.h"

/*
 * In order to read more clearly the energy values, we are scaling them
 * up by 10^5
 */
#define TEN_P_FIVE 100000
#define ScaleEnergy(en) (TEN_P_FIVE*en)

/*
 * Number of samples to be taken by the ADC and then fed to Goertzel's.
 */
#define N_SAMPLES 96
double samples[N_SAMPLES];

/*
 * Frequencies to be detected by Goertzel. Values in Hz.
 * 
 * 40 KHz encodes a `0`,
 * 45 KHz encodes a `1`
 */
double TARGET_FREQUENCIES[] {40000, 45000};
#define N_TARGETS sizeof(TARGET_FREQUENCIES)/sizeof(double)

#define MeasureSampleFreq 0
/*
 * To find the value that must be set on SAMPLE_FREQ, change
 * `MeasureSampleFreq` from 0 to 1. This will activate the
 * `MeasureTime(...)` block around FillBuffer, effectively
 * measuring the time elapsed when sampling `N_SAMPLES` samples
 * from the ADC (in us).
 * 
 * Given that we want SAMPLE_FREQ in Hz (i. e. samples per second), the
 * formula goes as follows:
 * 		SAMPLE_FREQ = (10^6/[measured time in us])*N_SAMPLES
 *
 * Example: for 256 samples taken during 880 us, SAMPLE_FREQ should
 *			be set to 290909 = round(10^6/880*256)
 * 
 * Once SAMPLE_FREQ is properly set, you can change back
 * `MeasureSampleFreq` from 1 to 0, to enable Goertzel processing.
 */
/*
 * 256 samples took 747 ms
 * 128 samples took 364 ms
 * 96 sam. took 278.5 ms
 * 80 sam. took 231.8 ms
 */
#define SAMPLE_FREQ 344703
/*
 * Detection threshold. Must be high enough to distinguish signal from noise,
 * but low enough to avoid missing actual symbols.
 */
#define THRESHOLD 4500 // As energies are scaled up, this is actually *10^-5

/*
 * Length of the time each symbol will be transmitted.
 */
#define symbolTime 35 // ms

/*
 * Set to 0 to show the detected symbols (or 'No Signal', 'Ambiguous').
 * Set to 1 to show the values of GoertzelJS's internal arrays.
 * (which is useful to check for overflows and NaNs)
 * DEBUG doesn't work when MeasureSampleFreq is set.
 */
#define DEBUG 0

/*
 * Macro which behaves like a function surrounding a statement.
 * Returns the elapsed time during the execution of STMT, in microseconds.
 */
unsigned long start_time, stop_time;
#define MeasureTime(STMT) \
  ({                      \
    start_time = micros();\
    STMT                  \
    stop_time = micros(); \
    stop_time-start_time; \
  })
/*
 * Ditto, but in milliseconds.
 */
#define MeasureMillis(STMT) \
  ({                      \
    start_time = millis();\
    STMT                  \
    stop_time = millis(); \
    stop_time-start_time; \
  })

/*
 * Stolen from http://frenki.net/2013/10/fast-analogread-with-arduino-due/
 * I only embellished the process in a black box macro to ease the minds
 * of us mortals who don't quite understand what is actually deep below.
 *
 * This will ONLY WORK IN AN ARDUINO DUE! (or similar arch).
 */
#define FillBuffer(buf) do{\
  for(int i=0; i<N_SAMPLES; i++){ \
    while((ADC->ADC_ISR & 0x80)==0); /* wait for conversion */ \
    buf[i]=ADC->ADC_CDR[7]*3.3/4096; /* get values */ \
  } \
  } while(0)

GoertzelJS gjs = GoertzelJS(TARGET_FREQUENCIES, N_TARGETS, SAMPLE_FREQ, 0);

/*
 * All this is to be able to print received messages.
 */
#define MAX_RECV_LEN 10 // Max number of bytes received.
byte *received = (byte*)malloc(MAX_RECV_LEN); // Buffer to store received bits
unsigned int nextBit = 0; // Next bit to be received.
#define ReceiveBit(b) do {\
  received[nextBit/8] |= b << (7 - nextBit%8); \
  nextBit++; \
  } \
  while(0);
#define ClearReceived() do {\
  memset(received, '\0', MAX_RECV_LEN); \
  nextBit = 0; \
  } \
  while(0);

int lastReceived = 0; // Did I receive a bit the previous loop?

void setup() {
  Serial.begin(115200);
/*
 * ADC stuff, same as `FillBuffer`. Only in Due.
 * https://forum.arduino.cc/index.php?topic=487989.msg3683581#msg3683581
 */
  ADC->ADC_MR |= 0x80;  // set free running mode on ADC
  ADC->ADC_CHER = 0x80; // enable ADC on pin A0
  ClearReceived();
}

void loop() {

#if MeasureSampleFreq // Read `SAMPLE_FREQ` comment, at the top of the page.
  int dt = MeasureTime(
#endif
    FillBuffer(samples);
#if MeasureSampleFreq 
  );

  Serial.print("Measured time (us): ");
  Serial.println(dt);

#else // SAMPLE_FREQ is already set, run Goertzel.

  // At 128 samples, this takes ~32 ms
  // At 96 samples, ~26 ms
  // At 80 samples, ~22 ms
  int goertzelTime = MeasureMillis(
    for (int i=0; i<N_SAMPLES; i++) {
    	/* Frequency detection behaves better when passing the samples through
       * a filter (check `utilities/util.cpp`)
       */
      double filteredSample = blackman4(samples[i], (uint8_t) i, N_SAMPLES);
      gjs.processSample(filteredSample);
    }
  );
  
#if DEBUG

  //gjs.printDebug(Serial);
  gjs.printEnergies(Serial, TEN_P_FIVE);
  
#else // not debuggin', print symbols
  //gjs.printEnergies(Serial, TEN_P_FIVE);

  int idx; 
  if ((idx=signal_over_threshold(gjs.energies, N_TARGETS)) == -1) {
    if (lastReceived) {
      Serial.print("Received: \"");
      for (int i=0; i<MAX_RECV_LEN; i++) {
        Serial.print((char)received[i]);
      }
      Serial.println("\"");
      ClearReceived();
    }
    Serial.println("[No signal]");
    lastReceived = 0;
    goto epilogue;
  }

  if ((idx=two_over_threshold(gjs.energies, N_TARGETS)) == -1) {
    Serial.println("[Ambiguous]");
    lastReceived = 0;
  } else {
    Serial.println(idx);
    ReceiveBit(idx);
    lastReceived = 1;
  }

#endif //DEBUG
epilogue:
  gjs.refresh(); // <-- must always go!
  delay(symbolTime - goertzelTime);
  
#endif //MeasureSampleFreq
}

/*
 * Returns the index of the first energy in `energies` which is greater than
 * the THRESHOLD. Otherwise, returns -1.
 */
int signal_over_threshold(double *energies, unsigned int n_energies) {
  for (int i=0; i<n_energies; i++) {
    if (ScaleEnergy(energies[i]) > THRESHOLD) {
      return i;
    }
  }
  return -1;
}

/*
 * In case there's only one energy value greater than THRESHOLD, its index is
 * returned.
 * If there's two (or more) energy values greater than THRESHOLD, this function
 * compares ONLY THE FIRST TWO (sorry, M>>1-FSK lovers), and returns the index
 * of the greatest ONLY if its value is greater than the smaller by at least
 *  GAP_FACTOR*THRESHOLD
 * If the two values are too close, it will return -1.
 */
#define GAP_FACTOR 0.5 // <- Tune wisely
int two_over_threshold(double *energies, unsigned int n_energies) {
  int last_over_threshold = -1;
  for (int i=0; i<n_energies; i++) {
    if (ScaleEnergy(energies[i]) > THRESHOLD && last_over_threshold == -1) {
      last_over_threshold = i;
      continue;
    }
    if (ScaleEnergy(energies[i]) > THRESHOLD /* && last_over_threshold != -1 */) {
      if (ScaleEnergy(energies[i]) - ScaleEnergy(energies[last_over_threshold]) >= GAP_FACTOR*THRESHOLD) {
        return i;
      }
      if (ScaleEnergy(energies[last_over_threshold]) - ScaleEnergy(energies[i]) >= GAP_FACTOR*THRESHOLD) {
        return last_over_threshold;
      }
      return -1;
    }
  }
  return last_over_threshold;
}
