/*
 * Based on http://frenki.net/2013/10/fast-analogread-with-arduino-due
 *      and https://github.com/kosme/arduinoFFT/blob/master/Examples/FFT_03/FFT_03.ino
 * */

#define N_SAMPLES 512
unsigned int SAMPLE_FREQ;

/* All frequencies outside this range will be zero'd out (bandpass).
   Lower precision when N_SAMPLES is low (<256) */
#define LOWER_FREQ 800
#define HIGHER_FREQ 5200

#define VERBOSE 0

/*
 * Forked https://github.com/niclabs/arduinoFFT
 * in order to have MagnitudeMajorPeak() and Bandpass(lf, hf)
 */
#include "arduinoFFT.h"

double vReal[N_SAMPLES];
double vImag[N_SAMPLES];
//unsigned long values[N_SAMPLES];
unsigned int i;

#define ZeroBuffers(real, imag) do{\
  for(i=0; i<N_SAMPLES; i++){ \
    vReal[i] = vImag[i] = 0; \
  } \
  } while(0)

#define FillBuffers() do{\
  for(i=0; i<N_SAMPLES; i++){ \
    while((ADC->ADC_ISR & 0x80)==0); /* wait for conversion */ \
    vReal[i]=ADC->ADC_CDR[7]*3.3/4096; /* get values */ \
  /*values[i]=ADC->ADC_CDR[7]; */ \
  } \
  } while(0)

unsigned long start_time, stop_time;
unsigned int delta_t_us, dt_us_fft;
#define MeasureTime(STMT) \
  ({                      \
    start_time = micros();\
    STMT                  \
    stop_time = micros(); \
    stop_time-start_time; \
  })

#define SampleFreqFromDt(delta_t) \
  ({                      \
    round(pow(10, 6)/delta_t)*N_SAMPLES; \
  })
  
void setup() {        
	Serial.begin(115200);  
	ADC->ADC_MR |= 0x80;  //set free running mode on ADC
	ADC->ADC_CHER = 0x80; //enable ADC on pin A0
}

void loop() {
  ZeroBuffers(vReal, vImag);
  delta_t_us = MeasureTime(
    FillBuffers();
  );

  SAMPLE_FREQ = SampleFreqFromDt(delta_t_us);

#if VERBOSE
  Serial.print("Acquisition time (us): ");
  Serial.println(delta_t_us);
  Serial.print("Sample Freq (Hz): ");
  Serial.println(SAMPLE_FREQ);
#endif //VERBOSE

  // FFT stuff
  arduinoFFT FFT = arduinoFFT(vReal, vImag, N_SAMPLES, SAMPLE_FREQ);
  dt_us_fft = MeasureTime(
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
    FFT.Compute(FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(); /* Compute magnitudes */
  );

#if VERBOSE
  Serial.print("FFT time (us): ");
  Serial.println(dt_us_fft);
#endif //VERBOSE
  //FFT.Bandpass(LOWER_FREQ, HIGHER_FREQ);

  double x = FFT.MajorPeak();
  double f = FFT.MagnitudeMajorPeak();
  Serial.print(x, 6);
#if VERBOSE
  Serial.print(" Hz (peak), Magnitude: ");
#else
  Serial.print('\t');
#endif //VERBOSE
  Serial.println(f, 6);
  
  delay(125);
#if VERBOSE
  Serial.println("--------------------------------");
#endif //VERBOSE
}
