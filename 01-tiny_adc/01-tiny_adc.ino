#define BUFFER_SIZE 1000
uint16_t buf[BUFFER_SIZE];

#define FillBuffer(buf) do{\
  for(int i=0; i<BUFFER_SIZE; i++){ \
    while((ADC->ADC_ISR & 0x80)==0); /* wait for conversion */ \
    buf[i]=ADC->ADC_CDR[7]*3.3/4096; /* get values */ \
  } \
  } while(0)

unsigned long start_time, stop_time;
#define MeasureTime(STMT) \
  ({                      \
    start_time = micros();\
    STMT                  \
    stop_time = micros(); \
    stop_time-start_time; \
  })

void setup() {
  Serial.begin(115200);
  ADC->ADC_MR |= 0x80;  // set free running mode on ADC
  ADC->ADC_CHER = 0x80; // enable ADC on pin A0

}

void loop() {
  int dt = MeasureTime(
    FillBuffer(buf);
  );
  Serial.print("T Measured time (us): ");
  Serial.println(dt);
  delay(10);
}
