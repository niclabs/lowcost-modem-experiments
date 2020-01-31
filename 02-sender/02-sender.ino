
#define outputPin 10 // Connected to S1 on the 4053
#define enablePin 11 // Connected to ~E on the 4053
#define symbolTime 35 // ms

/*
 * Encoding is achieved by setting the SELECT pin on the 4053 mux.
 * A logic 1 selects the 40 KHz signal (encoding a 0), while
 * a logic 0 selects the 45 KHz signal (encoding a 1).
 */
#define ZERO HIGH
#define ONE LOW

/*
 * In order for the multiplexer to work, the ENABLE pin must be set low, otherwise
 * no signal will be output (at least that I know. read the datasheet if you must).
 */
#define Enable() digitalWrite(enablePin, LOW)
#define Disable() digitalWrite(enablePin, HIGH)


/*
 * If FromSerial is 0, the same message `msg` will be send all the time.
 * If FromSerial is 1, the message to send will be read from the Serial input.
 */
#define FromSerial 1

#if FromSerial
#define MAX_SEND_LEN 10 // bytes
byte *msg = (byte *)malloc(MAX_SEND_LEN);
unsigned int nextChar = 0;
bool msgReady = false;
#define ClearMsg() do {\
    memset(msg, '\0', MAX_SEND_LEN); \
    nextChar = 0; \
  } while(0)
#else // not FromSerial
byte msg[] = { 0x49, 0x74, 0x20, 0x77, 0x6f, 0x72, 0x6b, 0x73, 0x21}; //0b11100110, 0b10101100, 0b11010010 };
#define MSG_DELAY 500 // ms
#endif


void setup() {
  Serial.begin(115200);
  pinMode(outputPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  Enable();
}

void loop() {
#if FromSerial
  if (msgReady) {
    Serial.println("Sending message");
    Enable();
    sendBits(msg, nextChar*8);
    Disable();
    Serial.println("Done");
    ClearMsg();
    msgReady = false;
  }
#else // not FromSerial
  Serial.println("Sending message");
  sendBits(msg, 80);
  Disable();
  Serial.println("Done");
  
  delay(MSG_DELAY);
  Enable();
#endif //FromSerial
}

void sendBits(byte message[], unsigned int len) {
  int sent = 0;
  for (int i=0;; i++) {
    Serial.print("[byte ");Serial.print(i);Serial.print("]");
    for (int sh=7; sh >= 0; sh--) {
      char msgBit =  (message[i] >> sh) & 0b1;
      int toSend = msgBit ? ONE : ZERO;
      digitalWrite(outputPin, toSend);
      Serial.print(msgBit ? '1' : '0');
      sent++;
      delay(symbolTime);
      if (sent == len) { 
        Serial.println();
        return;
      }
    }
  }
}

// https://www.arduino.cc/en/Tutorial/SerialEvent
#if FromSerial
void serialEvent() {
  while (Serial.available()) {
    byte inByte = Serial.read();
    msg[nextChar++] = inByte;
    if (inByte == '\n' || nextChar == MAX_SEND_LEN) {
      msgReady = true;
    }
  }
}
#endif
