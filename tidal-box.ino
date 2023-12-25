// originally sourced from
// https://github.com/diyelectromusic/sdemp/blob/main/src/SDEMP/SimpleMIDISerialMonitor/SimpleMIDISerialMonitor.ino

#include <MIDI.h>


#define MIDI_HW_SERIAL2  1


// ---- Definitions for MIDI INPUT devices ----
//
#ifdef MIDI_HW_SERIAL2
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI_HS2);
#endif

#define MIDI_LED LED_BUILTIN

void ledOn() {
#ifdef MIDI_LED
  digitalWrite(MIDI_LED, HIGH);
#endif
}

void ledOff() {
#ifdef MIDI_LED
  digitalWrite(MIDI_LED, LOW);
#endif
}

void ledInit() {
#ifdef MIDI_LED
  pinMode (MIDI_LED, OUTPUT);
#endif
  ledOff();
}

unsigned long baseMsPerClock = 30000;
unsigned long msPerClock = 30000;
int variation = 0;

const int potentiometerPin = A0;
int previousPotValue = 0;

const int potentiometerPin2 = A1;
int previousPotValue2 = 0;

#define MIDI_CLOCK 0xF8

#define MIDI_CLOCK_PIN 1

unsigned long previousMicros = 0;

void setup() {
    pinMode(MIDI_CLOCK_PIN, OUTPUT);

  // Using the standard serial link for the "monitor"
  Serial.begin(9600);
  Serial.println("MIDI Monitor Starting on ");
  ledInit();

  // Initialise all INPUT devices to listen on all channels.
  // Disable the THRU for all devices.
  Serial.print("HW-SERIAL2 ");
  MIDI_HS2.begin(MIDI_CHANNEL_OMNI);
  MIDI_HS2.turnThruOff();
  // MIDI_HS2.setHandleSystemExclusive(printMidiSysEx);
  Serial.print("\n");

}

void loop() {


  //Serial.println("MIDI_HW_SERIAL2");
  if (MIDI_HS2.read()) {
    midi::MidiType mt  = MIDI_HS2.getType();
    if (mt != midi::SystemExclusive) {
      printMidiMsg(mt, MIDI_HS2.getData1(), MIDI_HS2.getData2(), MIDI_HS2.getChannel());
    }
  }



  // Get the current time in microseconds
  unsigned long currentMicros = micros();


  // Check if the specified interval has passed
  if (currentMicros - previousMicros >= msPerClock) {

    checkA0();
    checkA1();

    // Send MIDI clock message at the specified interval

    sendMidiClock();
    previousMicros = currentMicros;
  }


}

void sendMidiClock() {
    // Send the MIDI clock message on the default Serial TX pin (pin 1)
    // Serial1.write(MIDI_CLOCK);
    // MIDI.sendRealTime(0xF8);
    // int x = Serial1.read();
    // Serial.println(x);


    MIDI_HS2.sendClock();
}

void checkA0() {
    int potValue = analogRead(potentiometerPin);
    // Serial.println(potValue);
    if (potValue != previousPotValue) {
      // Serial.println(potValue);
      changeTempo(potValue);
      previousPotValue = potValue;
    }
}

void checkA1() {
    int potValue = analogRead(potentiometerPin2);
    // Serial.println(potValue);
    if (potValue != previousPotValue2) {
      // Serial.println(potValue);
      changeVar(potValue);
      previousPotValue2 = potValue;
    }
}

void changeTempo(int val) {

      // Update the previous potentiometer value
      baseMsPerClock = scaleValue(val, 0, 1023, 45000, 6000);
      msPerClock = baseMsPerClock;
      // Serial.println(val);
}

void randomizeTempo() {
  msPerClock = baseMsPerClock + random(variation * (-30), variation * 30);
  Serial.println(variation);
  Serial.println(msPerClock);
}


void changeVar(int val) {
  variation = val;
}


void printMidiMsg (uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t ch) {

  if (cmd != midi::NoteOn){
    return;
  }

  switch (cmd) {
    case midi::ActiveSensing:
      // Ignore
      return;
      break;
  }

  if (ch == 5){
    // Serial.println("YESSSSSSSSSSS");
    randomizeTempo();
  }

  if (ch<10) Serial.print(" ");
  Serial.print(ch);
  Serial.print(" 0x");
  Serial.print(cmd, HEX);
  Serial.print(" 0x");
  Serial.print(d1, HEX);
  Serial.print(" 0x");
  Serial.print(d2, HEX);
  Serial.print("\t");
  switch(cmd) {
    case midi::NoteOff:
      Serial.print("NoteOff\t");
      Serial.print(d1);
      Serial.print("\t");
      Serial.print(d2);
      break;
    case midi::NoteOn:
      Serial.print("NoteOn\t");
      Serial.print(d1);
      Serial.print("\t");
      Serial.print(d2);
      break;
    case midi::AfterTouchPoly:
      Serial.print("AfterTouchPoly\t");
      Serial.print(d1);
      Serial.print("\t");
      Serial.print(d2);
      break;
    case midi::ControlChange:
      Serial.print("ControlChange\t");
      Serial.print(d1);
      Serial.print("\t");
      Serial.print(d2);
      break;
    case midi::ProgramChange:
      Serial.print("ProgramChange\t");
      Serial.print(d1);
      break;
    case midi::AfterTouchChannel:
      Serial.print("AfterTouchChannel\t");
      Serial.print(d1);
      break;
    case midi::PitchBend:
      Serial.print("PitchBend\t");
      Serial.print(d1);
      Serial.print("\t");
      Serial.print(d2);
      break;
  }
  Serial.print("\n");
}

void printMidiSysEx (byte *inArray, unsigned inSize) {
  Serial.print("SysEx Array size = ");
  Serial.println(inSize);
  int idx=0;
  for (int i=0; i<inSize; i++) {
   if (inArray[i] < 16) {
     Serial.print("0");
   }
   Serial.print(inArray[i], HEX);
   idx++;
   if (idx >= 16) {
     Serial.print("\n");
     idx = 0;
   }
  }
  if (idx != 0) {
    Serial.print("\n");
  }
}


unsigned long scaleValue(unsigned long inputValue, unsigned long fromLow, unsigned long fromHigh, unsigned long toLow, long unsigned toHigh) {
  // Ensure the input value is within the input range
  inputValue = constrain(inputValue, fromLow, fromHigh);

  // Serial.println("constrainted:");
  // Serial.println(inputValue);

  // Map the input value from the input range to the output range
  long scaledValue = map(inputValue, fromLow, fromHigh, toLow, toHigh);

  return scaledValue;
}
