/***************************************************
  This is the logic of an RFID-triggered compound machine
  with some inputs and actuators.

  Inputs:
    - RFID reader
    - button

  Actuators:
    - LED strip
    - servo motor to open/close a door

  Logic:
    - wait until a card is placed on the RFID reader
      - when card is detected
        - blink led strip
        - open door
        - wait until button is pressed
          - if button is pressed
            - close door
            - reset state
 ****************************************************/

// #define CALIBRATE_SERVO
// #define TEST_RFID

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <MFRC522.h>
#include <SPI.h>


// RFID reader
#define MFRC522_CS_PIN            D8
MFRC522 mfrc522;


/* Servo parameters */
#define SERVO_FREQ                50 // Analog servos run at ~50 Hz updates
Adafruit_PWMServoDriver PCA = Adafruit_PWMServoDriver();


/* button paramters */
#define BTN_PIN                   D0


/* LED parameters */
#define LED_N                     9
const uint8_t LED_PINS[LED_N] =   {4, 5, 6, 7, 8, 9, 10, 11, 12}; // in the PCA9685 array
#define LED_BLINK_TIMES           3
#define LED_BLINK_DELAY           50 // short == fast blinking (ms)


/* Door parameters */
#define DOOR_SERVO_PIN            0
#define DOOR_SERVO_CLOSED_POS     80
#define DOOR_SERVO_OPEN_POS       500
#define SERVOMIN                  80  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX                  510 // This is the 'maximum' pulse length count (out of 4096)

/* display */
#define SCREEN_WIDTH              128 // OLED display width, in pixels
#define SCREEN_HEIGHT             64 // OLED display height, in pixels
#define SCREEN_ADDRESS            0x3C ///< See datasheet
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO_WIDTH 66
#define LOGO_HEIGHT 60

const unsigned char bitmap [] PROGMEM = {
	// 'heart, 66x60px
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x0f, 0xf8, 
	0x00, 0x00, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x7f, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x01, 
	0xff, 0xff, 0xc0, 0x00, 0x01, 0xff, 0xff, 0xf0, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x03, 0xff, 0xff, 
	0xf8, 0x07, 0xff, 0xff, 0xf0, 0x00, 0x07, 0xff, 0xff, 0xfc, 0x0f, 0xff, 0xff, 0xf8, 0x00, 0x0f, 
	0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x1f, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xfe, 
	0x00, 0x1f, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 0xff, 0x3f, 0xff, 
	0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x7f, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x7f, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x80, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x80, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x3f, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1f, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 
	0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xfc, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xf0, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 
	0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 
	0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 
	0xf8, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 
	0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 
	0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xe0, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x1f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/* counters */
uint32_t love = 0;
uint32_t cards = 0;

/* State variables */
void cardSetup() {
  // setup RFID reader
  mfrc522.PCD_Init(MFRC522_CS_PIN);
  mfrc522.PCD_DumpVersionToSerial();

  Serial.println("Card reader setup");
}

void doorSetup() {
  doorClose();

  Serial.println("Door setup");
}

void buttonSetup() {
  pinMode(BTN_PIN, INPUT);

  Serial.println("Button setup");
}

void ledOff() {
  for (uint8_t i = 0; i < LED_N; i++) {
    PCA.setPin(LED_PINS[i], 0, false); // turns pin fully off
  }
}

void ledSetup() {
  ledOff();
  Serial.println("LED strip setup");
}

void doorClose() {
  Serial.println("Door action: CLOSE!");

  PCA.setPWM(DOOR_SERVO_PIN, 0, DOOR_SERVO_CLOSED_POS);
}

void doorOpen() {
  Serial.println("Door action: OPEN!");

  PCA.setPWM(DOOR_SERVO_PIN, 0, DOOR_SERVO_OPEN_POS);
}

void displaySetup() {
// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SCREEN_ADDRESS, true)) {
    Serial.println(F("SH1106 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.display();
  delay(1000);
  display.clearDisplay();

  displayWelcome();
}

void i2c_scanner()
{
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}

void displayWelcome(void) {
  display.clearDisplay();

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);

  display.println("VALENTINE'S DAY 2024");
  display.println("");

  display.println("When given love...");
  display.println("...I give cards!");

  display.println("");

  display.println("Give it a try!");

  display.display();

  display.clearDisplay();
}

void displayHeart() {
  display.clearDisplay();
  display.drawBitmap(0, 0, bitmap, LOGO_WIDTH, LOGO_HEIGHT, SH110X_WHITE);
  display.display();
}


void displayMessage(String msg) {

  display.clearDisplay();

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);

  display.println("");
  display.println("");
  display.println(msg);

  display.display();

  display.clearDisplay();
}


void updateDisplay(bool cardOut, bool next) {

  display.clearDisplay();

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);

  display.println("VALENTINE'S DAY 2024");
  display.println("");

  display.print("Love in: ");
  display.println(love);

  if (cardOut) {
    display.print("Cards out: ");
    display.println(cards);
  } else {
    display.println("");
    display.println("Pick a card, then");
    display.println("push the button!");
  }

  if (next) {
    display.println("");
    display.println("Keep giving!");
  }

  display.display();

  display.clearDisplay();
}

void ledBlink() {
  for (uint8_t t = 0; t < LED_BLINK_TIMES; t++)
    for (uint8_t i = 0; i < LED_N; i++) {
      PCA.setPin(LED_PINS[i], 4095, false); // turns pin fully on
      delay(LED_BLINK_DELAY);
      PCA.setPin(LED_PINS[i], 0, false); // turns pin fully off
    }
}

void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

void printCardInfo() {
  Serial.println("");
  Serial.print(F("Card PICC Type:"));
  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  Serial.println(mfrc522.PICC_GetTypeName(piccType));

  Serial.print(F("Card UID:"));
  dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
  Serial.println();
}

/**
 * Credit: https://github.com/Martin-Laclaustra/MFRC522-examples/
 *
 * Returns true if a PICC responds to PICC_CMD_WUPA.
 * All cards in state IDLE or HALT are invited.
 * 
 * @return bool
 */
bool PICC_IsAnyCardPresent() {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  
  // Reset baud rates
  mfrc522.PCD_WriteRegister(mfrc522.TxModeReg, 0x00);
  mfrc522.PCD_WriteRegister(mfrc522.RxModeReg, 0x00);

  // Reset ModWidthReg
  mfrc522.PCD_WriteRegister(mfrc522.ModWidthReg, 0x26);
  
  MFRC522::StatusCode result = mfrc522.PICC_WakeupA(bufferATQA, &bufferSize);
  return (result == MFRC522::STATUS_OK || result == MFRC522::STATUS_COLLISION);
} // End PICC_IsAnyCardPresent()

void setup() {
  Serial.begin(9600);

  Wire.begin();
  SPI.begin();
  PCA.begin();
  PCA.setOscillatorFrequency(27000000);
  PCA.setPWMFreq(SERVO_FREQ);

  ledSetup();
  cardSetup();
  buttonSetup();

#ifdef CALIBRATE_SERVO
  Serial.println("Servo calibration mode. Enter a value in [0, 4095] (will not be echoed).");
#else
  doorSetup();

  delay(10);

  Serial.println("Setup complete");

  // Scan for i2c devices
  // while (true) {
  //   i2c_scanner();
  // }

  displaySetup();
#endif
}

void loop() {
  /* This could have been done much more elegantly using an event-based loop,
   * but time is scarce. */

#ifdef CALIBRATE_SERVO
  long v = -1;

  if (Serial.available())
    v = Serial.parseInt();

  if (v >= 0 && v < 4096) {
    Serial.print("setPWM(0, 0, ");
    Serial.print(v);
    Serial.println(")");
    PCA.setPWM(DOOR_SERVO_PIN, 0, v);
  }
#else
#ifdef TEST_RFID
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
	if ( ! mfrc522.PICC_IsNewCardPresent()) {
		return;
	}

	// Select one of the cards
	if ( ! mfrc522.PICC_ReadCardSerial()) {
		return;
	}

	// Dump debug info about the card; PICC_HaltA() is automatically called
	mfrc522.PICC_DumpToSerial(&(mfrc522.uid)); TEST_RFID
#else
  buttonSetup();

  /* Not very reliable */
  // if (PICC_IsAnyCardPresent()) {
  //   Serial.println("Please remove card from the reader");
  //   displayMessage("Please remove any card!");
  // }
  //
  // while (PICC_IsAnyCardPresent());

  displayWelcome();

  // wait for card
  Serial.println("Waiting for card...");
  while (!(mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()));

  // card detected
  printCardInfo();

  delay(200);

  // blink LEDs
  ledBlink();

  displayHeart();

  delay(2000);

  love++;
  updateDisplay(false, false);

  delay(200);

  // open door
  doorOpen();

  /* Not very reliable */
  // if (PICC_IsAnyCardPresent()) {
  //   Serial.println("Please remove card from the reader");
  //   displayMessage("Remove the white card");
  // }
  //
  // while (PICC_IsAnyCardPresent());
  //
  // // wait for button pressure to indicate that the door must be closed

  Serial.println("Waiting for button...");

  while (digitalRead(BTN_PIN) != LOW) {
    delay(250);
  }

  delay(200);

  // close door
  doorClose();
  cards++;
  updateDisplay(true, true);

  Serial.println("...new round!");

  delay(2000);
#endif // TEST_RFID
#endif // CALIBRATE_SERVO
}

