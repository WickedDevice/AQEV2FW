#include <Wire.h>
#include <SPI.h>
#include <WildFire.h>
#include <WildFire_CC3000.h>
#include <WildFire_PubSubClient.h>
#include <TinyWatchdog.h>
#include <SHT25.h>
#include <MCP342x.h>
#include <LMP91000.h>
#include <WildFire_SPIFlash.h>
#include <Time.h>
#include <CapacitiveSensor.h>
#include <LiquidCrystal.h>
#include <util/crc16.h>

#define AQEV2FW_VERSION "0.1"

WildFire wf;
WildFire_CC3000 cc3000;
TinyWatchdog tinywdt;
LMP91000 lmp91000;
MCP342x mcp342x;
SHT25 sht25;
WildFire_SPIFlash flash;
CapacitiveSensor touch = CapacitiveSensor(A1, A0);
LiquidCrystal lcd(A3, A2, 4, 5, 6, 8);

// the software's operating mode
#define MODE_CONFIG      (1)
#define MODE_OPERATIONAL (2)
uint8_t mode = MODE_OPERATIONAL; 

// the config mode state machine's return values
#define CONFIG_MODE_NOTHING  (0)
#define CONFIG_MODE_GOT_INIT (1)
#define CONFIG_MODE_GOT_EXIT (2)

// tiny watchdog timer intervals
unsigned long previous_tinywdt_millis = 0;
const long tinywdt_interval = 1000;

void setup(){
  // initialize hardware
  initializeHardware();
  
  // check for initial integrity of configuration in eeprom
  if(!checkConfigIntegrity()){
    mode = MODE_CONFIG;
  }  
  else{
    // if the appropriate escape sequence is received within 5 seconds
    // go into config mode
    const long startup_time_period = 9000;
    long start = millis();
    long min_over = 100;
    boolean got_serial_input = false;
    while(millis() < start + startup_time_period){ // can get away with this sort of thing at start up
       if(Serial.available()){
         if(got_serial_input == false){
           Serial.println();                      
         }
         got_serial_input = true;         

         start = millis(); // reset the timeout
         if(CONFIG_MODE_GOT_INIT == configModeStateMachine(Serial.read())){
           mode = MODE_CONFIG;
           break;
         }
       }
       
       // output a countdown to the Serial Monitor
       if(millis() - start >= min_over){
         if(got_serial_input == false){
           Serial.print((startup_time_period - 500 - min_over) / 1000);
           Serial.print(F("..."));
         }
         min_over += 1000;
       }
    }    
  }
  Serial.println();
  
  if(mode == MODE_CONFIG){
    Serial.println(F("-~=* In CONFIG Mode *=~-"));
    for(;;){
      // stuck in this loop until the command line receives an exit command
      if(Serial.available()){
        // if you get serial traffic, pass it along to the configModeStateMachine for consumption
        if(CONFIG_MODE_GOT_EXIT == configModeStateMachine(Serial.read())){
          break;
        }
      }        
    }
  }
  else{
    Serial.println(F("-~=* In OPERATIONAL Mode *=~-"));
    // Try and Connect to the Network
    
    // If connected, Check for Firmware Updates
    
    // If connected, Get Network Time
    
  }
  
  // re-check for valid configuration
  if(!checkConfigIntegrity()){
    Serial.println(F("Resetting prior to Loop because of invalid configuration"));
    tinywdt.force_reset();     
  }
  else{
    Serial.println(F("Beginning main Loop")); 
  }
  
}

void loop(){
  unsigned long current_millis = millis();
  
  
  // pet the watchdog
  if(current_millis - previous_tinywdt_millis >= tinywdt_interval) {
    tinywdt.pet();
    previous_tinywdt_millis = current_millis;
  }
}

void initializeHardware(void){
  wf.begin();
  Serial.begin(115200);  
  
  Serial.println(F(" +------------------------------------+"));
  Serial.println(F(" |   Welcome to Air Quality Egg 2.0   |"));
  Serial.println(F(" |        Firmware Version " AQEV2FW_VERSION "        |"));
  Serial.println(F(" +------------------------------------+"));  
  Serial.println();  
  
  Wire.begin();
  
  // Initialize slot select pins
  Serial.print(F("Slot Select Pins Initialization..."));
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);       
  selectNoSlot();
  Serial.println(F("OK."));
  
  // Initialize Tiny Watchdog
  Serial.print(F("Tiny Watchdog Initialization..."));
  tinywdt.begin(500, 60000);
  Serial.println(F("OK."));
 
  // Initialize the LCD
  byte upArrow[8] = {
          B00100,
          B01110,
          B11111,
          B00100,
          B00100,
          B00100,
          B00100,
          B00100
  };  
  
  byte downArrow[8] = {
          B00100,
          B00100,
          B00100,
          B00100,
          B00100,
          B11111,
          B01110,
          B00100
  };   
  
  pinMode(A6, OUTPUT);
  backlightOn();
  
  lcd.createChar(0, upArrow);  
  lcd.createChar(1, downArrow);   
  lcd.begin(16, 2);  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Air Quality Egg"));
  lcd.setCursor(3, 1);
  lcd.print(F("Version 2"));
   
  // Initialize SPI Flash
  Serial.print(F("SPI Flash Initialization..."));
  if(flash.initialize()){
    Serial.println(F("OK."));
  }
  else{
    Serial.println(F("Fail."));
  }
  
  // Initialize SHT25
  Serial.print(F("SHT25 Initization..."));
  if(sht25.begin()){
    Serial.println(F("OK."));   
  }
  else{
    Serial.println(F("Failed.")); 
  }
  
  // Initialize NO2 Sensor
  Serial.print(F("NO2 Sensor Initization..."));
  selectSlot2();
  if(lmp91000.configure( 
      LMP91000_TIA_GAIN_350K | LMP91000_RLOAD_10OHM,
      LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_67PCT 
            | LMP91000_BIAS_SIGN_NEG | LMP91000_BIAS_8PCT,
      LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC)){
    Serial.println(F("OK.")); 
  }
  else{
    Serial.println(F("Failed.")); 
  }
  
  Serial.print(F("CO Sensor Initization..."));
  selectSlot1();
  if(lmp91000.configure( 
      LMP91000_TIA_GAIN_350K | LMP91000_RLOAD_10OHM,
      LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_20PCT 
            | LMP91000_BIAS_SIGN_POS | LMP91000_BIAS_1PCT,
      LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC)){  
    Serial.println(F("OK.")); 
  }
  else{
    Serial.println(F("Failed.")); 
  }    
  
  selectNoSlot(); 
}

boolean checkConfigIntegrity(void){
  
  return true;
}

// this state machine receives bytes and 
// returns true if the function is in config mode
uint8_t configModeStateMachine(char b){
  static boolean received_init_code = false;
  const char buf_max_write_idx = 62; // [63] must always have a null-terminator
  static char buf[64] = {0}; // buffer to hold commands / data
  static uint8_t buf_idx = 0;  // current number of bytes in buf  
  boolean line_terminated = false;
  uint8_t ret = CONFIG_MODE_NOTHING;

  //  Serial.print('[');
  //  if(isprint(b)) Serial.print((char) b);
  //  Serial.print(']');
  //  Serial.print('\t');
  //  Serial.print("0x");
  //  if(b < 0x10) Serial.print('0');
  //  Serial.println(b, HEX);     
  
  // the following logic rejects all non-printable characters besides 0D, 0A, and 7F
  if(b == 0x7F){ // backspace key is special
    if(buf_idx > 0){
      buf_idx--;
      buf[buf_idx] = '\0'; 
      Serial.print(b); // echo the character      
    }
  }
  else if(b == 0x0D || b == 0x0A){ // carriage return or new line is also special
    if(strlen(buf) > 0){
      buf[buf_idx++] = 0x0D; // line terminator
      line_terminated = true;      
    }
    Serial.println(); // echo the character
    prompt();    
  }
  else if((buf_idx <= buf_max_write_idx) && isprint(b)){
    // otherwise if there's space and the character is 'printable' add it to the buffer
    // silently drop all other non-printable characters
    buf[buf_idx++] = b;
    Serial.print(b); // echo the character
  }
  
  if(received_init_code){
    
  }
  else if(line_terminated){
    // we are looking for an exact match to the string
    // "AQECFG\r"   
    
    if((strncmp("aqe\r", buf, 4) == 0) || (strncmp("AQE\r", buf, 4) == 0)){
      received_init_code = true;
      ret = CONFIG_MODE_GOT_INIT;
    }
    else{
      buf[buf_idx-1] = '\0'; // buf_idx is certainly >= 1, and the last character buffered is a newline
      Serial.print(F("Error: Expecting Config Mode Unlock Code (\"aqe\"), but received \""));
      Serial.print(buf);
      Serial.println(F("\""));
      buf[0] = '\0';
      buf_idx = 0;
    }
  }
  
  return ret;
}

void prompt(void){
  Serial.print("AQE>:"); 
}

// Gas Sensor Slot Selection
void selectNoSlot(void){
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);  
}

void selectSlot1(void){
  selectNoSlot();
  digitalWrite(10, HIGH);    
}

void selectSlot2(void){
  selectNoSlot();
  digitalWrite(9, HIGH);        
}

void backlightOn(void){
  digitalWrite(A6, HIGH);
}

void backlightOff(void){
  digitalWrite(A6, LOW);  
}
