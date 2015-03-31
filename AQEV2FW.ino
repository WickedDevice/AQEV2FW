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
#include <util/crc16.h>

#define AQEV2FW_VERSION "0.1"

WildFire wf;
WildFire_CC3000 cc3000;
TinyWatchdog tinywdt;
LMP91000 lmp91000;
MCP342x mcp342x;
SHT25 sht25;
WildFire_SPIFlash flash;

// the software's operating mode
#define MODE_CONFIG      (1)
#define MODE_OPERATIONAL (2)
uint8_t mode = MODE_OPERATIONAL; 

// tiny watchdog timer intervals
unsigned long previous_tinywdt_millis = 0;
const long tinywdt_interval = 1000;

void setup(){
  // initialize hardware
  initializeHardware();
  
  Serial.println(F(" +------------------------------------+"));
  Serial.println(F(" |   Welcome to Air Quality Egg 2.0   |"));
  Serial.println(F(" |        Firmware Version " AQEV2FW_VERSION "        |"));
  Serial.println(F(" +------------------------------------+"));  
  Serial.println();
  
  // check for initial integrity of configuration in eeprom
  if(!checkConfigIntegrity()){
    mode = MODE_CONFIG;
  }  
  else{
    // if the appropriate escape sequence is received within 5 seconds
    // go into config mode
    long start = millis();
    long min_over = 100;
    while(millis() < start + 6000){ // can get away with this sort of thing at start up
       if(Serial.available()){
         if(configModeStateMachine(Serial.read())){
           mode = MODE_CONFIG;
           break;
         }
       }
       
       // output a countdown to the Serial Monitor
       if(millis() - start >= min_over){
         Serial.print((5500 - min_over) / 1000);
         Serial.print(F("..."));
         min_over += 1000;
       }
    }    
  }
  Serial.println();
  
  if(mode == MODE_CONFIG){
    Serial.println(F("-~= In CONFIG Mode =~-"));
  }
  else{
    Serial.println(F("-~= In OPERATIONAL Mode =~-"));
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
  sht25.begin();
  Serial.println(F("OK."));   
  
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
boolean configModeStateMachine(char b){
  static boolean received_init_code = false;
  const char buf_max_write_idx = 62; // [63] must always have a null-terminator
  static char buf[64] = {0}; // buffer to hold commands / data
  static uint8_t buf_idx = 0;  // current number of bytes in buf  
  boolean line_terminated = false;

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
    }
  }
  else if(b == 0x0D || b == 0x0A){ // carriage return or new line is also special
    buf[buf_idx++] = 0x0D; // line terminator
    line_terminated = true;
  }
  else if((buf_idx <= buf_max_write_idx) && isprint(b)){
    buf[buf_idx++] = b;
  }
  
  if(received_init_code){
    
  }
  else if(line_terminated){
    // we are looking for an exact match to the string
    // "AQECFG\r"
    if(strncmp("AQECFG\r", buf, 7) == 0){
      received_init_code = true;
    }
    else{
      buf[buf_idx-1] = '\0'; // buf_idx is certainly >= 1, and the last character buffered is a newline
      Serial.print(F("Error: Expecting Config Mode Unlock Code (\"AQECFG\"), but received \""));
      Serial.print(buf);
      Serial.println(F("\""));
      buf[0] = '\0';
      buf_idx = 0;
    }
  }
  
  return received_init_code;
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
