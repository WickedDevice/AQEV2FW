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
#define CONFIG_MODE_NOTHING_SPECIAL  (0)
#define CONFIG_MODE_GOT_INIT         (1)
#define CONFIG_MODE_GOT_EXIT         (2)

#define EEPROM_MAC_ADDRESS    (E2END + 1 - 6)    // i.e. the last 6-bytes of EEPROM
                                                 // more parameters here, address relative to each other so they don't overlap                                                 
#define EEPROM_CRC_CHECKSUM   (E2END + 1 - 1024) // reserve the last 1kB for config



void help_menu(char * arg);
void print_eeprom_value(char * arg);
void initialize_eeprom_value(char * arg);
void restore(char * arg);
void set_mac_address(char * arg);

char * commands[] = {
  "help   ",
  "get    ",
  "init   ",
  "restore",  
  "setmac ",  
  0
};

void (*command_functions[])(char * arg) = {
  print_eeprom_value,
  initialize_eeprom_value,
  restore,
  set_mac_address,
  0
};

// tiny watchdog timer intervals
unsigned long previous_tinywdt_millis = 0;
const long tinywdt_interval = 1000;

void setup(){
  // initialize hardware
  initializeHardware();
  
  // check for initial integrity of configuration in eeprom
  if(!checkConfigIntegrity()){
    Serial.println(F("Info: Config memory integrity check failed, automatically falling back to CONFIG mode."));
    configModeStateMachine('a');
    configModeStateMachine('q');
    configModeStateMachine('e');
    configModeStateMachine('\r');
    mode = MODE_CONFIG;
  }  
  else{
    // if the appropriate escape sequence is received within 8 seconds
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
    const uint16_t idle_timeout_period_ms = 1000*60*2; // 2 minutes
    uint16_t idle_time_ms = 0;
    Serial.println(F("-~=* In CONFIG Mode *=~-"));
    prompt();
    for(;;){
      unsigned long current_millis = millis();
            
      // stuck in this loop until the command line receives an exit command
      if(Serial.available()){
        idle_time_ms = 0;
        // if you get serial traffic, pass it along to the configModeStateMachine for consumption
        if(CONFIG_MODE_GOT_EXIT == configModeStateMachine(Serial.read())){
          break;
        }
      }

      // pet the watchdog once a ssecond
      if(current_millis - previous_tinywdt_millis >= tinywdt_interval) {
        idle_time_ms += tinywdt_interval;
        tinywdt.pet();
        previous_tinywdt_millis = current_millis;
      }            
      
      if(idle_time_ms >= idle_timeout_period_ms){
        Serial.println(F("Idle time expired, exiting CONFIG mode."));
        break;
      }
    }
  }

  
  Serial.println(F("-~=* In OPERATIONAL Mode *=~-"));
  // Try and Connect to the Network
  
  // If connected, Check for Firmware Updates
  
  // If connected, Get Network Time
  
  
  
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
  
  Serial.print(F("CC3000 Initialization..."));
  if (cc3000.begin()){
    Serial.println(F("OK."));
  }
  else{
    Serial.println(F("Failed."));
  }  
}

boolean checkConfigIntegrity(void){
  uint16_t computed_crc = computeConfigChecksum();
  uint16_t stored_crc = eeprom_read_word((const uint16_t *) EEPROM_CRC_CHECKSUM);    
  if(computed_crc == stored_crc){
    return true;
  }
  else{
    Serial.print(F("Computed CRC = "));
    Serial.print(computed_crc, HEX);
    Serial.print(F(", Stored CRC = "));
    Serial.println(stored_crc, HEX);
    
    return false; 
  }
}

// this state machine receives bytes and 
// returns true if the function is in config mode
uint8_t configModeStateMachine(char b){
  static boolean received_init_code = false;
  const char buf_max_write_idx = 62; // [63] must always have a null-terminator
  static char buf[64] = {0}; // buffer to hold commands / data
  static uint8_t buf_idx = 0;  // current number of bytes in buf  
  boolean line_terminated = false;
  char * first_arg = 0;
  uint8_t ret = CONFIG_MODE_NOTHING_SPECIAL;
  
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
    buf[buf_idx] = '\0'; // force terminator do not advance write pointer    
    line_terminated = true;     
    Serial.println(); // echo the character
  }
  else if((buf_idx <= buf_max_write_idx) && isprint(b)){
    // otherwise if there's space and the character is 'printable' add it to the buffer
    // silently drop all other non-printable characters
    buf[buf_idx++] = b;
    buf[buf_idx] = '\0';
    Serial.print(b); // echo the character
  }
  
  
  // process the data currently stored in the buffer
  if(received_init_code && line_terminated){
    // with the exeption of the command "exit"
    // commands are always of the form <command> <argument>
    // they are minimally parsed here and delegated to 
    // callback functions that take the argument as a string   

    // Serial.print("buf = ");
    // Serial.println(buf);
       
    if((strncmp("exit", buf, 4) == 0) || (strncmp("EXIT", buf, 4) == 0)){      
      ret = CONFIG_MODE_GOT_EXIT;
    }    
    else{
      // the string must have one, and only one, space in it
      uint8_t num_spaces = 0;
      char * p;      
      for(p = buf; *p != '\0'; p++){ // all lines are terminated by '\r' above
        if(*p == ' '){
          num_spaces++;
        }
        
        if((num_spaces == 1) && (*p == ' ')){
          // if this is the first space encountered, null the original string here
          // in order to mark the first argument string
          *p = '\0';
        }
        else if((num_spaces > 0) && (first_arg == 0) && (*p != ' ')){
          // if we are beyond the first space, 
          // and have not encountered the beginning of the first argument
          // and this character is not a space, it is by definition
          // the beginning of the first argument, so mark it as such
          first_arg = p;
        }
      }
      
      // deal with commands that can legitimately have no arguments first
      if((strncmp("help", buf, 4) == 0) || (strncmp("HELP", buf, 4) == 0)){  
        help_menu(first_arg);
      }      
      else if(first_arg != 0){ 
        //Serial.print(F("Received Command: \""));
        //Serial.print(buf);
        //Serial.print(F("\" with Argument: \""));
        //Serial.print(first_arg);
        //Serial.print(F("\""));
        //Serial.println();
        
        // command with argument was received, determine if it's valid
        // and if so, call the appropriate command processing function
        for(uint8_t ii = 0; commands[ii] != 0; ii++){
          if(strncmp(commands[ii], buf, strlen(buf)) == 0){
            command_functions[ii](first_arg);
            break; 
          }
        }
        
      }
      else if(strlen(buf) > 0){
        Serial.print(F("Error: Argument expected for command \""));
        Serial.print(buf);
        Serial.println(F("\", but none was received"));
      }      
    }    
  }
  else if(line_terminated){ 
    // before we receive the init code, the only things
    // we are looking for are an exact match to the strings
    // "AQE\r" or "aqe\r"
    
    if((strncmp("aqe", buf, 3) == 0) || (strncmp("AQE", buf, 3) == 0)){
      received_init_code = true;
      ret = CONFIG_MODE_GOT_INIT;
    }
    else if(strlen(buf) > 0){
      Serial.print(F("Error: Expecting Config Mode Unlock Code (\"aqe\"), but received \""));
      Serial.print(buf);
      Serial.println(F("\""));
    }
  }
  
  // clean up the buffer if you got a line termination
  if(line_terminated){
    if(ret == CONFIG_MODE_NOTHING_SPECIAL){
      prompt(); 
    }
    buf[0] = '\0';
    buf_idx = 0;       
  }
  
  return ret;
}

void prompt(void){
  Serial.print(F("AQE>: ")); 
}

// command processing function implementations
void help_menu(char * arg){
  const uint8_t commands_per_line = 3;
  const uint8_t first_dynamic_command_index = 2;
  if(arg == 0){
    // list the commands that are legal
    Serial.print(F("1. help  \t2. exit  \t"));
    for(uint8_t ii = 0, jj = first_dynamic_command_index; commands[ii] != 0; ii++, jj++){
      if((jj % commands_per_line) == 0){
        Serial.println();
      }
      Serial.print(jj + 1);
      Serial.print(". ");
      Serial.print(commands[ii]);
      Serial.print('\t');       
    } 
    Serial.println();
  }
  else{
    
  }
}

void print_eeprom_value(char * arg){  
  if(strncmp(arg, "mac", 3) == 0){
    uint8_t _mac_address[6] = {0};
    // retrieve the value from EEPROM
    eeprom_read_block(_mac_address, (const void *) EEPROM_MAC_ADDRESS, 6);
    
    // print the stored value, formatted
    for(uint8_t ii = 0; ii < 6; ii++){
      if(_mac_address[ii] < 0x10){
        Serial.print(F("0"));
      }
      Serial.print(_mac_address[ii], HEX);
      
      // only print colons after the first 5 values
      if(ii < 5){
        Serial.print(F(":"));  
      }
    }
    Serial.println();
  }
  else{
    Serial.print(F("Error: Unexpected Variable Name \""));
    Serial.print(arg);
    Serial.println(F("\""));
  }
}

void initialize_eeprom_value(char * arg){
  if(strncmp(arg, "mac", 3) == 0){
    uint8_t _mac_address[6];
    if(!cc3000.getMacAddress(_mac_address)){
      Serial.println(F("Error: Could not retrieve MAC address from CC3000"));    
    }
    else{
      eeprom_write_block(_mac_address, (void *) EEPROM_MAC_ADDRESS, 6);
      recomputeAndStoreConfigChecksum();
    }   
  }
  else{
    Serial.print(F("Error: Unexpected Variable Name \""));
    Serial.print(arg);
    Serial.println(F("\""));
  }
}

void restore(char * arg){
  if(strncmp(arg, "mac", 3) == 0){
    uint8_t _mac_address[6] = {0};
    eeprom_read_block(_mac_address, (const void *) EEPROM_MAC_ADDRESS, 6);
    if (!cc3000.setMacAddress(_mac_address)){
       Serial.println(F("Error: Failed to restore MAC address to CC3000"));
    }
  } 
  else{
    Serial.print(F("Error: Unexpected Variable Name \""));
    Serial.print(arg);
    Serial.println(F("\""));
  }  
}

// goes into the CC3000 and stores the 
// MAC address from it in the EEPROM
void set_mac_address(char * arg){
  uint8_t _mac_address[6] = {0}; 
  char tmp[32] = {0};
  strncpy(tmp, arg, 31); // copy the string so you don't mutilate the argument
  char * token = strtok(tmp, ":");
  uint8_t num_tokens = 0;
  
  // parse the argument string, expected to be of the form ab:01:33:51:c8:77
  while(token != NULL){
    if((strlen(token) == 2) && isxdigit(token[0]) && isxdigit(token[1]) && (num_tokens < 6)){
      _mac_address[num_tokens++] = atoi(token);
    }
    else{
      Serial.print(F("Error: MAC address parse error on input \""));
      Serial.print(arg);
      Serial.println(F("\""));
      return; // return early
    }
    token = strtok(NULL, ":");
  }
  
  if (!cc3000.setMacAddress(_mac_address)){
     Serial.println(F("Error: Failed to write MAC address to CC3000"));
  }  
  else{ // cc3000 mac address accepted
    eeprom_write_block(_mac_address, (void *) EEPROM_MAC_ADDRESS, 6);
    recomputeAndStoreConfigChecksum();   
  }
}

void recomputeAndStoreConfigChecksum(void){
  uint16_t crc = computeConfigChecksum();
  eeprom_write_word((uint16_t *) EEPROM_CRC_CHECKSUM, crc);
}

uint16_t computeConfigChecksum(void){
  uint16_t crc = 0;
  // the checksum is 2 bytes, so start computing the checksum at 
  // the second byte after it's location
  for(uint16_t address = EEPROM_CRC_CHECKSUM + 2; address <= E2END; address++){
    crc = _crc16_update(crc, eeprom_read_byte((const uint8_t *) address));
  }
  return crc;  
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
