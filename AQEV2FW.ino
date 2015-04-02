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

#define EEPROM_MAC_ADDRESS    (E2END + 1 - 6)    // MAC address, i.e. the last 6-bytes of EEPROM
                                                 // more parameters follow, address relative to each other so they don't overlap                                                 
#define EEPROM_CONNECT_METHOD (EEPROM_MAC_ADDRESS - 1) // connection method encoded as a single byte value 
#define EEPROM_SSID           (EEPROM_CONNECT_METHOD - 32) // ssid string, up to 32 characters (one of which is a null terminator)
#define EEPROM_NETWORK_PWD    (EEPROM_SSID - 32) // network password, up to 32 characters (one of which is a null terminator)
#define EEPROM_SECURITY_MODE  (EEPROM_NETWORK_PWD - 1) // security mode encoded as a single byte value, consistent with the CC3000 library
#define EEPROM_STATIC_IP_ADDRESS (EEPROM_SECURITY_MODE - 4) // static ipv4 address, 4 bytes - 0.0.0.0 indicates use DHCP
#define EEPROM_CRC_CHECKSUM   (E2END + 1 - 1024) // reserve the last 1kB for config

// valid connection methods
// only DIRECT is supported initially
#define CONNECT_METHOD_DIRECT        (0)
#define CONNECT_METHOD_SMARTCONFIG   (1)
#define CONNECT_METHOD_PFOD          (2)

void help_menu(char * arg);
void print_eeprom_value(char * arg);
void initialize_eeprom_value(char * arg);
void restore(char * arg);
void set_mac_address(char * arg);
void set_connection_method(char * arg);
void set_ssid(char * arg);
void set_network_password(char * arg);
void set_network_security_mode(char * arg);
void set_static_ip_address(char * arg);
void use_command(char * arg);

// Note to self:
//   When implementing a new parameter, ask yourself:
//     should there be a command for the user to set its value directly
//     should 'get' support it (almost certainly the answer is yes)
//     should 'init' support it (is there a way to set it without user intervention)
//     should 'restore' support it directly
//     should 'restore defaults' support it
//   ... and anytime you do the above, remember to update the help_menu
//   ... and remember, anything that changes the config EEPROM 
//       needs to call recomputeAndStoreConfigChecksum after doing so

// the order of the command keywords in this array
// must be kept in index-correspondence with the associated 
// function pointers in the command_functions array
//
// these keywords are padded with spaces
// in order to ease printing as a table
// string comparisons should use strncmp rather than strcmp
char * commands[] = {
  "get     ",
  "init    ",
  "restore ",  
  "setmac  ",  
  "method  ",
  "ssid    ",
  "pwd     ",
  "security",
  "staticip",
  "use     ",
  0
};

void (*command_functions[])(char * arg) = {
  print_eeprom_value,
  initialize_eeprom_value,
  restore,
  set_mac_address,
  set_connection_method,
  set_ssid,
  set_network_password,
  set_network_security_mode,
  set_static_ip_address,
  use_command,
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
    configInject("aqe\r");
    mode = MODE_CONFIG;
  }  
  else{
    // if the appropriate escape sequence is received within 8 seconds
    // go into config mode
    const long startup_time_period = 12000;
    long start = millis();
    long min_over = 100;
    boolean got_serial_input = false;
    Serial.println(F("Enter 'aqe' for CONFIG mode."));
    Serial.print(F("OPERATIONAL mode automatically begins after "));
    Serial.print(startup_time_period/1000);
    Serial.println(F(" secs of no input."));
    while(millis() < start + startup_time_period){ // can get away with this sort of thing at start up
       if(Serial.available()){
         if(got_serial_input == false){
           Serial.println();                      
         }
         got_serial_input = true;         

         start = millis(); // reset the timeout
         if(CONFIG_MODE_GOT_INIT == configModeStateMachine(Serial.read(), false)){
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
    const uint32_t idle_timeout_period_ms = 1000UL * 60UL * 5UL; // 5 minutes
    uint32_t idle_time_ms = 0;
    Serial.println(F("-~=* In CONFIG Mode *=~-"));
    Serial.print(F("OPERATIONAL mode begins automatically after "));
    Serial.print((idle_timeout_period_ms / 1000UL) / 60UL);
    Serial.println(F(" mins without input.")); 
    Serial.println(F("Enter 'help' for a list of available commands, "));
    Serial.println(F("      ...or 'help <cmd>' for help on a specific command"));
    prompt();
    for(;;){
      unsigned long current_millis = millis();
            
      // stuck in this loop until the command line receives an exit command
      if(Serial.available()){
        idle_time_ms = 0;
        // if you get serial traffic, pass it along to the configModeStateMachine for consumption
        if(CONFIG_MODE_GOT_EXIT == configModeStateMachine(Serial.read(), false)){
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
uint8_t configModeStateMachine(char b, boolean reset_buffers){
  static boolean received_init_code = false;
  const char buf_max_write_idx = 62; // [63] must always have a null-terminator
  static char buf[64] = {0}; // buffer to hold commands / data
  static uint8_t buf_idx = 0;  // current number of bytes in buf  
  boolean line_terminated = false;
  char * first_arg = 0;
  uint8_t ret = CONFIG_MODE_NOTHING_SPECIAL;
  
  if(reset_buffers){
    buf_idx = 0;
  }
  
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
  
  char lower_buf[64] = {0};  
  if(line_terminated){
    strncpy(lower_buf, buf, 63);
    lowercase(lower_buf);  
  }  
  
  // process the data currently stored in the buffer
  if(received_init_code && line_terminated){  
    // with the exeption of the command "exit"
    // commands are always of the form <command> <argument>
    // they are minimally parsed here and delegated to 
    // callback functions that take the argument as a string   

    // Serial.print("buf = ");
    // Serial.println(buf);
       
    if(strncmp("exit", lower_buf, 4) == 0){      
      Serial.println(F("Exiting CONFIG mode..."));
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
      if(strncmp("help", lower_buf, 4) == 0){  
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
          if(strncmp(commands[ii], lower_buf, strlen(buf)) == 0){
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
    
    if(strncmp("aqe", lower_buf, 3) == 0){
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
void configInject(char * str){
  boolean reset_buffers = true;
  while(*str != '\0'){
    configModeStateMachine(*str++, reset_buffers);
    if(reset_buffers){
      reset_buffers = false;
    }
  }  
}

void lowercase(char * str){
  uint8_t len = strlen(str);
  if(len < 255){ // guard against an infinite loop
    for(uint8_t ii = 0; ii < len; ii++){
      str[ii] = tolower(str[ii]);
    } 
  }
}

void help_menu(char * arg){
  const uint8_t commands_per_line = 3;
  const uint8_t first_dynamic_command_index = 2;
  
  lowercase(arg);
  
  if(arg == 0){
    // list the commands that are legal
    Serial.print(F("help    \texit    \t"));
    for(uint8_t ii = 0, jj = first_dynamic_command_index; commands[ii] != 0; ii++, jj++){
      if((jj % commands_per_line) == 0){
        Serial.println();
      }
      //Serial.print(jj + 1);
      //Serial.print(". ");
      Serial.print(commands[ii]);
      Serial.print('\t');       
    } 
    Serial.println();
  }
  else{
    // we have an argument, so the user is asking for some specific usage instructions
    // as they pertain to this command
    if(strncmp("help", arg, 4) == 0){
      Serial.println(F("help <param>"));
      Serial.println(F("   <param> is any legal command keyword"));
      Serial.println(F("   result: usage instructions are printed"));
      Serial.println(F("           for the command named <arg>"));     
    }
    else if(strncmp("exit", arg, 4) == 0){
      Serial.println(F("exit"));
      Serial.println(F("   exits CONFIG mode and begins OPERATIONAL mode."));
    }
    else if(strncmp("get", arg, 3) == 0){
      Serial.println(F("get <param>"));
      Serial.println(F("   <param> is one of:"));
      Serial.println(F("      mac - the MAC address of the cc3000"));
      Serial.println(F("      method - the Wi-Fi connection method"));
      Serial.println(F("      ssid - the Wi-Fi SSID to connect to"));
      Serial.println(F("      pwd - lol, sorry, that's not happening!"));
      Serial.println(F("      security - the Wi-Fi security mode"));
      Serial.println(F("      staticip - the Wi-Fi IP-address mode"));
      Serial.println(F("   result: the current, human-readable, value of <param>"));
      Serial.println(F("           is printed to the console."));      
    }
    else if(strncmp("init", arg, 4) == 0){
      Serial.println(F("init <param>"));
      Serial.println(F("   <param> is one of:"));
      Serial.println(F("      mac - retrieves the mac address from"));
      Serial.println(F("            the CC3000 and stores it in EEPROM"));
    }
    else if(strncmp("restore", arg, 7) == 0){
      Serial.println(F("restore <param>"));
      Serial.println(F("   <param> is one of:"));
      Serial.println(F("      defaults - performs 'method direct'"));
      Serial.println(F("                 performs 'init mac'"));      
      Serial.println(F("                 performs 'security wpa2'"));
      Serial.println(F("                 performs 'use dhcp'"));
      Serial.println(F("                 clears the SSID from memory"));
      Serial.println(F("                 clears the Network Password from memory"));
      Serial.println(F("      mac      - retrieves the mac address from"));
      Serial.println(F("                 EEPROM and assigns it to the CC3000"));
    }
    else if(strncmp("setmac", arg, 6) == 0){
      Serial.println(F("setmac <address>"));
      Serial.println(F("   <address> is a MAC address of the form:"));
      Serial.println(F("                08:ab:73:DA:8f:00"));
      Serial.println(F("   result: The entered MAC address is assigned to the CC3000"));
      Serial.println(F("           but is NOT stored in the EEPROM, so that 'restore mac'"));
      Serial.println(F("           can subsequently be used to undo this command."));            
      Serial.println(F("   note:   If you DO want to store the newly configured MAC in EEPROM,"));
      Serial.println(F("           follow this command by 'init mac'."));                  
    }
    else if(strncmp("method", arg, 6) == 0){
      Serial.println(F("method <type>"));
      Serial.println(F("   <type> is one of:"));
      Serial.println(F("      direct - use parameters entered in CONFIG mode"));
      Serial.println(F("      smartconfig - use smart config process [not yet supported]"));      
      Serial.println(F("      pfod - use pfodWifiConnect config process  [not yet supported]"));            
    }    
    else if(strncmp("ssid", arg, 4) == 0){
      Serial.println(F("ssid <string>"));
      Serial.println(F("   <string> is the SSID of the network the device should connect to."));
    }
    else if(strncmp("pwd", arg, 3) == 0){
      Serial.println(F("pwd <string>"));
      Serial.println(F("   <string> is the network password for "));
      Serial.println(F("      the SSID that the device should connect to."));
    }    
    else if(strncmp("security", arg, 8) == 0){
      Serial.println(F("security <mode>"));
      Serial.println(F("   <mode> is one of:"));
      Serial.println(F("      open - the network is unsecured"));
      Serial.println(F("      wep  - the network WEP security"));
      Serial.println(F("      wpa  - the network WPA Personal security"));  
      Serial.println(F("      wpa2 - the network WPA2 Personal security"));        
    }
    else if(strncmp("staticip", arg, 8) == 0){
      Serial.println(F("staticip <address>"));
      Serial.println(F("   <address> is an IPv4 address of the form:"));
      Serial.println(F("                192.168.1.152"));
      Serial.println(F("   result: The entered IPv4 address will be used by the CC3000"));
      Serial.println(F("   note:   To configure DHCP use command 'use dhcp'"));
    }
    else if(strncmp("use", arg, 3) == 0){
      Serial.println(F("use <param>"));
      Serial.println(F("   <param> is one of:"));
      Serial.println(F("      dhcp - wipes the Static IP address from the EEPROM"));
    }
    else{
      Serial.print(F("Error: There is no help available for command \""));
      Serial.print(arg);
      Serial.println(F("\"")); 
    }
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
  else if(strncmp(arg, "method", 6) == 0){
    uint8_t method = eeprom_read_byte((const uint8_t *) EEPROM_CONNECT_METHOD);
    switch(method){
      case CONNECT_METHOD_DIRECT:
        Serial.println(F("Direct Connect"));
        break;
      case CONNECT_METHOD_SMARTCONFIG:
        Serial.println(F("Smart Config Connect [not currently supported]"));
        break;
      case CONNECT_METHOD_PFOD:
        Serial.println(F("Pfod Wi-Fi Connect [not currently supported]"));
        break;
      default:
        Serial.print(F("Error: Unknown connection method code [0x"));
        if(method < 0x10){
          Serial.print(F("0")); 
        }
        Serial.print(method, HEX);
        Serial.println(F("]"));
        break;   
    }
    
  }
  else if(strncmp(arg, "ssid", 4) == 0){
    char ssid[32] = {0};
    eeprom_read_block(ssid, (const void *) EEPROM_SSID, 32);
    if(strlen(ssid) == 0){
      Serial.println(F("No SSID currently configured."));
    }
    else{
      Serial.println(ssid);
    }
  }
  else if(strncmp(arg, "security", 8) == 0){
    uint8_t security = eeprom_read_byte((const uint8_t *) EEPROM_SECURITY_MODE);    
    switch(security){
      case WLAN_SEC_UNSEC:
        Serial.println(F("Open"));
        break;
      case WLAN_SEC_WEP:
        Serial.println(F("WEP"));
        break;
      case WLAN_SEC_WPA:
        Serial.println(F("WPA"));
        break;
      case WLAN_SEC_WPA2:
        Serial.println(F("WPA2"));
        break;
      default:
        Serial.print(F("Error: Unknown security mode code [0x"));
        if(security < 0x10){
          Serial.print(F("0")); 
        }
        Serial.print(security, HEX);
        Serial.println(F("]"));
        break;   
    }    
  }  
  else if(strncmp(arg, "staticip", 8) == 0){
    uint8_t ip[4] = {0};
    uint8_t noip[4] = {0};
    eeprom_read_block(ip, (const void *) EEPROM_STATIC_IP_ADDRESS, 4);
    if(memcmp(ip, noip, 4) == 0){
      Serial.println(F("Configured for DHCP"));      
    }
    else{
      Serial.print(F("Configured for Static IP Address: "));
      for(uint8_t ii = 0; ii < 3; ii++){
        Serial.print(ip[ii], DEC);
        Serial.print(F("."));
      } 
      Serial.println(ip[3], DEC);
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
  char blank[32] = {0};
  if(strncmp(arg, "defaults", 8) == 0){
    prompt();
    configInject("init mac\r");
    configInject("method direct\r");
    configInject("security wpa2\r");
    configInject("use dhcp\r");
    eeprom_write_block(blank, (void *) EEPROM_SSID, 32); // clear the SSID
    eeprom_write_block(blank, (void *) EEPROM_NETWORK_PWD, 32); // clear the Network Password
    recomputeAndStoreConfigChecksum();
    Serial.println();
  }
  else if(strncmp(arg, "mac", 3) == 0){
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

void set_mac_address(char * arg){
  uint8_t _mac_address[6] = {0}; 
  char tmp[32] = {0};
  strncpy(tmp, arg, 31); // copy the string so you don't mutilate the argument
  char * token = strtok(tmp, ":");
  uint8_t num_tokens = 0;
  
  // parse the argument string, expected to be of the form ab:01:33:51:c8:77
  while(token != NULL){
    if((strlen(token) == 2) && isxdigit(token[0]) && isxdigit(token[1]) && (num_tokens < 6)){
      _mac_address[num_tokens++] = (uint8_t) strtoul(token, NULL, 16);
    }
    else{
      Serial.print(F("Error: MAC address parse error on input \""));
      Serial.print(arg);
      Serial.println(F("\""));
      return; // return early
    }
    token = strtok(NULL, ":");
  }
  
  if(num_tokens == 6){
    if (!cc3000.setMacAddress(_mac_address)){
       Serial.println(F("Error: Failed to write MAC address to CC3000"));
    }  
    else{ // cc3000 mac address accepted
      // eeprom_write_block(_mac_address, (void *) EEPROM_MAC_ADDRESS, 6);
      // recomputeAndStoreConfigChecksum();   
    } 
  }
  else{
    Serial.println(F("Error: MAC address must contain 6 bytes, with each separated by ':'")); 
  } 
}

void set_connection_method(char * arg){
  lowercase(arg);
  boolean valid = true;
  if(strncmp(arg, "direct", 6) == 0){
    eeprom_write_byte((uint8_t *) EEPROM_CONNECT_METHOD, CONNECT_METHOD_DIRECT);
  }
  else if(strncmp(arg, "smartconfig", 11) == 0){
    eeprom_write_byte((uint8_t *) EEPROM_CONNECT_METHOD, CONNECT_METHOD_SMARTCONFIG);
  }
  else if(strncmp(arg, "pfod", 4) == 0){
    eeprom_write_byte((uint8_t *) EEPROM_CONNECT_METHOD, CONNECT_METHOD_PFOD);
  }
  else{
    Serial.print(F("Error: Invalid connection method entered - \""));
    Serial.print(arg);
    Serial.println(F("\""));
    Serial.println(F("       valid options are: 'direct', 'smartconfig', and 'pfod'"));
    valid = false; 
  }
  
  if(valid){
    recomputeAndStoreConfigChecksum();
  }
}

void set_ssid(char * arg){
  // we've reserved 32-bytes of EEPROM for an SSID
  // so the argument's length must be <= 31
  char ssid[32] = {0};
  uint16_t len = strlen(arg);
  if(len < 32){
    strncpy(ssid, arg, len);
    eeprom_write_block(ssid, (void *) EEPROM_SSID, 32);
    recomputeAndStoreConfigChecksum();
  }
  else{
    Serial.println(F("Error: SSID must be less than 32 characters in length"));
  }
}

void set_network_password(char * arg){
  // we've reserved 32-bytes of EEPROM for a network password
  // so the argument's length must be <= 31
  char password[32] = {0};
  uint16_t len = strlen(arg);
  if(len < 32){
    strncpy(password, arg, len);
    eeprom_write_block(password, (void *) EEPROM_NETWORK_PWD, 32);
    recomputeAndStoreConfigChecksum();
  }
  else{
    Serial.println(F("Error: SSID must be less than 32 characters in length"));
  }  
}

void set_network_security_mode(char * arg){
  boolean valid = true;
  if(strncmp("open", arg, 4) == 0){
    eeprom_write_byte((uint8_t *) EEPROM_SECURITY_MODE, WLAN_SEC_UNSEC);
  }
  else if(strncmp("wep", arg, 3) == 0){
    eeprom_write_byte((uint8_t *) EEPROM_SECURITY_MODE, WLAN_SEC_WEP);
  }
  else if(strncmp("wpa2", arg, 4) == 0){
    eeprom_write_byte((uint8_t *) EEPROM_SECURITY_MODE, WLAN_SEC_WPA2);
  }  
  else if(strncmp("wpa", arg, 3) == 0){
    eeprom_write_byte((uint8_t *) EEPROM_SECURITY_MODE, WLAN_SEC_WPA);
  }
  else{
    Serial.print(F("Error: Invalid security mode entered - \""));
    Serial.print(arg);
    Serial.println(F("\""));
    Serial.println(F("       valid options are: 'open', 'wep', 'wpa', and 'wpa2'"));
    valid = false;
  }
  
  if(valid){
    recomputeAndStoreConfigChecksum();
  }    
}

void set_static_ip_address(char * arg){
  uint8_t _ip_address[4] = {0}; 
  char tmp[32] = {0};
  strncpy(tmp, arg, 31); // copy the string so you don't mutilate the argument
  char * token = strtok(tmp, ".");
  uint8_t num_tokens = 0;
  
  // parse the argument string, expected to be of the form 192.168.1.52
  while(token != NULL){
    uint8_t tokenlen = strlen(token);
    if((tokenlen < 4) && (num_tokens < 4)){
      for(uint8_t ii = 0; ii < tokenlen; ii++){
        if(!isdigit(token[ii])){
          Serial.println(F("Error: Static IP address octets must be integer values"));
          return;
        }
      }
      uint32_t octet = (uint8_t) strtoul(token, NULL, 10);
      if(octet < 256){
        _ip_address[num_tokens++] = octet;
      }
      else{
        Serial.println(F("Error: Static IP address octets must be less between 0 and 255 inclusive."));
        return; 
      }
    }
    else{
      Serial.print(F("Error: Static IP address parse error on input \""));
      Serial.print(arg);
      Serial.println(F("\""));
      return; // return early
    }
    token = strtok(NULL, ".");
  }
  
  if(num_tokens == 4){
    eeprom_write_block(_ip_address, (void *) EEPROM_STATIC_IP_ADDRESS, 4);
    recomputeAndStoreConfigChecksum();   
  }
  else{
    Serial.println(F("Error: Static IP Address must contain 4 valid octets separated by '.'")); 
  }   
}

void use_command(char * arg){
  const uint8_t noip[4] = {0};
  if(strncmp("dhcp", arg, 3) == 0){
    eeprom_write_block(noip, (void *) EEPROM_STATIC_IP_ADDRESS, 4);
    recomputeAndStoreConfigChecksum();    
  }
  else{
    Serial.print(F("Error: Invalid parameter provided to use - \""));
    Serial.print(arg);
    Serial.println("\""); 
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
