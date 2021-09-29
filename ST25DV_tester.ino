
/*

  ST25DV NFC chip tester. (c)2021 josh.comb 

  Demonstrate having a phone write a block to the mailbox over NFC and have the Arduino 
  notice the new connection and read that block. 

  ST25DV is only powered from RF.
  Requires only 2 pins. 

  Phew that was fricken hard. 
  
 
  Requires these static registers already be set on the ST25DV:

  EH_MODE=0x00 - Force energy harvesting on at power up. We need this to power the VCC since the chip will not allow mailbox
                 transactions without VCC. 

  MB_MODE=0x01 - Allows the mailbox function to be enabled.


  Requires these connections:              
  SCL->Arduino A5
  SDA->Arduino A4
  GND->Arduino GND
  EH->VCC

  5K resistor from SDA to VCC
  100K resistor from SDA to GND
  
  To use:
  Load this sketch into an UNO and then set the the serial monitor to 1000000 and send a message. It should print. 

  To send a message:
  
  1. Load the ST NFC app on your phone.
  2. Set the MB_CTRL_DYN register to 0x01. This enables mailbox.
  3. Send a message using the following custom command on the phone...
  
      Custom Command
      Addressed Mode
      CMD_CODE = 0xac
      Manufacturer checked (0x02)
      Data: 00 00 (this is for the pointer to where to start reading in the mailbox buffer and len. 00,00 mean start at beginning and read whole thing)
  
*/  


#define I2C_VCC_PIN A3

#include "i2c.h"
#include "st25dv.h"
#include <util/crc16.h>

#define ST25_DEVICE_TYPE_ID (0b1010)  // "the 4-bit device type identifier is 1010b."

// The first byte sent with the start condition
// From datasheet page 66

constexpr uint8_t ST25_deviceSelectCode(  const uint8_t e2  , const uint8_t read_flag ) {

  return ( ST25_DEVICE_TYPE_ID << 4 ) | ( e2 ? 0b00001000 : 0b00000000) | ( 0b0110 ) | ( read_flag ? I2C_READ : I2C_WRITE ) ;
  
}


byte i2cWrite( const uint8_t *pData, const uint8_t e2, const uint16_t TarAddr,  uint16_t len) {

  uint8_t retVal=0;    

  i2c_start( ST25_deviceSelectCode( e2 , 0 ) );    // Get the slave's attention, tell it we're sending a command byte
  i2c_write(TarAddr >> 8);           //  The command byte, sets pointer to register with address of 0x32
  i2c_write(TarAddr & 0xFF);         //  The command byte, sets pointer to register with address of 0x32

  while (len-- && !retVal) {
    retVal = i2c_write(*pData++);   // Returns 0 on success
  }

  i2c_stop();

  return retVal;

}

void vccOn() {
  digitalWrite( I2C_VCC_PIN , 1 );  
  pinMode( I2C_VCC_PIN , OUTPUT ); 
}

// Force Vcc low to bleed off any charge

void vccOff() {
  digitalWrite( I2C_VCC_PIN , 0 );  
  pinMode( I2C_VCC_PIN , OUTPUT ); 
}


void vccFloat() {
  pinMode( I2C_VCC_PIN , INPUT );   
  digitalWrite( I2C_VCC_PIN , 0 );  
}


// pData is 8 byte password

void i2cPresentPassord( const uint8_t *pData) {

  byte buffer[ (8*2)+1];   // Password two with "present password" command between them

  for( byte i=0; i<8; i++ ) {

    buffer[i]=pData[i];
    buffer[i+9]=pData[i];     // 2nd copy of password
    
  }

  buffer[8] = 0x09;   // "Present password"

  i2cWrite( buffer ,   1 , ST25DV_I2CPASSWD_REG , (8*2) + 1 ); 


}


byte i2cRead( byte *pData, const uint8_t e2, const uint16_t TarAddr,  uint16_t len) {

  if (i2c_start(ST25_deviceSelectCode( e2 , 0 ))) {    // All reads start with a write to set the address
    // Got NAK
    return 1;
  }
  
  if (i2c_write(TarAddr >> 8)) {          
    // Got NAK
    return 1;
  }
  if (i2c_write(TarAddr & 0xFF)) {        
    // Got NAK
    return 1;
  }
  
  if (i2c_restart(ST25_deviceSelectCode( e2 , 1 ))) {  // Do a restart to now read from the address set above
    // Got NAK
    return 1;
  }
    
  while ( len ) {

    len--;

    const byte lastFlag = (len==0);
    byte c = i2c_read( lastFlag );      // the param here is called "last" and is true if this is the lasy byte to read
    *pData = c;
    pData++;
  }

  i2c_stop();

  return 0;
  
}

void readUUID() {

  
  byte b[256];
  
  if ( !i2cRead( b , 1 , 0x18 , 0x0008 ) ) {

    Serial.println("Read good...");  

    for( int i=0; i<0x18; i++ ) {
          Serial.print( i );              
          Serial.print( ' ' );                  
          Serial.print( (char) b[i] );        
          Serial.print( ' ' );                            
          Serial.println( (unsigned) b[i] , 16 );        
                
    }

    
  } else {

    Serial.println("Read fail.");  

  }
}


// Program static registers. Must be done once per chip.
// Returns 0=success, 1=failure. 
// Turns on power to chip, programs the registers, then turns off power. 
// Sets...
// 1. Mailbox to default allowed
// 2. GPO pulses low on RF change

// Note that we pick the pulse low in RF becuase...
// Using a message_put to wake would be better, but the RF side can not enable mailbox until there is Vcc power
// Using a RF low would be better than pulse, but id the GPO pin is low then we can do not any i2c to turn it off since it shares the SDA pin.

uint8_t initialProgramming() {

   // Turn on Vcc
  Serial.println("Turn on power to ST25...");   
  vccOn();


  _delay_us(600);   // Tboot=0.6ms, time from power up until i2c available

  Serial.println("Send password to open security session (nessisary to change regs)..."); 

  const byte password_in_array[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};    // Default password
  i2cPresentPassord( password_in_array );

  Serial.println("Check security session open success..."); 
  uint8_t sso_reg;
  i2cRead( &sso_reg , 0 , ST25DV_I2C_SSO_DYN_REG , 0x0001 );
  Serial.print("SSO=");
  Serial.println( sso_reg , 16 );

  if ( ! ( sso_reg  & ST25DV_I2C_SSO_DYN_I2CSSO_MASK) ) {
    Serial.println("Security session NOT open. Maybe wrong password? Abort."); 
    return 1; 
  }

  // Enable GPO interrupt on "RF_INTERRUPT". The RF side will send the "Manage GPO" command to
  // pulse the GPO pin low, which will wake the MCU and start an NFC session. 

  // Also enable on "RF_PUT". This will pulse GPO pin low when a new mailbox message has been received from
  // the RF side. This should be faster than polling thew mailbox regs since that polling on the i2c bus
  // could block the RF side form doing its put. 

  Serial.println("Write enable GPIO on RF INTERRUPT...");
  uint8_t new_gpio_reg_val = ST25DV_GPO_RFINTERRUPT_MASK | ST25DV_GPO_DYN_RFPUTMSG_MASK | ST25DV_GPO_ENABLE_MASK;
  i2cWrite(  &new_gpio_reg_val , 1 , ST25DV_GPO_REG , 0x0001 );


  Serial.println("Wait for EEPROM write to complete...");

  /*
      I²C write time = 5 ms
      I2C write time for 1 Byte, 2 Bytes, 3 Bytes or 4 Bytes in EEPROM (user memory and system configuration),
      provided they are all located in the same memory page, that is the most significant memory address bits
      (b16-b2) are the same.
   */

  _delay_ms(5);  // Must wait 

  // Check the register updated...
   
  // Read static GPIO 
  uint8_t gpo_reg_read;
  i2cRead( &gpo_reg_read , 1 , ST25DV_GPO_REG, 0x0001 );
  Serial.print("GPO_REG=");
  Serial.println( gpo_reg_read , 16 );   

  if (gpo_reg_read!=new_gpio_reg_val) {
    Serial.println("GPO_REGG write failed. Abort."); 
    return 1;     
  }

  // Next authorize mailbox operations. This allows us to later dynamically enable the mailbox on each power up. 
  
  Serial.println("Authorizing mailbox mode...");
  uint8_t new_mbmode_reg_val = ST25DV_MB_MODE_RW_MASK;    // That is an odd name for this field?
  i2cWrite(  &new_mbmode_reg_val , 1 , ST25DV_MB_MODE_REG , 0x0001 );

  Serial.println("Wait for EEPROM write to complete...");

  /*
      I²C write time = 5 ms
      I2C write time for 1 Byte, 2 Bytes, 3 Bytes or 4 Bytes in EEPROM (user memory and system configuration),
      provided they are all located in the same memory page, that is the most significant memory address bits
      (b16-b2) are the same.
   */

  _delay_ms(5);  // Must wait 

  // Check the register updated...
   
  // Read static GPIO 
  uint8_t mb_mode_read;
  i2cRead( &mb_mode_read , 1 , ST25DV_MB_MODE_REG, 0x0001 );
  Serial.print("MB_MODE=");
  Serial.println( mb_mode_read , 16 );   

  if (mb_mode_read!=new_mbmode_reg_val) {
    Serial.println("MB_MODE write failed. Abort."); 
    return 1;     
  }
  

  Serial.print("Initial programming successfully completed. Powwering ST25 down.");

  // Turn off Vcc   
  vccFloat();
  
  // TODO: Maybe protect all regs and add a password? Add NDEF record to open app store/web page? 

  return 0;
  
}


void setup() {
  
  Serial.begin(1000000);

  delay(100);
  Serial.println("\r\nNFC blink emulator 1.0.");  

  // Make sure ST25DV is reset
  vccOff();
  delay(10);
  vccFloat();

  // Set up pins
  // Leaves SDA pull-up
  i2c_init(); 
  delay(1);      // Wait for SDA to pull high

  // Only needs to be done one time per ST25 chip, but no harm doing multipule times
  // initialProgramming(); 

}


void print_mb_dyn() {
  uint8_t mb_dyn_reg;

  i2cRead( &mb_dyn_reg , 0 , ST25DV_MB_CTRL_DYN_REG , 0x0001 );  

  Serial.print("mb_dyn=");
  Serial.println( mb_dyn_reg , 16 );
}


void readblocks() {

  Serial.println("Begin game download.");  
    
  uint16_t gameLen = 0;
  uint16_t expected_crc=0;
  uint16_t received_crc=0xffff;   // Defined starting value

  do {    

    Serial.println("Waiting for GPO pulse to indicate mailbox message from RF side...");
  
    while ( SDA_PIN & _BV(SDA_BIT) );

    Serial.println("Wait for GPO pulse command from RF side to be over so we can i2c...");
    
    while ( ! (SDA_PIN & _BV(SDA_BIT)) );

    print_mb_dyn();
    
    uint8_t mb_len_reg;

    i2cRead( &mb_len_reg , 0 , ST25DV_MB_LEN_DYN_REG , 0x0001 );    // "Size in byte, minus 1 byte, of message contained in FTM mailbox"

    const uint16_t mb_len = mb_len_reg + 1;  // Number of bytes is len reg + 1, so if reg=0 then 1 byte in mailbox buffer

    Serial.print("Got block from RF. LEN=");
    Serial.println( mb_len );   

    uint8_t mailbox_buffer[ST25DV_MAX_MAILBOX_LENGTH];

    const uint8_t rr = i2cRead( mailbox_buffer , 0 , ST25DV_MAILBOX_RAM_REG , mb_len );

    if (rr) {

      Serial.print("i2cread error=");        
      Serial.println(rr);        
    }

    if (gameLen==0) {
      // read header

      gameLen       = mailbox_buffer[0] | (mailbox_buffer[1] * 0x100);
      expected_crc  = mailbox_buffer[2] | (mailbox_buffer[3] * 0x100);

      Serial.print("header len=");
      Serial.println( gameLen );
      Serial.print("header crc=");
      Serial.println( expected_crc );
      
    } else {
      // read game block
  
      Serial.print("game data:");

      for( uint16_t i=0; i<mb_len; i++ ) {
        uint8_t c = mailbox_buffer[i];
        received_crc  = _crc16_update  ( received_crc , mailbox_buffer[i]) ;               
        Serial.print( (char) c );
      }
      
      Serial.print("crc after this block: ");
      Serial.println( received_crc );


      gameLen -= mb_len;
        
      Serial.print("remaining bytes after this block: ");
      Serial.println( gameLen  );
      
    }
   
  } while (gameLen>0);

  
  Serial.println("Game download complete.");  
  
}

void loop() {


  // First we wait for SDA to be pulled low by the GPO pin. This signals the phone has sent the
  // GPO interrupt command. 

  // On connection, we will have the app send the Manage GPO command to drive GPO pin low...
  // Command=0xa9 (Magage GPO), Data 0x80 (USER_INTERUPT - GPO pin pulsed to 0 during IT Time then released)
  // This command can be sent without Vcc power, so we use it to wake the MCU which will then power up Vcc
  // We could also trigger on an RF field being present, but that could unessisarily wake MCU on any
  // RF thing happening (NDEF  ref, or even just a field being present). This will limit us to only 
  // waking pretty much on a good connection from our own app. 
  
  Serial.println("Wait for GPO pulse command from RF side...");

  while ( SDA_PIN & _BV(SDA_BIT) );

  uint32_t startTime = millis(); 
  
   // Turn on Vcc
  Serial.println("Turn on power to ST25...");   
  vccOn();

  _delay_ms(1); // Give time for the GPO pulse to end. Since GPO and SDA are connected to the same MCU pin 
                // we can not do any i2c until this pulse is over. 
                // This also gives us time for Tboot=0.6ms, time from power up until i2c available
                
  //print_mb_dyn();
  
  // OK, now should be all clear to use i2c now

  // Now we are ready to send the welcome_bundle message to the RF side. This tells the app (1) it is talking to a 
  // blink, the latest high score data to save, (3) that we are ready to start downloading game blocks. 

  // First we need to enable mailbox

  Serial.println("Enable mailbox...");

  const uint8_t mailbox_enable_reg_val = ST25DV_MB_CTRL_DYN_MBEN_MASK;
  
  i2cWrite(  &mailbox_enable_reg_val , 0 , ST25DV_MB_CTRL_DYN_REG , 0x0001 );  

  //print_mb_dyn();

  Serial.println("Putting gamestat block in mailbox...");
  // Send a mailbox
  const uint8_t message[] = "bks1";   // Gamstat always starts with this magic cookie 

  #warning put real gamestats here. 
  i2cWrite(  message , 0 , ST25DV_MAILBOX_RAM_REG  , sizeof( message ) + 251 );

  print_mb_dyn();

  // Now wait for the phone to send us some blocks...

  uint32_t endTime = millis();

  Serial.print("Millis until gamestats mailbox ready=");
  Serial.println( endTime - startTime );   

  readblocks();
  
  vccFloat();

}
