
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

/** @brief I2C address to be used for ST25DV Data accesses. */
#define ST25DV_ADDR_DATA_I2C                 0xA6
/** @brief I2C address to be used for ST25DV System accesses. */
#define ST25DV_ADDR_SYST_I2C                 0xAE

void setup() {
  
  Serial.begin(1000000);

  delay(100);
  Serial.println("\r\nNFC JOSH Start...");  

  i2c_init();

  // put your setup code here, to run once:

    
}




byte i2cWrite( const byte *pData, const uint8_t addr, const uint16_t TarAddr,  uint16_t len) {

  i2c_start(addr|I2C_WRITE );    // Get the slave's attention, tell it we're sending a command byte
  i2c_write(TarAddr >> 8);           //  The command byte, sets pointer to register with address of 0x32
  i2c_write(TarAddr & 0xFF);         //  The command byte, sets pointer to register with address of 0x32

  Serial.print("WRITE:");
  while (len--) {
    Serial.print( (int) *pData , 16 );
    Serial.print( ' ' );
    i2c_write(*pData++);
  }

  i2c_stop();

  Serial.println(".");

  return 0;

}


// pData is 8 byte password

void i2cPresentPassord( const byte *pData) {

  byte buffer[8*2+1];   // Password two with "present password" command between them

  for( byte i=0; i<8; i++ ) {

    buffer[i]=pData[i];
    buffer[i+9]=pData[i];     // 2nd copy of password
    
  }

  buffer[8] = 0x09;   // "Present password"

  i2cWrite( buffer ,   ST25DV_ADDR_SYST_I2C , 0x0900 , 17 );  // Device select code = 1010111


}


byte i2cRead( byte *pData, const uint8_t addr, const uint16_t TarAddr,  uint16_t len) {

  i2c_start(addr|I2C_WRITE);    // Get the slave's attention, tell it we're sending a command byte
  i2c_write(TarAddr >> 8);           //  The command byte, sets pointer to register with address of 0x32
  i2c_write(TarAddr & 0xFF);         //  The command byte, sets pointer to register with address of 0x32
  
  i2c_restart(addr|I2C_READ);
    
  while ( len ) {

    len--;

    byte c = i2c_read(len==0 );      // the param here is called "last" and is true if this is the lasy byte to read
    *pData = c;
    pData++;
  }

  i2c_stop();

  return 0;
  
}

void readUUID() {

  
  byte b[256];
  
  if ( !i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x18 , 0x008 ) ) {

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

void loop() {

  // First we wait for SDA to go high. Ths is pulled to VCC and EH by a 10K resistor so it will go high when the chip sees RF power
  // This just happens to work out since i2c idle is both lines pulled high

/*

  Serial.println("Waiting for RF power...");

  while ( !(SDA_INPUT_REG & _BV(SDA_PIN)) );


  Serial.println("We have an RF connection!");
*/

  // Turn on Vcc
  digitalWrite( I2C_VCC_PIN , 1 );
  pinMode( I2C_VCC_PIN , OUTPUT );

  delay(200);   // Wait for some power to stabilize

  unsigned long start_time = millis(); 

  byte b[256];

  const byte one_in_array[] = {0x01};

  // First we enable the mailbox so the RF side doesn't have to (we can do it faster)

  Serial.println("Read MB_CTRL...");

  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  Serial.print("MB_CTRL=?:");                                                      
  Serial.println( b[0] , 16 );     

  Serial.println("Write MB_CTRL...");
  
  i2cWrite(  one_in_array , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  delay(5);  // Tw i2c max write time 
  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  Serial.print("MB_CTRL=1:");                                                      
  Serial.println( b[0] , 16 );     

  // Keep waiting for messages as long as there is RF power

  while ( (SDA_PIN & _BV(SDA_BIT)) ) {
    
    // Now we poll waiting for a message to come in or for power to drop

     while ((SDA_PIN & _BV(SDA_BIT)) ) {
     
        // Read the dynamic mailbox register
        i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
        const byte mb_ctrl_dyn = b[0];


        if ( mb_ctrl_dyn & 0x04 ) {   // bit 2 = RF_MESSAGE_PUT indicates a message in the mailbox from RF

          // There is a message waiting for us! 

          // Read message len
          i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2007 , 0x0001 );
          const unsigned mb_len_dyn = b[0] + 1;    // Note that message length is register + 1 (you can't have a 0 len message)

          // Read out the message. This automatically clears is so a new message cen be sent. 
          i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2008 , mb_len_dyn );

          Serial.println("Message (hex):");

          for( unsigned i=0; i<mb_len_dyn; i++) {
            Serial.print( (unsigned) b[i] , 16 );
          }

          Serial.println();
                    
        }

        delay(100);   // Give RF some time to write! (i2c activity blocks NFC)
      
     }
  
    
  }


//
//
//  const byte zero_in_array[] = {0x00};  
//  const byte eight_in_array[] = {0x08};
//
//  const byte password_in_array[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};    // Default password
//  
//  i2cPresentPassord( password_in_array );
//  Serial.println("Presented password.");                                                    
//    
//
//  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2004 , 0x0001 );
//  Serial.print("I2C_SSO=1:");                                                      
//  Serial.println( b[0] , 16 );           
//
//                                            
//  i2cWrite(  one_in_array , ST25DV_ADDR_SYST_I2C , 0x000D , 0x0001 );
//  delay(5);
//  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x000D , 0x0001 );
//  Serial.print("MB_MODE=1:");                                                      
//  Serial.println( b[0] , 16 );                                                    
//
//
//  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
//  Serial.print("MB_CTRL=?:");
//  Serial.println( b[0] , 16 );   
//
//  i2cWrite(  one_in_array , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
//  delay(5);
//  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
//  Serial.print("MB_CTRL=1:");                                                      
//  Serial.println( b[0] , 16 );                                                            
//
//
//  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x000E , 0x0001 );
//  Serial.print("MB_WDG:");                                                      
//  Serial.println( b[0] , 16 );                                                            
//  i2cWrite(  zero_in_array , ST25DV_ADDR_SYST_I2C , 0x000E , 0x0001 );
//  delay(5);
//  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x000E , 0x0001 );
//  Serial.print("MB_WDG=0:");                                                      
//  Serial.println( b[0] , 16 );                                                            
//
//
//  const byte magic_message_in_array[] = { 0x62 , 0x63 };      
//  i2cWrite(  magic_message_in_array , ST25DV_ADDR_DATA_I2C , 0x2008 , 0x0002 );
//  delay(5);
//  Serial.println( "Wrote message into mailbox." );   
//  
//  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
//  Serial.print("MB_CTRL=?:");                                                        
//  Serial.println( b[0] , 16 );                                                    
//  
//  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2007 , 0x0001 );
//  Serial.print("MB_LEN=?:");                                                        
//  Serial.println( b[0] , 16 );                                                    
//                                          
//
//  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x17 , 0x001 ); 
//  Serial.print("IC_REF:");                                                      
//  Serial.println( b[0] , 16 );                                                    
//  
//
//  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x18 , 0x008 ); 
//  Serial.print("UUID:");                                                      
//  for( int i=0; i<0x08; i++ ) {
//          Serial.print( (unsigned) b[i] , 16 );        
//          Serial.print( ' ' );                                                    
//    }  
//  Serial.println();        
//
//
//  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2004 , 0x0001 );
//  Serial.print("I2C_SSO=1:");                                                      
//  Serial.println( b[0] , 16 );           
//  
//
//  
//  while (1);
  
//
//
//  if ( !i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x18 , 0x008 ) ) {
//
//    Serial.println("Read good...");  
//
//    for( int i=0; i<0x18; i++ ) {
//          Serial.print( i );              
//          Serial.print( ' ' );                  
//          Serial.print( (char) b[i] );        
//          Serial.print( ' ' );                            
//          Serial.println( (unsigned) b[i] , 16 );        
//                
//    }
//
//    
//  } else {
//
//    Serial.println("Read fail.");  
//
//  }
//  
  

}
