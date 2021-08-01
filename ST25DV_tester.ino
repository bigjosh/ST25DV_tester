


#define SCL_PIN 5
#define SCL_PORT PORTC
#define SDA_PIN 4
#define SDA_PORT PORTC

#include "SoftI2CMaster.h"


/** @brief I2C address to be used for ST25DV Data accesses. */
#define ST25DV_ADDR_DATA_I2C                 0xA6
/** @brief I2C address to be used for ST25DV System accesses. */
#define ST25DV_ADDR_SYST_I2C                 0xAE

void setup() {
  
  Serial.begin(1000000);

  delay(100);
  Serial.println("\r\nNFC JOSH Start...");  

  // put your setup code here, to run once:

  i2c_init();
    
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

  
  i2c_rep_start(addr|I2C_READ);
    
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

  byte b[256];

  const byte zero_in_array[] = {0x00};
  const byte one_in_array[] = {0x01};
  const byte eight_in_array[] = {0x08};



  const byte password_in_array[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};    // Default password
  
  i2cPresentPassord( password_in_array );
  Serial.println("Presented password.");                                                    
    

  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2004 , 0x0001 );
  Serial.print("I2C_SSO=1:");                                                      
  Serial.println( b[0] , 16 );           

                                            
  i2cWrite(  one_in_array , ST25DV_ADDR_SYST_I2C , 0x000D , 0x0001 );
  delay(5);
  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x000D , 0x0001 );
  Serial.print("MB_MODE=1:");                                                      
  Serial.println( b[0] , 16 );                                                    


  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  Serial.print("MB_CTRL=?:");
  Serial.println( b[0] , 16 );   

  i2cWrite(  one_in_array , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  delay(5);
  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  Serial.print("MB_CTRL=1:");                                                      
  Serial.println( b[0] , 16 );                                                            


  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x000E , 0x0001 );
  Serial.print("MB_WDG:");                                                      
  Serial.println( b[0] , 16 );                                                            
  i2cWrite(  zero_in_array , ST25DV_ADDR_SYST_I2C , 0x000E , 0x0001 );
  delay(5);
  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x000E , 0x0001 );
  Serial.print("MB_WDG=0:");                                                      
  Serial.println( b[0] , 16 );                                                            


  const byte magic_message_in_array[] = { 0x62 , 0x63 };      
  i2cWrite(  magic_message_in_array , ST25DV_ADDR_DATA_I2C , 0x2008 , 0x0002 );
  delay(5);
  Serial.println( "Wrote message into mailbox." );   
  
  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  Serial.print("MB_CTRL=?:");                                                        
  Serial.println( b[0] , 16 );                                                    
  
  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2007 , 0x0001 );
  Serial.print("MB_LEN=?:");                                                        
  Serial.println( b[0] , 16 );                                                    
                                          

  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x17 , 0x001 ); 
  Serial.print("IC_REF:");                                                      
  Serial.println( b[0] , 16 );                                                    
  

  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x18 , 0x008 ); 
  Serial.print("UUID:");                                                      
  for( int i=0; i<0x08; i++ ) {
          Serial.print( (unsigned) b[i] , 16 );        
          Serial.print( ' ' );                                                    
    }  
  Serial.println();        


  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2004 , 0x0001 );
  Serial.print("I2C_SSO=1:");                                                      
  Serial.println( b[0] , 16 );           
  

  
  while (1);
  
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
  
delay(1000);

}
