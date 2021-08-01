

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
  Wire.begin();   // Join as master

  Wire.setClock( 10000 );

  //st25dv.ST25DV_SelectI2cSpeed(0);
  
}




byte i2cWrite( const byte *pData, const uint8_t devAddr, const uint16_t TarAddr,  uint16_t len) {

  byte addr = devAddr >> 1;         // Loose bottom r/w bit

  Wire.beginTransmission(addr);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(TarAddr >> 8);           //  The command byte, sets pointer to register with address of 0x32
  Wire.write(TarAddr & 0xFF);         //  The command byte, sets pointer to register with address of 0x32

  Wire.write(pData,len);
  
  byte ret = Wire.endTransmission(true);

  if (ret) {

    Serial.print( ret ); 
    Serial.println( "=W endTransmission fail." );                  
    
    //return 1;
  }

  return ret;

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


byte i2cRead( byte *pData, const uint8_t devAddr, const uint16_t TarAddr,  uint16_t len) {

  byte addr = devAddr >> 1;

  Wire.beginTransmission(addr);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(TarAddr >> 8);           //  The command byte, sets pointer to register with address of 0x32
  Wire.write(TarAddr & 0xFF);         //  The command byte, sets pointer to register with address of 0x32
  
  byte ret = Wire.endTransmission(false);

  if (ret) {

    Serial.print( ret ); 
    Serial.println( "=W endTransmission fail." );                  
    
    return 1;
  }
  
  Wire.requestFrom( addr ,  len  ); // Tell slave we need to read 1byte from the current register
  
  while ( len && Wire.available()) {
    byte c = Wire.read();      // read that byte into 'slaveByte2' variable
    *pData = c;
    pData++;
    len--;
  }

  ret = Wire.endTransmission();

  if (ret) {

    Serial.print( ret ); 
    Serial.println( "=R endTransmission fail." );                  
    
    //return 1;
  }


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



  const byte password_in_array[] = {0x00,0x00,0x00,0x00,0x00,0x00};    // Default password
  
  i2cPresentPassord( password_in_array );
  Serial.println("Presented password.");                                                    
    

  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2004 , 0x0001 );
  Serial.print("I2C_SSO=1:");                                                      
  Serial.println( b[0] , 16 );           

                                            
  i2cWrite(  one_in_array , ST25DV_ADDR_DATA_I2C , 0x000D , 0x0001 );
  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x000D , 0x0001 );
  Serial.print("MB_MODE=1:");                                                      
  Serial.println( b[0] , 16 );                                                    


  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  Serial.print("MB_CTRL=?:");                                                        
  Serial.println( b[0] , 16 );   

  i2cWrite(  one_in_array , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  Serial.print("MB_CTRL=1:");                                                      
  Serial.println( b[0] , 16 );                                                            


  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x000D , 0x0001 );
  Serial.print("MB_WDG:");                                                      
  Serial.println( b[0] , 16 );                                                            
  i2cWrite(  zero_in_array , ST25DV_ADDR_SYST_I2C , 0x000D , 0x0001 );
  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x000D , 0x0001 );
  Serial.print("MB_WDG=0:");                                                      
  Serial.println( b[0] , 16 );                                                            



  const byte magic_message_in_array[] = { 0x62 , 0x63 };      
  i2cWrite(  magic_message_in_array , ST25DV_ADDR_DATA_I2C , 0x2008 , 0x0002 );
  Serial.println( "Wroite message into mailbox." );   
  
  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  Serial.print("MB_CTRL=?:");                                                        
  Serial.println( b[0] , 16 );                                                    
  


  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2007 , 0x0001 );
  Serial.print("MB_LEN=?:");                                                        
  Serial.println( b[0] , 16 );                                                    

  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );
  Serial.print("MB_CTRL=?:");                                                        
  Serial.println( b[0] , 16 );                                                    


  i2cWrite(  magic_message_in_array , ST25DV_ADDR_DATA_I2C , 0x2008 , 0x0002 );
  Serial.println( "Wroite message into mailbox." );                                                    
  
  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2007 , 0x0001 );
  Serial.print("MB_LEN=2:");                                                        
  Serial.println( b[0] , 16 );                                                    
                                           


  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2000 , 0x001 ); 
  Serial.print("GPIO_CONTRL:");
  Serial.println( b[0] , 16 );                                                    

  Serial.println("Writing 0x08 to GPIOI_CTRL.");  
  i2cWrite(  eight_in_array , ST25DV_ADDR_DATA_I2C , 0x2000 , 0x0001 );
  
  
  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2000 , 0x001 ); 
  Serial.print("GPIO_CONTRL:");
  Serial.println( b[0] , 16 );                                                    

  const byte eightyeight_in_array[] = {0x88};


  Serial.println("Writing 0x88 to GPIOI_CTRL.");  
  i2cWrite(  eightyeight_in_array , ST25DV_ADDR_DATA_I2C , 0x2000 , 0x0001 );
  
  
  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2000 , 0x001 ); 
  Serial.print("GPIO_CONTRL:");
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


  i2cRead( b , ST25DV_ADDR_SYST_I2C , 0x0d , 0x001 ); 


  Serial.print("MB_MODE:");                                                    
  
  Serial.println( b[0] , 16 );            



  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2004 , 0x0001 );

  Serial.print("I2C_SSO:");                                                    
  
  Serial.println( b[0] , 16 );                                                    



  
  i2cPresentPassord( password_in_array );
  

  Serial.println("Presented password.");                                                    
    

  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2004 , 0x0001 );

  Serial.print("I2C_SSO:");                                                    
  
  Serial.println( b[0] , 16 );                                                    

                                          
                                            
  i2cWrite(  one_in_array , ST25DV_ADDR_DATA_I2C , 0x000D , 0x0001 );

  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x000D , 0x0001 );

  Serial.print("MB_MODE=1:");                                                    
  
  Serial.println( b[0] , 16 );                                                    


  i2cWrite( zero_in_array, ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );

  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );


  Serial.print("MB=0:");                                                    
  
  Serial.println( b[0] , 16 );   
                                                   
  
  i2cWrite( one_in_array , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );

  i2cRead( b , ST25DV_ADDR_DATA_I2C , 0x2006 , 0x0001 );

  Serial.print("MB=1:");                                                    

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
