
#include <avr/io.h>
#include "i2c.h"
#include <util/delay.h>

#define CBI(x,b) (x&=~(1<<b))
#define SBI(x,b) (x|=(1<<b))
#define TBI(x,b) ((x&(1<<b))!=0)


// Assumes OUT bits are still at startup default of 0

// SDA allways either (1) pulled high, or (2) driven low. 
// Assumes there is ~5K external pull-up on here if you want to go fast.

// SCL always driven. Idle state is HIGH.

// Drive SDA low
static inline void sda_low(void) {
    CBI( SDA_PORT , SDA_BIT );
    SBI( SDA_DDR  , SDA_BIT );
}

// Pull SDA high
static inline void sda_high(void) {
    CBI( SDA_DDR  , SDA_BIT );    
    SBI( SDA_PORT , SDA_BIT );    
}

static inline uint8_t sda_read() {
  return ( TBI( SDA_PIN , SDA_BIT) );
}

// Drive high
static inline void scl_enable(void) {
    SBI( SCL_DDR, SCL_BIT );  
}

static inline void scl_high(void) {
    SBI( SCL_PORT, SCL_BIT );  
}

static inline void scl_low(void) {
    CBI( SCL_PORT, SCL_BIT );  
}


// Leaves SDA pulled high, SCL driven high

void i2c_init() {
  scl_enable();
  scl_high();
  sda_high();
  
}



// Write a byte out to the slave and look for ACK bit
// Assumes SCL low, SDA doesn't matter

// Returns 0=success, SDA high (ACK bit released), SCL low .

uint8_t i2c_write( uint8_t data ) {

    for( uint8_t bitMask=0b10000000; bitMask !=0; bitMask>>=1 ) {

        // setup data bit

        if ( data & bitMask) {
            sda_high();
        } else {
            sda_low();
        }

        // clock it out

        _delay_us(BIT_TIME_US);

        scl_high();           // Clock in the next address bit

        _delay_us(BIT_TIME_US);

        scl_low();

    }

    // The device acknowledges the address by driving SDIO
    // low after the next falling SCLK edge, for 1 cycle.

    sda_high();            // Pull SDA high so we can see if the slave is driving low
    _delay_us(BIT_TIME_US);     // Not needed, but so we can see what is happening on the scope    
    scl_high();

    uint8_t ret = sda_read();   // slave should be driving low now

    _delay_us(BIT_TIME_US);     // Not needed, but so we can see what is happening on the scope    

    scl_low();            // Slave release

    return(ret);

}



// Assumes SCL and SDA are off
// Returns with SCL low, SDA low
// Returns 0 is ACK

uint8_t i2c_start( uint8_t slave ) {

    scl_high();
    sda_high();    
    
    _delay_us(BIT_TIME_US);         // Make sure we have been in idle at least long enough to see the falling SDA

    // Data transfer is always initiated by a Bus Master device. A high to low transition on the SDA line, while
    // SCL is high, is defined to be a START condition or a repeated start condition.

    sda_low();

    _delay_us(BIT_TIME_US);

    scl_low();    

    return i2c_write( slave ); 

}


// Assumes SCL=low, SDA=?
// Returns with SCL low, SDA low

uint8_t i2c_restart( uint8_t slave ) {
    _delay_us(BIT_TIME_US);     // Give time for slave to release SDA high
    scl_high(); 
    return i2c_start( slave );
}

// Assumes on entry SCL=low, SDA low
// Returns with bus idle, SCL high SDA high (idle)

void i2c_stop(void) {

    // Data transfer ends with the STOP condition
    // (rising edge of SDIO while SCLK is high).

    // Q: Is this Really needed? Can we just do repeat starts and save this code? Spec is vague if address is reset on start.
    // A: Yes, we need this because it is possible that the FM_IC is holding the SDA line low
    // wait for us to clock out the MSB of the next byte!
    
    _delay_us(BIT_TIME_US);     // Give is a moment to stabilize in case
    scl_low();
    sda_low();
    _delay_us(BIT_TIME_US);     // Give is a moment to stabilize in case
    scl_high();
    _delay_us(BIT_TIME_US);
    sda_high();            // SDA low to high while SCLK is high is a STOP
}

// Returns byte read.
// Returns with SCL=high SDA=LOW

uint8_t i2c_read( uint8_t lastFlag ) {
  
  uint8_t data=0;
  
   for( uint8_t bitMask=0b10000000; bitMask !=0; bitMask>>=1 ) {
  
      // Clock in the data bits

      _delay_us(BIT_TIME_US);
          
      scl_high();           // Clock in thedata bit
  
      _delay_us(BIT_TIME_US);

      // Note here we sample at the last possibible moment after the clock has been
      // high for a full bit time. This gives the line max time to go back up when released. 
 
      if ( sda_read() ) {
  
          data |= bitMask;
  
      }

        
      scl_low();
  
  }

  // ACK

  if (lastFlag) {    
    sda_high();       // NAK on last byte of sequence read
  } else {    
    sda_low();
  } 

  _delay_us(BIT_TIME_US);

  scl_high(); 
  
  return data; 

}
