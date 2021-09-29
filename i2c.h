
#define BIT_TIME_US     (1)          // How long should should we wait between bit transitions?

// Sorry these are defined here. We can not use built-in digital___() functions becuase they are too slow. 
// Could have used FastGPIO library, but then would have to
// explain how to install that. :/

// Arduino UNO pin A3
#define SCL_BIT   3
#define SCL_PORT  PORTC
#define SCL_DDR   DDRC

// Arduino UNO pin A2
#define SDA_BIT   2
#define SDA_PORT  PORTC
#define SDA_DDR   DDRC
#define SDA_PIN   PINC


// Leaves SDA pulled high, SCL driven high
void i2c_init();

// Write a byte out to the slave and look for ACK bit
// Assumes SCL low, SDA doesn't matter

// Returns 0=success, SDA high (ACK bit released), SCL low .

uint8_t i2c_write( uint8_t data );


// Assumes SCL high, SDA high (idle)
// Returns with SCL low, SDA low
// Returns 0 is ACK

uint8_t i2c_start( uint8_t slave );


// Assumes SCL=low, SDA=high
// Returns with SCL low, SDA low

uint8_t i2c_restart( uint8_t slave );

// Assumes on entry SCL low 
// Returns with bus idle, SCL high SDA high (idle)

void i2c_stop(void);

// Returns byte read.
// Returns with SCL=high SDA=LOW

uint8_t i2c_read( uint8_t lastFlag);

// OR these with the device address 

#define I2C_WRITE 0x00
#define I2C_READ  0x01
