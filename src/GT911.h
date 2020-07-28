#ifndef _GT911_H_
#define _GT911_H_

#include "GT911Structs.h"

#define GT911_OK   0

// 0x28/0x29 (0x14 7bit)
#define GT911_I2C_ADDR_28  0x14
// 0xBA/0xBB (0x5D 7bit)
#define GT911_I2C_ADDR_BA  0x5D

#define GT911_MAX_HEIGHT   4096
#define GT911_MAX_WIDTH    4096
#define GT911_INT_TRIGGER    1
#define GT911_CONTACT_SIZE   8
#define GT911_MAX_CONTACTS   10

#define GT911_CONFIG_MAX_LENGTH  240
#define GT911_CONFIG_911_LENGTH  186
#define GT911_CONFIG_967_LENGTH  228

/* Register defines */
#define GT_REG_CMD  0x8040

#define GT_REG_CFG  0x8047
#define GT_REG_DATA 0x8140


// Write only registers
#define GT911_REG_COMMAND        0x8040
#define GT911_REG_LED_CONTROL    0x8041
#define GT911_REG_PROXIMITY_EN   0x8042

// Read/write registers
// The version number of the configuration file
#define GT911_REG_CONFIG_DATA  0x8047
// X output maximum value (LSB 2 bytes)
#define GT911_REG_MAX_X        0x8048
// Y output maximum value (LSB 2 bytes)
#define GT911_REG_MAX_Y        0x804A
// Maximum number of output contacts: 1~5 (4 bit value 3:0, 7:4 is reserved)
#define GT911_REG_MAX_TOUCH    0x804C

// Module switch 1
// 7:6 Reserved, 5:4 Stretch rank, 3 X2Y, 2 SITO (Single sided ITO touch screen), 1:0 INT Trigger mode */
#define GT911_REG_MOD_SW1      0x804D
// Module switch 2
// 7:1 Reserved, 0 Touch key */
#define GT911_REG_MOD_SW2      0x804E

// Number of debuffs fingers press/release
#define GT911_REG_SHAKE_CNT    0x804F

// ReadOnly registers (device and coordinates info)
// Product ID (LSB 4 bytes, GT9110: 0x06 0x00 0x00 0x09)
#define GT911_REG_ID           0x8140
// Firmware version (LSB 2 bytes)
#define GT911_REG_FW_VER       0x8144

// Current output X resolution (LSB 2 bytes)
#define GT911_READ_X_RES       0x8146
// Current output Y resolution (LSB 2 bytes)
#define GT911_READ_Y_RES       0x8148
// Module vendor ID
#define GT911_READ_VENDOR_ID   0x814A

#define GT911_READ_COORD_ADDR  0x814E

/* Commands for REG_COMMAND */
//0: read coordinate state
#define GT911_CMD_READ         0x00
// 1: difference value original value
#define GT911_CMD_DIFFVAL      0x01
// 2: software reset
#define GT911_CMD_SOFTRESET    0x02
// 3: Baseline update
#define GT911_CMD_BASEUPDATE   0x03
// 4: Benchmark calibration
#define GT911_CMD_CALIBRATE    0x04
// 5: Off screen (send other invalid)
#define GT911_CMD_SCREEN_OFF   0x05

/* When data needs to be sent, the host sends command 0x21 to GT9x,
 * enabling GT911 to enter "Approach mode" and work as a transmitting terminal */
#define GT911_CMD_HOTKNOT_TX   0x21

#define RESOLUTION_LOC    1
#define MAX_CONTACTS_LOC  5
#define TRIGGER_LOC 6

class GT911 {
  public:
    uint8_t i2cAddr;
    struct GTConfig config;
    struct GTInfo info;
    uint8_t points[GT911_MAX_CONTACTS*GT911_CONTACT_SIZE]; //points buffer

    GT911();

    void setHandler(void (*handler)(int8_t, GTPoint*));

    bool begin(uint8_t interruptPin, uint8_t resetPin, uint8_t addr=GT911_I2C_ADDR_BA);
    bool reset();
    uint8_t test();
    void loop();

    uint8_t write(uint16_t reg, uint8_t *buf, size_t len);
    uint8_t write(uint16_t reg, uint8_t value);
    uint8_t read(uint16_t reg, uint8_t *buf, size_t len);

    uint8_t calcChecksum(uint8_t* buf, uint8_t len);
    uint8_t readChecksum();

    uint8_t fwResolution(uint16_t maxX, uint16_t maxY);
    
    GTConfig* readConfig();
    GTInfo* readInfo();

    uint8_t productID(char *buf);

    int16_t readInput(uint8_t *data);

  //--- Private routines ---
  private:
    uint8_t intPin, rstPin;
    void (*touchHandler)(int8_t, GTPoint*);

    void debugPin(uint8_t level);
    void armIRQ();
    void onIRQ();

    //--- utils ---
    void usSleep(uint16_t microseconds);
    void msSleep(uint16_t milliseconds);

    void pinIn(uint8_t pin);
    void pinOut(uint8_t pin);
    void pinSet(uint8_t pin, uint8_t level);

    // Used with pulled-up lines, set pin mode to out, write LOW
    void pinHold(uint8_t pin);

    // Check pin level
    bool pinCheck(uint8_t pin, uint8_t level);

    void i2cStart(uint16_t reg);
    void i2cRestart();
    uint8_t i2cStop();
};

#endif
