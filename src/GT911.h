#ifndef _GT911_H_
#define _GT911_H_

#include "GT911Structs.h"

class TS_Point {
 public:
  TS_Point(void);
  TS_Point(int16_t x, int16_t y, int16_t z, int16_t id);

  bool operator==(TS_Point);
  bool operator!=(TS_Point);

  int16_t x; /*!< X coordinate */
  int16_t y; /*!< Y coordinate */
  int16_t z; /*!< Z coordinate (often used for pressure) */
  int16_t id; /* tracking id for multiple touches */
};

class GT911 {
  public:

    GT911();

    // note: GTPoint uint16_t are always little endian
    void setHandler(void (*handler)(int8_t, GTPoint*));

    bool begin(uint8_t interruptPin, uint8_t resetPin, uint8_t addr=GT911_I2C_ADDR_BA);

    TS_Point getPoint(uint8_t n = 0);
    uint8_t touched();

    bool reset();
    uint8_t test();
    void loop();

    uint8_t i2cAddr;

  //--- Private routines ---
  private:
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

    struct GTConfig config;
    struct GTInfo info;
    int8_t contacts;
    GTPoint points[GT911_MAX_CONTACTS]; //points buffer
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
