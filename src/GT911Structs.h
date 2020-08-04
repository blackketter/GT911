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

struct GTInfo {
  // 0x8140-0x814A
  char productId[4];
  uint16_t fwId;
  uint16_t xResolution;
  uint16_t yResolution;
  uint8_t vendorId;
};

struct GTPoint {
  // 0x814F-0x8156, ... 0x8176 (5 points)
  uint8_t trackId;
  uint16_t x;
  uint16_t y;
  uint16_t area;
  uint8_t reserved;
};

struct GTLevelConfig {
  uint8_t touch; // Threshold of touch grow out of nothing
  uint8_t leave; // Threshold of touch decrease to nothing
};

struct GTStylusConfig {
  uint8_t txGain;
  uint8_t rxGain;
  uint8_t dumpShift;
  GTLevelConfig level;
  uint8_t control;     //Pen mode escape time out period (Unit: Sec)
};

struct GTFreqHoppingConfig {
  uint16_t hoppingBitFreq;
  uint8_t hoppingFactor;
};

struct GTKeyConfig {
  // Key position: 0-255 valid
  // 0 means no touch, it means independent touch key when 4 of the keys are 8 multiples
  uint8_t pos1;
  uint8_t pos2;
  uint8_t pos3;
  uint8_t pos4;
  uint8_t area;
  GTLevelConfig level;
  uint8_t sens12;
  uint8_t sens34;
  uint8_t restrain;
};

struct GTConfig {
  // start at 0x8047
  uint8_t configVersion;
  uint16_t xResolution;
  uint16_t yResolution;
  // 0x804C
  uint8_t touchNumber;   // 3:0 Touch No.: 1~10

  // 7:6 Reserved, 5:4 Stretch rank, 3 X2Y, 2 Sito
  // 1:0 INT trig method: 00-rising, 01-falling, 02-low level, 03-high level enquiry
  uint8_t moduleSwitch1;
  uint8_t moduleSwitch2;  // bit0 TouchKey
  uint8_t shakeCount;     // 3:0 Finger shake count
  // 0x8050
  // 7:6 First filter, 5:0 Normal filter (filtering value of original coordinate window, coefficiency is 1)
  uint8_t filter;
  uint8_t largeTouch;
  uint8_t noiseReduction;
  GTLevelConfig screenLevel;

  uint8_t lowPowerControl;  // Time to low power consumption (0~15s)
  uint8_t refreshRate;      // Coordinate report rate (Cycle: 5+N ms)
  uint8_t xThreshold; //res
  uint8_t yThreshold; //res
  uint8_t xSpeedLimit; //res
  uint8_t ySpeedLimit; //res
  uint8_t vSpace; // 4bit top/bottom  (coefficient 32)
  uint8_t hSpace; // 4bit left/right
  //0x805D-0x8061
  uint8_t stretchRate; //Level of weak stretch (Strtch X/16 Pitch)
  uint8_t stretchR0;   // Interval 1 coefficient
  uint8_t stretchR1;   // Interval 2 coefficient
  uint8_t stretchR2;   // Interval 3 coefficient
  uint8_t stretchRM;   // All intervals base number

  uint8_t drvGroupANum;
  uint8_t drvGroupBNum;
  uint8_t sensorNum;
  uint8_t freqAFactor;
  uint8_t freqBFactor;
  // 0x8067
  uint16_t pannelBitFreq;      //Baseband of Driver group A\B (1526HZ<baseband<14600Hz)
  uint16_t pannelSensorTime;  //res
  uint8_t pannelTxGain;
  uint8_t pannelRxGain;
  uint8_t pannelDumpShift;
  uint8_t drvFrameControl;
  // 0x806F - 0x8071
  uint8_t NC_2[3];
  GTStylusConfig stylusConfig;
  // 0x8078-0x8079
  uint8_t NC_3[2];
  uint8_t freqHoppingStart; // Frequency hopping start frequency (Unit: 2KHz, 50 means 100KHz )
  uint8_t freqHoppingEnd;   // Frequency hopping stop frequency (Unit: 2KHz, 150 means 300KHz )
  uint8_t noiseDetectTims;
  uint8_t hoppingFlag;
  uint8_t hoppingThreshold;

  uint8_t noiseThreshold;
  uint8_t NC_4[2];
  // 0x8082
  GTFreqHoppingConfig hoppingSegments[5];
  // 0x8091
  uint8_t NC_5[2];
  GTKeyConfig keys;


};

