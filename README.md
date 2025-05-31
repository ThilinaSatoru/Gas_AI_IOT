# Gas Sensors

```c++
// BME680 (SPI)
#define BME_CS           5     // GPIO5 for - (CS)
#define BME_MOSI         23    // VSPI MOSI - (SDA)
#define BME_MISO         19    // VSPI MISO - (SD0)
#define BME_SCK          18    // VSPI SCK  - (SCL)

// PMS7003 (UART Serial2)
#define PMS_RX_PIN       16    // RX2 (connects to PMS7003 TX)
#define PMS_TX_PIN       17    // TX2 (connects to PMS7003 RX)

// Gas Sensors
#define MQ6_ANALOG_PIN   34    // LPG sensor
#define MQ7_ANALOG_PIN   35    // CO sensor

#define MICS4514_CO_PIN  36    // CO sensor
#define MICS4514_NO2_PIN 39    // NO2 sensor (VN)

// HX711 Load Cell
#define HX711_DOUT_PIN   25    // Digital input
#define HX711_SCK_PIN    26    // Digital output
```
