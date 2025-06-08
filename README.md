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

# Data Output

```json
{
  "sensor_data": {
    "device_001": {
      "2025-06-07T08:37:00": {
        "bme680": {
          "altitude": 42.79875,
          "gas_resistance": 127.183,
          "humidity": 58.33585,
          "pressure": 1008.11,
          "temperature": 39.35321
        },
        "hx711": {
          "weight": -21873.32
        },
        "mics4514": {
          "co": 145.3055,
          "co_voltage": 0.344103,
          "no2": 33,
          "no2_voltage": 3.3
        },
        "mq6": {
          "butane": 0,
          "lpg": 8,
          "methane": 0.009768,
          "voltage": 0.009768
        },
        "mq7": {
          "co": 258.4604,
          "voltage": 0.523004
        },
        "pms7003": {
          "particles_0_3um": 755,
          "particles_0_5um": 140,
          "particles_10_0um": 0,
          "particles_1_0um": 16,
          "particles_2_5um": 8,
          "particles_5_0um": 5,
          "pm10": 14,
          "pm1_0": 23,
          "pm2_5": 27
        }
      },
      "2025-06-07T08:37:31": {
        "bme680": {
          "altitude": 42.96587,
          "gas_resistance": 124.258,
          "humidity": 57.57693,
          "pressure": 1008.1,
          "temperature": 39.41778
        },
        "hx711": {
          "weight": -21865.83
        },
        "mics4514": {
          "co": 130.8976,
          "co_voltage": 0.381978,
          "no2": 33,
          "no2_voltage": 3.3
        },
        "mq6": {
          "butane": 0,
          "lpg": 8.4,
          "methane": 0.010256,
          "voltage": 0.010256
        },
        "mq7": {
          "co": 263.4556,
          "voltage": 0.528645
        },
        "pms7003": {
          "particles_0_3um": 652,
          "particles_0_5um": 132,
          "particles_10_0um": 0,
          "particles_1_0um": 18,
          "particles_2_5um": 8,
          "particles_5_0um": 4,
          "pm10": 13,
          "pm1_0": 22,
          "pm2_5": 26
        }
      },
      "2025-06-07T08:38:01": {
        "bme680": {
          "altitude": 43.21631,
          "gas_resistance": 127.183,
          "humidity": 57.46362,
          "pressure": 1008.07,
          "temperature": 39.47667
        },
        "hx711": {
          "weight": -21876.93
        },
        "mics4514": {
          "co": 123.3508,
          "co_voltage": 0.405348,
          "no2": 33,
          "no2_voltage": 3.3
        },
        "mq6": {
          "butane": 0,
          "lpg": 2.2,
          "methane": 0.002686,
          "voltage": 0.002686
        },
        "mq7": {
          "co": 291.4938,
          "voltage": 0.559267
        },
        "pms7003": {
          "particles_0_3um": 739,
          "particles_0_5um": 131,
          "particles_10_0um": 0,
          "particles_1_0um": 17,
          "particles_2_5um": 6,
          "particles_5_0um": 4,
          "pm10": 13,
          "pm1_0": 22,
          "pm2_5": 26
        }
      },
      "2025-06-07T08:38:31": {
        "bme680": {
          "altitude": 43.21631,
          "gas_resistance": 138.888,
          "humidity": 57.03908,
          "pressure": 1008.07,
          "temperature": 39.43951
        },
        "hx711": {
          "weight": -21821.15
        },
        "mics4514": {
          "co": 125.0916,
          "co_voltage": 0.399707,
          "no2": 33,
          "no2_voltage": 3.3
        },
        "mq6": {
          "butane": 0,
          "lpg": 7.4,
          "methane": 0.009035,
          "voltage": 0.009035
        },
        "mq7": {
          "co": 256.3356,
          "voltage": 0.520586
        },
        "pms7003": {
          "particles_0_3um": 709,
          "particles_0_5um": 113,
          "particles_10_0um": 0,
          "particles_1_0um": 15,
          "particles_2_5um": 9,
          "particles_5_0um": 6,
          "pm10": 14,
          "pm1_0": 21,
          "pm2_5": 25
        }
      },
      "2025-06-07T08:39:00": {
        "bme680": {
          "altitude": 43.04918,
          "gas_resistance": 131.682,
          "humidity": 57.96966,
          "pressure": 1008.08,
          "temperature": 39.24645
        },
        "hx711": {
          "weight": -21866.18
        },
        "mics4514": {
          "co": 121.4197,
          "co_voltage": 0.411795,
          "no2": 33,
          "no2_voltage": 3.3
        },
        "mq6": {
          "butane": 0,
          "lpg": 6.8,
          "methane": 0.008303,
          "voltage": 0.008303
        },
        "mq7": {
          "co": 262.0231,
          "voltage": 0.527033
        },
        "pms7003": {
          "particles_0_3um": 678,
          "particles_0_5um": 103,
          "particles_10_0um": 0,
          "particles_1_0um": 11,
          "particles_2_5um": 5,
          "particles_5_0um": 3,
          "pm10": 14,
          "pm1_0": 22,
          "pm2_5": 25
        }
      }
    }
  }
}
```
