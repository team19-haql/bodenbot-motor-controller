// #include <Encoder.h>
#include <Wire.h>
#include <SparkFunSX1509.h>
#include <time.h>
#include <Adafruit_NeoPixel.h>

// class Motor {
// public:
//   Motor(uint8_t pin_a, uint8_t pin_b, uint8_t dir_pin, uint8_t pwm_pin) :
//     _enc(pin_a, pin_b), _dir_pin(dir_pin), _pwm_pin(pwm_pin) {}

//   void setup(SX1509* io) {
//     io->pinMode(_dir_pin, OUTPUT);
//     io->pinMode(_pwm_pin, ANALOG_OUTPUT);
//   }

//   void update(SX1509* io) {
//     // read encoder


//     // PID

//     // write to pins
//     uint8_t pwm= 100;
//     uint8_t dir = 15;
//     io->analogWrite(_pwm_pin, pwm);
//     io->digitalWrite(_dir_pin, dir);
//   }
// private:
//   Encoder _enc;
//   uint8_t _dir_pin;
//   uint8_t _pwm_pin;
// };

// #define MOTORS 6

// Motor motors[] = {
//   Motor(0, 1, 2, 0), // left front
//   Motor(2, 3, 3, 0), // left middle
//   Motor(4, 5, 4, 0), // left rear
//   Motor(6, 7, 5, 1), // right front
//   Motor(8, 9, 6, 1), // right middle
//   Motor(10, 11, 7, 1), // right rear
// };

MbedI2C i2c(0, 1);

void scan() {
  int nDevices = 0;
  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device did acknowledge to the address.
    i2c.beginTransmission(address);
    byte error = i2c.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
}

const byte SX1509_ADDRESS = 0x3E; // SX1509 I2C address
SX1509 io;                        // Create an SX1509 object to be used throughout

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  while (!Serial) ;

  Serial.println("Startup");

  i2c.begin();


  scan();

  if (io.begin(SX1509_ADDRESS, i2c) == false) {
    Serial.println("Failed to connect to SX1509");
    while (1);
  } else {
    Serial.println("Connected to thing");
  }

  io.pinMode(0, ANALOG_OUTPUT);

}


// long last_time = get_ms_count();

void loop() {
  Serial.println("hi");
  // put your main code here, to run repeatedly:

  // 500 is the time between updates
  for (int i = 255; i >= 0; i--) {
    io.analogWrite(0, i);
    delay(20);
  }
  // delay(500 - (get_ms_count() - last_time));
}
