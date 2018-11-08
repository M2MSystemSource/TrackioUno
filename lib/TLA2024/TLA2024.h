#include <Arduino.h>
#include <Wire.h>

#pragma once
#define SINGLE true
#define CONT false

#define AIN0	4
#define AIN1	5
#define AIN2	6
#define AIN3	7

class TLA2024 {
 public:
  TLA2024();

  /*
    Initializes I2C bus
    returns:
       0 - adc responds with correct default conf
       1 - otherwise
  */
  int begin();

  // resets device to default configuration
  void reset();

  // restores device to last config
  void restore();

  float analogRead();

  /*
    Sets the Full Scale Range of the ADC

    gain:
      60 -> ± 6.144v
      40 -> ± 4.096
      20 -> ± 2.048 (default)
      10 -> ± 1.024
      5  -> ± 0.512
      2  -> ± 0.256
  */
  void setFSR(uint8_t gain);

  /*
    Configures the input signals to the ADC

    option:
      1 -> P = 0, N = 1 (TP1 - TP2)
      2 -> P = 0, N = 3 (TP1 - VGND)
      3 -> P = 1, N = 3 (TP2 - VGND)
      4 -> P = 0, N = GND
      5 -> P = 1, N = GND
      6 -> P = 2, N = GND
      7 -> P = 3, N = GND
  */
  void setMux(uint8_t option);

  /*
    Continous conversion (0) or single shot (1)
  */
  void setMode(bool mode);

  void setDR(uint8_t rate);

 private:
  uint8_t addr = 0x48;
  uint8_t conv_reg = 0x00;
  uint8_t conf_reg = 0x01;

  // this is default conf.
  uint16_t init_conf = 0x8583;
  uint16_t saved_conf = 0x8583;
  bool mode = SINGLE;

  union I2C_data {
    uint8_t packet[2];
    uint16_t value;
  } data;

  /*
    A generic read from mem_addr.
  */
  uint16_t read(uint8_t mem_addr);

  /*
    We only write to the configuration register.
    out_data is the 16 bits of conf_regs

    Should always return 2

    Saves data to current_conf
  */
  int write(uint16_t data);
};

TLA2024::TLA2024() {}

int TLA2024::begin() {
  Wire.begin();
  Wire.setClock(100000);
  uint16_t init = read(conf_reg);
  // make sure communication with device is working and that it is OK
  if ((init == init_conf) || 1) {
    return 0;
  } else {
    return 1;
  }
}

uint16_t TLA2024::read(uint8_t mem_addr) {
  Wire.beginTransmission(addr);
  Wire.write(mem_addr);
  Wire.endTransmission();
  delay(5);
  Wire.requestFrom(addr, 2);
  if (2 <= Wire.available()) {
    // bring in data
    data.packet[1] = Wire.read();
    data.packet[0] = Wire.read();
    uint16_t ret = data.value;
    data.value = 0;
    return ret;
  }
  return -1;
}

int TLA2024::write(uint16_t out_data) {
  int written = 0;
  // save conf
  saved_conf = out_data;
  // put our out_data into the I2C data union so we can send MSB and LSB
  data.value = out_data;
  Wire.beginTransmission(addr);
  Wire.write(conf_reg);
  written += Wire.write(data.packet[1]);
  written += Wire.write(data.packet[0]);
  Wire.endTransmission();
  data.value = 0;
  return written;
}

void TLA2024::reset(void) { write(init_conf); }

void TLA2024::restore(void) {
  uint16_t restore_conf = saved_conf & ~0x8000;
  write(restore_conf);
}


float TLA2024::analogRead(void) {

  // this only needs to run when in single shot.
  if (mode) {
    // write 1 to OS bit to start conv
    uint16_t current_conf = read(conf_reg);
    current_conf |= 0x8000;
    write(current_conf);
    // OS bit will be 0 until conv is done.
    do {
      delay(5);
    } while ((read(conf_reg) & 0x8000) == 0);
  }

  // get data from conv_reg
  uint16_t in_data = read(conv_reg);

  // shift out unused bits
  in_data >>= 4;

  // get sign and mask accordingly
  if (in_data & (1 << 11)) {
    // 11th bit is sign bit. if its set, set bits 15-12
    in_data |= 0xF000;
  } else {
    // not set, clear bits 15-12
    in_data &= ~0xF000;
  }

  // now store it as a signed 16 bit int.
  int16_t ret = in_data;

  // default Full Scale Range is -2.048V to 2.047V.
  // our 12bit 2's complement goes from -2048 to 2047 :)
  // return ret /1000.0;

  // return raw adc data
  return ret;
}


void TLA2024::setFSR(uint8_t gain) {
  // bring in conf reg
  uint16_t conf = read(conf_reg);
  // clear the PGA bits:
  conf &= ~0x0E00;

  switch (gain) {
    case 60:
      // PGA bits already cleared
      break;

    case 40:
      // set bit 9
      conf |= 0x0200;
      break;

    case 20:
      // set bit 10
      conf |= 0x0400;
      break;

    case 10:
      // set bit 10-9
      conf |= 0x0600;
      break;

    case 5:
      // set bit 11
      conf |= 0x0800;
      break;

    case 2:
      // set bit 11-9
      conf |= 0x0E00;
      break;
  }
  write(conf);
}

void TLA2024::setMux(uint8_t option) {
  // bring in conf reg
  uint16_t conf = read(conf_reg);
  // clear MUX bits
  conf &= ~0x7000;

  switch (option) {
    case 1:
      // bits already cleared
      break;

    case 2:
      // set bit 12
      conf |= 0x1000;
      break;

    case 3:
      // set bit 13
      conf |= 0x2000;
      break;

    case 4:
      // set bit 14
      conf |= 0x4000;
      break;

    case 5:
      // set bits 14 and 12
      conf |= 0x5000;
      break;

    case 6:
      // set bits 14, and 13
      conf |= 0x6000;
      break;

    case 7:
      // set bits 14, 13 and 12
      conf |= 0x7000;
      break;
  }
  write(conf);
}

void TLA2024::setMode(bool mode) {
  // bring in conf reg
  TLA2024::mode = mode;
  uint16_t conf = read(conf_reg);
  // clear MODE bit (8) (continous conv)
  conf &= ~(1 << 8);
  if (mode) {
    // single shot
    conf |= (1 << 8);
  }
  write(conf);
}

void TLA2024::setDR(uint8_t rate) {
  // bring in conf reg
  uint16_t conf = read(conf_reg);
  // set bits 7:5
  conf |= 0b111 << 5;
  write(conf);
}
