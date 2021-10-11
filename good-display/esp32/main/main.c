/**
 * Driving good-display epaper displays using an esp32.
 * Code has been adapted from the sample Arduino code from good-display to run
 * directly as an ESP-IDF program.
 *
 * This has been tested on a GDEM213U23 display.
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_task_wdt.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "demo.h"

// Note: Change this to the pins you are using.
#define BUSY_Pin 16  // READ   (BUSY)
#define RES_Pin  17  // WRITE  (RESET/RST)
#define DC_Pin   18  // WRITE  (DATA/COMMAND)
#define CS_Pin   23  // WRITE  (CHIP SELECT)
#define SCK_Pin  22  // WRITE  (CLOCK)
#define SDI_Pin  21  // WRITE  (MOSI)

// See "Command Table" in the provided specificaton PDF.
#define CMD_DRIVER_OUTPUT_CTRL       0x01
#define CMD_GATE_DRIVING_VOLT_CTRL   0x03
#define CMD_SOURCE_DRIVING_VOLT_CTRL 0x04
#define CMD_RED_DISPLAY_CTRL         0x05
#define CMD_DEEP_SLEEP               0x10
#define CMD_DATA_ENTRY_MODE          0x11
#define CMD_SW_RESET                 0x12
#define CMD_MASTER_ACTIVATION        0x20
#define CMD_DISP_UPDATE_CTRL         0x22
#define CMD_WRITE_RAM_1              0x24
#define CMD_WRITE_RAM_RED            0x26
#define CMD_WRITE_VCOM_REGISTER      0x2C
#define CMD_WRITE_LUT_REG            0x32
#define CMD_SET_DUMMY_LINE_PERIOD    0x3A
#define CMD_SET_GATE_LINE_WIDTH      0x3B
#define CMD_BORDER_CONTROL           0x3C
#define CMD_SET_RAM_X_ADDR_POS       0x44
#define CMD_SET_RAM_Y_ADDR_POS       0x45
#define CMD_SET_RAM_X_ADDR           0x4E
#define CMD_SET_RAM_Y_ADDR           0x4F
#define CMD_RED_DISPLAY_CONTROL      0x75

#define PIN_LOW  false
#define PIN_HIGH true

#define  MONO 1
#define  RED  2
#define ALLSCREEN_BYTES   4000

const char* TAG = "EPAP";

void delay_millis(int millis) {
  esp_task_wdt_reset();
  vTaskDelay(millis / portTICK_PERIOD_MS);
}

void set_pin(gpio_num_t pin, bool value) {
  gpio_set_level(pin, value ? 1 : 0);
}

bool read_pin(gpio_num_t pin) {
  return gpio_get_level(pin) == 0 ? false : true;
}

void Epaper_READBUSY() {
  ESP_LOGI(TAG, "Waiting for device to become non-busy ...");
  while(read_pin(BUSY_Pin)) {
    delay_millis(5);
  }
  ESP_LOGI(TAG, "Waiting for device to become non-busy ... DONE");
}

void SPI_Write(unsigned char value) {
  for (uint8_t i = 0; i < 8; i++) {
    set_pin(SCK_Pin, PIN_LOW);
    if (value & 0x80) {
      set_pin(SDI_Pin, PIN_HIGH);
    } else {
      set_pin(SDI_Pin, PIN_LOW);
    }
    value = (value << 1);
    set_pin(SCK_Pin, PIN_HIGH);
  }
}

void Epaper_Write_Command(unsigned char command) {
  set_pin(CS_Pin, PIN_LOW);
  set_pin(DC_Pin, PIN_LOW); // command write
  SPI_Write(command);
  set_pin(CS_Pin, PIN_HIGH);
}
void Epaper_Write_Data(unsigned char data) {
  set_pin(CS_Pin, PIN_LOW);
  set_pin(DC_Pin, PIN_HIGH); // data write
  SPI_Write(data);
  set_pin(CS_Pin, PIN_HIGH);
}

const unsigned char LUT_bw[] =
{0xA0,  0xA0, 0x54, 0xA2, 0xAA, 0x55,
0x8,  0x1,  0xA0, 0x50, 0x0,  0x0,
0x0,  0x0,  0x0,  0x0,  0x2A, 0xA,
0x32, 0x13, 0x62, 0x42, 0x2F, 0x10,
0x26, 0x3,  0x0,  0x0,  0x0,  0x6,
0x19, 0x0B};

const unsigned char LUT_r[] =
{0x88,  0x44, 0x88, 0x44, 0x20, 0x1,
0x44, 0x44, 0x0,  0x0,  0x0,  0x0,
0x0,  0x0,  0x0,  0x0,  0xC4, 0x18,
0xC3, 0xF,  0xA1, 0x3,  0xA,  0xD,
0x0,  0x0,  0x0,  0x0,  0x0,  0x6,
0x33, 0x0B};

void Epaper_LUT(uint8_t * wave_data) {
  // Exactly 29 bytes.
  Epaper_Write_Command(CMD_WRITE_LUT_REG);
  for (int count = 0; count < 29; count++) {
    Epaper_Write_Data(wave_data[count]);
  }
  Epaper_READBUSY();
}

void Epaper_Update(void) {
  Epaper_Write_Command(CMD_DISP_UPDATE_CTRL);
  Epaper_Write_Data(0xC7);
  Epaper_Write_Command(CMD_MASTER_ACTIVATION);
  Epaper_READBUSY();
  delay_millis(100);
}

void Epaper_DeepSleep(void) {
   Epaper_Write_Command(CMD_DEEP_SLEEP);
   Epaper_Write_Data(0x01);
   delay_millis(100);
}

void Display_All_White(void) {
  Epaper_Write_Command(CMD_SET_RAM_X_ADDR);
  Epaper_Write_Data(0x00);
  Epaper_Write_Command(CMD_SET_RAM_Y_ADDR);
  Epaper_Write_Data(0x00);
  Epaper_READBUSY();

  // write RAM for white(1)/black(0)
  Epaper_Write_Command(CMD_WRITE_RAM_1);

  for (int i = 0; i < 250; i++) {
    for (int j = 0; j < 16; j++) {
      Epaper_Write_Data(0xFF);
    }
  }

  Epaper_Write_Command(CMD_SET_RAM_X_ADDR);
  Epaper_Write_Data(0x00);
  Epaper_Write_Command(CMD_SET_RAM_Y_ADDR);
  Epaper_Write_Data(0x00);
  Epaper_READBUSY();

  // write RAM for red(FF)/ black/white(00)
  Epaper_Write_Command(CMD_WRITE_RAM_RED);

  for (int i = 0; i < 250; i++) {
    for (int j = 0; j < 16; j++) {
      Epaper_Write_Data(0x00);
    }
  }
}

void LUT_Written_by_MCU(int mode) {
  if (mode == MONO) {       //2.13a
    Epaper_Write_Command(CMD_GATE_DRIVING_VOLT_CTRL);
    Epaper_Write_Data(0x10);
    Epaper_Write_Data(0x0A);
    Epaper_Write_Command(CMD_SOURCE_DRIVING_VOLT_CTRL);
    Epaper_Write_Data(LUT_bw[30]);

    Epaper_Write_Command(CMD_RED_DISPLAY_CTRL);
    Epaper_Write_Data(0x00);  // LUT1

    Epaper_Write_Command(CMD_WRITE_VCOM_REGISTER);
    Epaper_Write_Data(0x8F);

    Epaper_Write_Command(CMD_BORDER_CONTROL);
    Epaper_Write_Data(0x71);
    Epaper_Write_Command(CMD_SET_DUMMY_LINE_PERIOD);
    Epaper_Write_Data(LUT_bw[29]);

    Epaper_Write_Command(CMD_SET_GATE_LINE_WIDTH);
    Epaper_Write_Data(LUT_bw[31]);

    Epaper_LUT((uint8_t*)LUT_bw);

    // 00,00,00 = BLACK/WHITE
    Epaper_Write_Command(CMD_RED_DISPLAY_CONTROL);
    Epaper_Write_Data(0x00);
    Epaper_Write_Data(0x00);
    Epaper_Write_Data(0x00);
  } else  if (mode == RED) {       //2.13r
    Epaper_Write_Command(CMD_GATE_DRIVING_VOLT_CTRL);
    Epaper_Write_Data(0x02);
    Epaper_Write_Data(0x0A);
    Epaper_Write_Command(CMD_SOURCE_DRIVING_VOLT_CTRL);
    Epaper_Write_Data(LUT_r[30]);

    Epaper_Write_Command(CMD_RED_DISPLAY_CTRL);
    Epaper_Write_Data(0xC0);
//      Epaper_Write_Command(0x05);
//      Epaper_Write_Data(0x00);

    Epaper_Write_Command(CMD_WRITE_VCOM_REGISTER);
    Epaper_Write_Data(0x8F);

    Epaper_Write_Command(CMD_BORDER_CONTROL);
    Epaper_Write_Data(0x71);
    Epaper_Write_Command(CMD_SET_DUMMY_LINE_PERIOD);
    Epaper_Write_Data(LUT_r[29]);

    Epaper_Write_Command(CMD_SET_GATE_LINE_WIDTH);
    Epaper_Write_Data(LUT_r[31]);

    Epaper_LUT((uint8_t*)LUT_r);

    // 00,80,00 = RED
    Epaper_Write_Command(CMD_RED_DISPLAY_CONTROL);
    Epaper_Write_Data(0x00);
    Epaper_Write_Data(0x80);
    Epaper_Write_Data(0x00);
  }
}

void Epaper_Init() {
  set_pin(RES_Pin, PIN_LOW);  // Module reset
  delay_millis(100);          // At least 10ms
  set_pin(RES_Pin, PIN_HIGH);
  delay_millis(100);

  Epaper_READBUSY();
  Epaper_Write_Command(CMD_SW_RESET);
  Epaper_READBUSY();

  Epaper_Write_Command(CMD_DRIVER_OUTPUT_CTRL);
  Epaper_Write_Data(0xf9);
  Epaper_Write_Data(0x01);

  Epaper_Write_Command(CMD_DATA_ENTRY_MODE);
  Epaper_Write_Data(0x03);

  Epaper_Write_Command(CMD_SET_RAM_X_ADDR_POS); //set Ram-X address start/end position
  Epaper_Write_Data(0x00);  // Start address
  Epaper_Write_Data(0x0F);  // End address. 0x0F-->(15+1)*8=128

  Epaper_Write_Command(CMD_SET_RAM_Y_ADDR_POS); //set Ram-Y address start/end position
  Epaper_Write_Data(0x00);   // Start Address
  Epaper_Write_Data(0xF9);   // End address
}

void init_gpio() {
  // Init WRITE pins.
  gpio_config_t io_conf_write = {};
  io_conf_write.intr_type = GPIO_INTR_DISABLE;
  io_conf_write.mode = GPIO_MODE_OUTPUT;
  io_conf_write.pin_bit_mask = ((1ULL<<RES_Pin) |
                                (1ULL<<DC_Pin)  |
                                (1ULL<<CS_Pin)  |
                                (1ULL<<SCK_Pin) |
                                (1ULL<<SDI_Pin));
  io_conf_write.pull_down_en = 0;
  io_conf_write.pull_up_en = 0;
  gpio_config(&io_conf_write);

  // Init READ pin.
  gpio_config_t io_conf_read = {};
  io_conf_read.intr_type = GPIO_INTR_DISABLE;
  io_conf_read.mode = GPIO_MODE_INPUT;
  io_conf_read.pin_bit_mask = (1ULL<<BUSY_Pin);
  io_conf_read.pull_down_en = 0;
  io_conf_read.pull_up_en = 0;
  gpio_config(&io_conf_read);
}

void Epaper_Load_Image(uint8_t* data, int num, int mode) {
  Epaper_Write_Command(CMD_SET_RAM_X_ADDR);
  Epaper_Write_Data(0x00);
  Epaper_Write_Command(CMD_SET_RAM_Y_ADDR);
  Epaper_Write_Data(0x00);
  Epaper_READBUSY();

  if (mode == MONO) {
    Epaper_Write_Command(CMD_WRITE_RAM_1);   //write RAM for black(0)/white (1)
  } else if (mode == RED) {
    Epaper_Write_Command(CMD_WRITE_RAM_RED);   //write RAM for red(1)/BW (0)
  }

  for (int i = 0; i < num; i++) {
    Epaper_Write_Data(data[i]);
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Initializing GPIO pins ...");
  init_gpio();
  ESP_LOGI(TAG, "Initializing GPIO pins DONE");
  Epaper_Init();

  // Display_All_White();

  ESP_LOGI(TAG, "Loading images ...");
  Epaper_Load_Image((uint8_t*)image_bw, ALLSCREEN_BYTES, MONO);
  Epaper_Load_Image((uint8_t*)image_red, ALLSCREEN_BYTES, RED);
  ESP_LOGI(TAG, "Loading images DONE");

  LUT_Written_by_MCU(MONO); // Load black and white waveform files
  Epaper_Update();
  LUT_Written_by_MCU(RED);  // Load red and white wave files
  Epaper_Update();
  Epaper_DeepSleep(); // Enter deep sleep, it's required, do not delete!
  delay_millis(500);
}
