#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <math.h>

#include "esp_system.h"
#include "esp_timer.h"

// SPI
#define PIN_NUM_MOSI_MISO     23
#define PIN_NUM_CLK           18
#define PIN_NUM_CS            5

#define LED_PIN               32
#define INTERRUPT_PIN         33

#define RANGE_2G_G            0.00006f
#define RANGE_4G_G            0.00012f
#define RANGE_8G_G            0.00024f
#define GRAVITY               9.81f

// Register

// axis high pass filter accelerometer output
#define KX122_XHP_L          0x00
#define KX122_XHP_H          0x01
#define KX122_YHP_L          0x02
#define KX122_YHP_H          0x03
#define KX122_ZHP_L          0x04
#define KX122_ZHP_H          0x05

// axis accelerometer output
#define KX122_XOUT_L          0x06
#define KX122_XOUT_H          0x07
#define KX122_YOUT_L          0x08
#define KX122_YOUT_H          0x09
#define KX122_ZOUT_L          0x0A
#define KX122_ZOUT_H          0x0B

#define KX122_WHO_AM_I        0x0F
#define KX122_INT_REL		      0x17
#define KX122_CNTL1           0x18
#define KX122_CNTL2           0x19
#define KX122_CNTL3           0x1A
#define KX122_ODCNTL          0x1B
#define KX122_INC1	        	0x1C
#define KX122_INC2		        0x1D
#define KX122_INC3		        0x1E
#define KX122_INC4		        0x1F 
#define KX122_TILT_TIMER      0x22
#define KX122_TILT_ANGLE_LL   0x32
#define KX122_TILT_ANGLE_HL   0x33
#define KX122_WUFC		        0x23
#define KX122_TDTRC           0x24
#define KX122_TDTC            0x25
#define KX122_TTH             0x26
#define KX122_TTL             0x27
#define KX122_FTD             0x28
#define KX122_STD             0x29
#define KX122_TLT             0x2A
#define KX122_TWS             0x2B
#define KX122_ATH		          0x30
#define KX122_BUF_CNTL2       0x3B
#define KX122_BUF_READ        0x3F
#define KX122_STATUS_REG      0x15
#define KX122_INS1            0x12
#define KX122_INS2            0x13
#define KX122_INT_REL         0x17
#define KX122_HYST_SET        0x34
#define KX122_FFCNTL          0x2E
#define KX122_FFTH            0x2C
#define KX122_FFC             0x2D

spi_device_handle_t spi;

struct Acceleration {
  float x_raw;
  float y_raw;
  float z_raw;

  float x_data;
  float y_data;
  float z_data;
} pos;


float roll, pitch, rollF, pitchF;

void spiInit()
{
  spi_bus_config_t bus_config = {
    .mosi_io_num = PIN_NUM_MOSI_MISO,
    .miso_io_num = -1,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 0,
  };

  spi_device_interface_config_t devcfg = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .clock_speed_hz = 1 * 1000 * 1000, // Hz
    .duty_cycle_pos = 128, // 50% duty cycle
    .mode = 0,
    .spics_io_num = PIN_NUM_CS,
    .cs_ena_posttrans = 0,
    .queue_size = 1,
    .flags = SPI_DEVICE_3WIRE | SPI_DEVICE_HALFDUPLEX,
  };

  spi_bus_initialize(VSPI_HOST, &bus_config, SPI_DMA_DISABLED);
  spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
}

void spi3WireInit()
{
  spi_transaction_t startData = {
    .flags = SPI_TRANS_USE_TXDATA,
    .cmd = 0,
    .addr = 0,
    .length = 16,
    .rxlength = 0,
    .tx_data = { 0x1c, 0x31 },
    .rx_data = { },
  };

  spi_device_polling_transmit(spi, &startData);
}
void registerWrite(uint8_t adr, uint8_t data)
{
  spi_transaction_t startData = {
    .flags = SPI_TRANS_USE_TXDATA,
    .cmd = 0,
    .addr = 0,
    .length = 16,
    .rxlength = 0,
    .tx_data = { adr, data },
    .rx_data = { },
  };

  spi_device_polling_transmit(spi, &startData);
}

uint8_t registerRead(uint8_t addr)
{
  spi_transaction_t registerData = {
    .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
    .cmd = 0,
    .addr = 0,
    .length = 8,
    .rxlength = 9,
    .tx_data = { (0x80 | addr) }, // older bit for write - 0, read - 1
    .rx_data = {},
  };

  spi_device_polling_transmit(spi, &registerData);
  uint8_t data = registerData.rx_data[0] << 1 | registerData.rx_data[1] >> 7;

  return data;
}

//------------------------------------------------------------------------------------------------------------------------//
// 4. Asynchronous Read Back Acceleration Data (Setting G-Range and ODR)
void setAsynchronousReadBack()
{
  registerWrite(KX122_CNTL1, 0x00);   // clear Control Register
  registerWrite(KX122_CNTL1, 0x40);   // set the accelerometer in stand-by mode, to set the performance mode to High Resolution
  registerWrite(KX122_ODCNTL, 0x02);  // set (ODR) of the accelerometer to 50 Hz
  registerWrite(KX122_CNTL1, 0xD0);   // set the accelerometer into operating mode (PC1 = 1), 0xC0 - 2G, 0xC8 - 4G, 0xD0 -8G
}

// 5. Synchronous Hardware Interrupt Read Back Acceleration Data(Setting G-Range and ODR)
void setSynchronousHardwareInterrupt()
{
  registerWrite(KX122_CNTL1, 0x00);   // clear Control Register
  registerWrite(KX122_CNTL1, 0x60);   // set the accelerometer in stand - by mode, to set the performance mode to High Resolution(full power)
  registerWrite(KX122_INC1, 0x39);    // enable physical interrupt pin INT1
  registerWrite(KX122_INC2, 0x10);    // set the Data Ready interrupt to be reported on physical interrupt pin INT1.
  registerWrite(KX122_ODCNTL, 0x02);  // set (ODR) of the accelerometer to 50 Hz
  registerWrite(KX122_CNTL1, 0xE0);   // set the accelerometer into operating mode (PC1 = 1)
}
// 6. Sample Buffer-Full Interrupt via Physical Hardware Interrupt
void setBufferFullInterrupt()
{
  registerWrite(KX122_CNTL1, 0x00);      // clear Control Register, set the accelerometer in stand-by mode
  registerWrite(KX122_INC1, 0x39);       // enable physical interrupt pin INT1
  registerWrite(KX122_INC4, 0x40);       // set the Buffer Full interrupt to be reported on physical interrupt pin INT1
  registerWrite(KX122_ODCNTL, 0x02);     // set the ODR of the accelerometer to 50 Hz
  registerWrite(KX122_BUF_CNTL2, 0xA0);  // set the resolution of the acceleration data
  registerWrite(KX122_WUFC, 0x80);       // set the accelerometer in stand-by mode
}

// 7. Wake Up Function via latched physical hardware interrupt
void setWakeUpFunctionInterrupt()
{
  registerWrite(KX122_CNTL1, 0x00);      // clear Control Register, set the accelerometer in stand-by mode
  registerWrite(KX122_CNTL1, 0x02);      // enable the Wake Up (0x42 High Power) 02
  registerWrite(KX122_CNTL3, 0x06);      // set the Output Data Rate of the Wake-Up function (motion detection)

  registerWrite(KX122_INC2, 0x7F);       // define the direction of detected motion for all positive and negative directions

  //registerWrite(KX122_WUFC, 0x00);        // ??? timer wake-up

  registerWrite(KX122_ATH, 0x09);        // set the level to 0.5g
  registerWrite(KX122_INC1, 0x31);       // create an active high and latched interrupt 0x31
  registerWrite(KX122_INC4, 0x02);       // set the wake-up function interrupt to be reported on physical interrupt pin INT1
  registerWrite(KX122_CNTL1, 0x82);      // set the accelerometer in operating mode with the previously defined settings (0xC2 High Power) 82
}
void getBufferFullInterrupt(struct Acceleration *p)
{
  // if interrupt
  if (registerRead(KX122_STATUS_REG) & 0x10 || registerRead(KX122_INS2) & 0x40) {
    p->x_raw = registerRead(KX122_BUF_READ);
    p->y_raw = registerRead(KX122_BUF_READ);
    p->z_raw = registerRead(KX122_BUF_READ);

    p->x_data = (p->x_raw * RANGE_2G_G) * GRAVITY;
    p->y_data = (p->y_raw * RANGE_2G_G) * GRAVITY;
    p->z_data = (p->z_raw * RANGE_2G_G) * GRAVITY;
  }
}

// 8. Activate Tilt Position Function with Face Detect
void setActivateTiltPosition()
{
  registerWrite(KX122_CNTL1, 0x00);         // clear Control Register
  registerWrite(KX122_CNTL1, 0x41);         // set the accelerometer in stand - by mode, to set the performance mode of the accelerometer to High Resolution
  registerWrite(KX122_CNTL2, 0x08);         // enable Tilt detection from positive and negative directions of all three axes(+ x, -x, + y, -y, + z, -z). 
  registerWrite(KX122_CNTL3, 0x98);         // set the output data rate for the Tilt Position function to 12.5Hz. 
  registerWrite(KX122_TILT_TIMER, 0x01);    // 80 msec timer will be sufficient
  registerWrite(KX122_TILT_ANGLE_LL, 0x0C); // set the low threshold to 22?
  registerWrite(KX122_TILT_ANGLE_HL, 0x1A); // set the High threshold
  registerWrite(KX122_HYST_SET, 0x14);      // set the hysteresis that is placed between the screen rotation states to ±15?
  registerWrite(KX122_INC1, 0x31);          // physical interrupt of the previously defined Tilt Position function
  registerWrite(KX122_INC4, 0x01);          // set the Tilt Position interrupt
  registerWrite(KX122_CNTL1, 0xC1);         // set the accelerometer in operating mode
}

// 9.  Activate Tap/Double Tap Function
void setTap()
{
  registerWrite(KX122_CNTL1, 0x00);   // clear Control Register
  registerWrite(KX122_CNTL1, 0x44);   // set the accelerometer in stand-by mode, to set the performance mode of the accelerometer to High Resolution(full power)
  registerWrite(KX122_INC3, 0x3F);    // enable tap/double tap from positive and negative directions of all three axes(+ x, -x, + y, -y, + z, -z)
  registerWrite(KX122_CNTL3, 0x98);   // set the output data rate for the Directional Tap function to 400Hz
  registerWrite(KX122_TDTRC, 0x03);   // enable interrupt on single tap and double tap
  registerWrite(KX122_TDTC, 0x78);    // set the counter to 0.3 sec
  registerWrite(KX122_TTH, 0xCB);     // high threshold 203 0xCB
  registerWrite(KX122_TTL, 0x1A); // low threshold 26 1A
  registerWrite(KX122_FTD, 0xA2);     // set the FTD counter register to 0.005 seconds
  registerWrite(KX122_STD, 0x24);     // set the STD counter register to 0.09 seconds
  registerWrite(KX122_TLT, 0x28);     // set the TLT counter register to 0.1 seconds
  registerWrite(KX122_TWS, 0xA0);     // set the TWS counter register to 0.4 seconds
  registerWrite(KX122_INC1, 0x31);    // output the physical interrupt
  registerWrite(KX122_INC4, 0x04);    // set the Tap/Double-Tap interrupt (TDTI) to be reported on physical interrupt pin INT1
  registerWrite(KX122_CNTL1, 0xC4);   // set the accelerometer in operating mode
}

// 10. Activate Free - fall Function
void setFreeFall()
{
  registerWrite(KX122_CNTL1, 0x00);   // clear Control Register
  registerWrite(KX122_CNTL1, 0x40);   // set the accelerometer in stand-by mode
  registerWrite(KX122_FFCNTL, 0x80);  // enable Free fall engine
  registerWrite(KX122_FFTH, 0x08);    // enable Free fall engine
  registerWrite(KX122_FFC, 0x04);     // set the Free-fall delay detection to 0.320 sec
  registerWrite(KX122_INC1, 0x31);    // output the physical interrupt
  registerWrite(KX122_INC4, 0x80);    // set the Free-fall interrupt (FFI) to bereported on physical interrupt pin INT1
  registerWrite(KX122_CNTL1, 0xC0);   // set the accelerometer in operating mode
}

void getAccelerationData(struct Acceleration *p)
{
  p->x_raw = (float)((int16_t)(registerRead(KX122_XOUT_H) << 8) | registerRead(KX122_XOUT_L));
  p->x_data = (p->x_raw * RANGE_2G_G) * GRAVITY;

  p->y_raw = (float)((int16_t)(registerRead(KX122_YOUT_H) << 8) | registerRead(KX122_YOUT_L));
  p->y_data = (p->y_raw * RANGE_2G_G) * GRAVITY;

  p->z_raw = (float)((int16_t)(registerRead(KX122_ZOUT_H) << 8) | registerRead(KX122_ZOUT_L));
  p->z_data = (p->z_raw * RANGE_2G_G) * GRAVITY;
}


//------------------------------------------------------------------------------------------------------------------------//
void visualizatio(struct Acceleration *p)
{
  float x_pos = p->x_data * 100;
  float y_pos = p->y_data * 100;
  float z_pos = p->z_data * 100;
  
  // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
  roll = atan(y_pos / sqrt(pow(x_pos, 2) + pow(z_pos, 2))) * 180 / 3.14;
  pitch = atan(-1 * x_pos / sqrt(pow(y_pos, 2) + pow(z_pos, 2))) * 180 / 3.14;

  // Low-pass filter
  rollF = 0.94 * rollF + 0.06 * x_pos;
  pitchF = 0.94 * pitchF + 0.06 * y_pos;

  printf("%.2f", roll);
  printf("/");
  printf("%.2f\n", pitch);
}

//------------------------------------------------------------------------------------------------------------------------//

uint8_t count;
int64_t timer_start;
bool flag = false;

void counter()
{
  if (gpio_get_level(INTERRUPT_PIN) == 1)
  {
    timer_start = esp_timer_get_time();
    count++;
    registerRead(KX122_INT_REL);
    flag = true;
  }

  if (flag)
  {
    if ((esp_timer_get_time() - timer_start) > 100000) {
      printf("%d", count);
      printf("/");
      printf("\n");

      timer_start = 0;
      count = 0;
      flag = false;
    }
  }
}


//------------------------------------------------------------------------------------------------------------------------//

void app_main(void)
{
  // Init SPI
  spiInit();
  spi3WireInit();

  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << INTERRUPT_PIN),
    .mode = GPIO_MODE_INPUT,
    .intr_type = GPIO_INTR_DISABLE,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE
  };
  gpio_config(&io_conf);

  // WHO_AM_I
  uint8_t status = registerRead(KX122_WHO_AM_I);
  (status == 0x1B) ? gpio_set_level(LED_PIN, 1), printf("WHO_AM_I: 0x%02X\n", status) : gpio_set_level(LED_PIN, 0);

  printf("X\n");

  // Example
  //setAsynchronousReadBack();
  //setSynchronousHardwareInterrupt();
  //setBufferFullInterrupt();
  setWakeUpFunctionInterrupt();
  //setActivateTiltPosition();
  //setTap();
  //setFreeFall();


  while (1)
  {
    // Example
    //getAccelerationData(&pos);
    //getBufferFullInterrupt(&pos);
//
//    if ((registerRead(KX122_STATUS_REG) & (1 << 4)) != 0) {
     //registerRead(KX122_INT_REL); // clear interrupt
//    }

//    if (gpio_get_level(INTERRUPT_PIN) == 1)
//    {
//      gpio_set_level(LED_PIN, 1);
//      registerRead(KX122_INT_REL);
//    }
//    else
//    {
//      gpio_set_level(LED_PIN, 0);
//    }

    counter();


    //vTaskDelay(1000 / portTICK_PERIOD_MS)

     //printf("TIMER: %lld\n", esp_timer_get_time() - start);
   
    //printf("%d\n", pos.x_data);
//    printf("%d,", pos.y_data);
//    printf("%d\n", pos.z_data);

    //visualizatio(&pos);
    //vTaskDelay(pdMS_TO_TICKS(50));
  }
}