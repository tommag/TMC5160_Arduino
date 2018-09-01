/* ESP32 / TMC5160 UART example

This code demonstrates the usage of a Trinamic TMC5160 stepper driver using its
single-wire interface from an ESP32 host.

Hardware setup :
A RS485 transceiver must be connected to the Serial1 pins with the TX Enable pin
accessible to the uC

Tie the A output to the TMC5160 I/O voltage with a 1k resistor

The TMC5160 Enable line must be connected to GND to enable the driver.

                                                3.3/5V
                                                  +     +-----------------+
                                            +-----+---+-+ VCC_IO          |
                                            |         | |                 |
                                           +++        | |                 |
+-----------+        +--------------+      | | 1k     +-+ SWSEL           |
|           |        |              |      +++          |                 |
|     RX 16 +--------+ RO           |       |           |                 |
|           |        |            A +-------+-----------+ SWP         NAI ++ open
|           |    +---+ /RE          |                   |                 |
|  TX EN 19 +----+   |            B +-------------------+ SWN             |
|           |    +---+ DE           |                   |                 |
|           |        |          GND +----+         +----+ DRV_ENN         |
|     TX 17 +--------+ DI           |    |         |    |                 |
|           |        |              |    +---------+----+ GND             |
+-----------+        +--------------+                   +-----------------+
 ESP32                   MAX3485                              TMC5130
                       or equivalent


Tested on Adafruit Feather HUZZAH32

Copyright (c) 2018 Tom Magnier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <Arduino.h>
#include <TMC5160.h>
#include "driver/uart.h"

const uint8_t UART1_RX_PIN = 16;
const uint8_t UART1_TX_PIN = 17;
const uint8_t UART1_TX_EN_PIN = 21;   // Differential transceiver TX enable pin
const int UART_BUF_SIZE = 256;

TMC5160_UART_ESP32 tmc = TMC5160_UART_ESP32(UART_NUM_1, 0); //Use UART 1 ; address 0


void setup()
{
  // USB/debug serial coms
  Serial.begin(115200);

  //UART TMC bus init
  uart_config_t uart_config = {
        .baud_rate = 500000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
  };
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, UART1_TX_PIN, UART1_RX_PIN, UART1_TX_EN_PIN, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_1, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX); //Not yet available on Arduino-ESP32...

  // status LED
  pinMode(LED_BUILTIN, OUTPUT);

  // This sets the motor & driver parameters /!\ run the configWizard for your driver and motor first !
  TMC5160::PowerStageParameters powerStageParams;
  TMC5160::MotorParameters motorParams;
  powerStageParams.drvStrength = 0;
  powerStageParams.bbmTime = 8;
  powerStageParams.bbmClks = 0;
  motorParams.globalScaler = 98;
  motorParams.irun = 31;
  motorParams.ihold = 0;
  motorParams.freewheeling = TMC5160_Reg::FREEWHEEL_SHORT_LS;
  motorParams.pwmOfsInitial = 36;
  motorParams.pwmGradInitial = 73;

  tmc.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);

  // ramp definition
  tmc.setRampMode(TMC5160::POSITIONING_MODE);
  tmc.setMaxSpeed(200);
  tmc.setRampSpeeds(0, 0.1, 100); //Start, stop, threshold speeds
  tmc.setAccelerations(250, 350, 500, 700); //AMAX, DMAX, A1, D1

  Serial.println("starting up");
}

void loop()
{
  uint32_t now = millis();
  static unsigned long t_dirchange, t_echo;
  static bool dir;

  // every n seconds or so...
  if ( now - t_dirchange > 3000 )
  {
    t_dirchange = now;

    // reverse direction
    dir = !dir;
    tmc.setTargetPosition(dir ? 200 : 0);  // 1 full rotation = 200s/rev

    digitalWrite(LED_BUILTIN, dir);
  }

  // print out current position
  if( now - t_echo > 100 )
  {
    t_echo = now;

    // get the current target position
    float xactual = tmc.getCurrentPosition();
    float vactual = tmc.getCurrentSpeed();
    Serial.print("current position : ");
    Serial.print(xactual);
    Serial.print("\tcurrent speed : ");
    Serial.println(vactual);
  }

}
