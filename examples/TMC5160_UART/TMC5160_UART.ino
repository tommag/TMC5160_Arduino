/* TMC5160 UART example

This code demonstrates the usage of a Trinamic TMC5160 stepper driver using its
single-wire interface.

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
|     RX 0 +--------+ RO           |       |           |                 |
|           |        |            A +-------+-----------+ SWP         NAI ++ open
|           |    +---+ /RE          |                   |                 |
|  TX EN 5 +----+   |            B +-------------------+ SWN             |
|           |    +---+ DE           |                   |                 |
|           |        |          GND +----+         +----+ DRV_ENN         |
|     TX 1 +--------+ DI           |    |         |    |                 |
|           |        |              |    +---------+----+ GND             |
+-----------+        +--------------+                   +-----------------+
 Arduino                 MAX3485                              TMC5130
                       or equivalent


Copyright (c) 2018-2021 Tom Magnier

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

const uint8_t UART_TX_EN_PIN = 5;   // Differential transceiver TX enable pin

TMC5160_UART_Transceiver motor = TMC5160_UART_Transceiver(UART_TX_EN_PIN, Serial1, 0); //Use Serial 1 ; address 0


void setup()
{
  // USB/debug serial coms
  Serial.begin(115200);

  // Init TMC serial bus @ 250kbps
  Serial1.begin(250000);
  Serial1.setTimeout(5); // TMC5130 should answer back immediately when reading a register.

  // status LED
  pinMode(LED_BUILTIN, OUTPUT);

   // This sets the motor & driver parameters /!\ run the configWizard for your driver and motor for fine tuning !
  TMC5160::PowerStageParameters powerStageParams; // defaults.
  TMC5160::MotorParameters motorParams;
  motorParams.globalScaler = 98; // Adapt to your driver and motor (check TMC5160 datasheet - "Selecting sense resistors")
  motorParams.irun = 31;
  motorParams.ihold = 16;

  motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);

  // ramp definition
  motor.setRampMode(TMC5160::POSITIONING_MODE);
  motor.setMaxSpeed(400);
  motor.setAcceleration(500);

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
    motor.setTargetPosition(dir ? 200 : 0);  // 1 full rotation = 200s/rev
  }

  // print out current position
  if( now - t_echo > 100 )
  {
    t_echo = now;

    // get the current target position
    float xactual = motor.getCurrentPosition();
    float vactual = motor.getCurrentSpeed();
    Serial.print("current position : ");
    Serial.print(xactual);
    Serial.print("\tcurrent speed : ");
    Serial.println(vactual);
  }
}
