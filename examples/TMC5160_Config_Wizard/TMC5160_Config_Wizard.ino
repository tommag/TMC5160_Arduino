/* TMC5160 Config wizard

This code gives an easy way to configure the various driver setting of a Trinamic TMC5160-based driver.

Hardware setup :


///// Option 1 : UART
A RS485 transceiver must be connected to the Serial1 pins with the TX Enable pin
accessible to the uC

Tie the A output to the TMC5160 I/O voltage with a 1k resistor.

The TMC5160 Enable line must be connected to GND to enable the driver.

                                                3.3/5V
                                                  +     +-----------------+
                                            +-----+---+-+ VCC_IO          |
                                            |         | |                 |
                                           +++        | |                 |
+-----------+        +--------------+      | | 1k     +-+ SWSEL           |
|           |        |              |      +++          |                 |
|      RX 0 +--------+ RO           |       |           |                 |
|           |        |            A +-------+-----------+ SWP         NAI ++ open
|           |    +---+ /RE          |                   |                 |
|   TX EN 2 +----+   |            B +-------------------+ SWN             |
|           |    +---+ DE           |                   |                 |
|           |        |          GND +----+         +----+ DRV_ENN         |
|      TX 1 +--------+ DI           |    |         |    |                 |
|           |        |              |    +---------+----+ GND             |
+-----------+        +--------------+                   +-----------------+
Arduino / Teensy          MAX3485                              TMC5130
                       or equivalent

Tie CLK16 to GND to use the TMC5160 internal clock.
Tie SPI_MODE to GND, SD_MODE to GND for UART mode.

///// Option 2 : SPI
Connect the following pins to the TMC5160 : 
  MOSI (Teensy : 11)  <=> SDI
  MISO (Teensy : 12)  <=> SDO
  SCK (Teensy : 13)   <=> SCK
  5                   <=> CSN
  8                   <=> DRV_ENN (optional, tie to GND if not used)
  GND                 <=> GND
  3.3V/5V             <=> VCC_IO (depending on the processor voltage)

Tie CLK16 to GND to use the TMC5160 internal clock.
Tie SPI_MODE to VCC_IO, SD_MODE to GND for SPI mode.

For both options, the TMC5160 VS pin must also be powered. 

Copyright (c) 2017 Tom Magnier

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

const uint8_t UART_TX_EN = 2;   // Differential transceiver TX enable pin
const uint8_t SPI_CS = 5; // CS pin in SPI mode
const uint8_t SPI_DRV_ENN = 8;  // DRV_ENN pin in SPI mode

TMC5160* motor;

char readCommandToken(const char *tokens, int tokenCount, int defaultToken = -1)
{
  char answer = 0;

  while (answer == 0)
  {
    //Flush input buffer
    while (Serial.available())
      Serial.read();

    for (int i = 0; i < tokenCount-1; i++)
    {
      Serial.print(tokens[i]);
      Serial.print('/');
    }
    if (tokenCount > 0)
      Serial.print(tokens[tokenCount-1]);

    if (defaultToken >= 0 && defaultToken < tokenCount)
    {
      Serial.print(" (Default: ");
      Serial.print(tokens[defaultToken]);
    }
    Serial.println("):");

    while (!Serial.available());

    char readChar = Serial.read();

    if (defaultToken >= 0 && defaultToken < tokenCount && isSpace(readChar))
    {
      answer = tokens[defaultToken];
      break;
    }

    for (int i = 0; i < tokenCount; i++)
      if (readChar == tokens[i])
        answer = readChar;

    if (answer == 0)
      Serial.print("Invalid character. ");
  }

  return answer;
}

void printDriverStatus()
{
  //TODO inside library

  TMC5160_Reg::GSTAT_Register gstat = {0};
  gstat.value = motor->readRegister(TMC5160_Reg::GSTAT);

  if (gstat.uv_cp)
  {
    Serial.println("Charge pump undervoltage");
  }
  else if (gstat.drv_err)
  {
    Serial.print("DRV_STATUS: ");
    TMC5160_Reg::DRV_STATUS_Register drvStatus = {0};
    drvStatus.value = motor->readRegister(TMC5160_Reg::DRV_STATUS);
    Serial.print(drvStatus.value, HEX);
    Serial.print(" (");
    if (drvStatus.s2vsa)
      Serial.print("Short to supply phase A");
    else if (drvStatus.s2vsb)
      Serial.print("Short to supply phase B");
    else if (drvStatus.s2ga)
      Serial.print("Short to ground phase A");
    else if (drvStatus.s2gb)
      Serial.print("Short to ground phase B");
    else if (drvStatus.ot)
      Serial.print("Overtemperature");
    Serial.println(")");
  }
  else
  {
    Serial.println("No error condition.");
  }
}

void tunePowerStage(TMC5160::PowerStageParameters *powerParams);
void getPowerStageParams(TMC5160::PowerStageParameters *powerParams);
void tuneMotorCurrent(TMC5160::MotorParameters *motorParams);
void tuneStealthChop(TMC5160::PowerStageParameters *powerParams, TMC5160::MotorParameters *motorParams);
void getStealthChopParams(TMC5160::MotorParameters *motorParams);
void startPowerStageTroubleshooting();

void setup()
{
  // init serial coms
  Serial.begin(115200);
  while(!Serial);

  Serial.print("Select the communication interface to use ('s' for SPI, 'u' for UART)");
  if (readCommandToken("su", 2, 1) == 's') 
  {
    SPI.begin();
    motor = new TMC5160_SPI(SPI_CS);
    pinMode(SPI_DRV_ENN, OUTPUT);
    digitalWrite(SPI_DRV_ENN, LOW); // Active low

    // Check if the TMC5160 answers back
    TMC5160_Reg::IOIN_Register ioin = { 0 };

    while (ioin.version != motor->IC_VERSION)
    {
      ioin.value = motor->readRegister(TMC5160_Reg::IO_INPUT_OUTPUT);

      if (ioin.value == 0 || ioin.value == 0xFFFFFFFF) 
      {
        Serial.println("No TMC5160 found.");
        delay(2000);
      }
      else
      {
        Serial.println("Found a TMC device.");
        Serial.print("IC version: 0x");
        Serial.print(ioin.version, HEX);
        Serial.print(" (");
        if (ioin.version == motor->IC_VERSION)
          Serial.println("TMC5160).");
        else
          Serial.println("unknown IC !)");
      }
    }
  }
  else
  {
    // Init TMC serial bus @ 500kbps
    Serial1.begin(500000);
    Serial1.setTimeout(2); // TMC5160 should answer back immediately when reading a register.

    TMC5160_UART_Generic * _motor; // Temp pointer for UART specific functions
    //Use Serial1 (hardware UART on Feather M0/Teensy) ; address 0
    #if defined(KINETISK) && 0
    Serial1.transmitterEnable(UART_TX_EN);
    _motor = new TMC5160_UART(Serial1, 0);
    #else
    _motor = new TMC5160_UART_Transceiver(UART_TX_EN, Serial1, 0);
    #endif
    _motor->setCommunicationMode(TMC5160_UART::RELIABLE_MODE);

    motor = _motor;

    // TMC5160 research on bus
    TMC5160_UART::ReadStatus readStatus;
    TMC5160_Reg::IOIN_Register ioin = { 0 };

    while (ioin.version != _motor->IC_VERSION)
    {
      ioin.value = _motor->readRegister(TMC5160_Reg::IO_INPUT_OUTPUT, &readStatus);

      switch (readStatus)
      {
        case TMC5160_UART::SUCCESS:
        Serial.print("Found a TMC device at address ");
        Serial.println(_motor->getSlaveAddress());
        Serial.print("IC version: 0x");
        Serial.print(ioin.version, HEX);
        Serial.print(" (");
        if (ioin.version == _motor->IC_VERSION)
          Serial.println("TMC5160).");
        else
          Serial.println("unknown IC !)");
        break;

        case TMC5160_UART::NO_REPLY:
        Serial.println("No TMC5160 found.");
        delay(2000);
        break;

        case TMC5160_UART::BAD_CRC:
        Serial.println("A TMC device replied with a bad CRC.");
        delay(100);
        break;
      }
    }
  }

  /* Driver status */
  Serial.println();
  Serial.print("Global status: 0x");
  Serial.println(motor->readRegister(TMC5160_Reg::GSTAT), HEX);
  Serial.print("Driver status: 0x");
  Serial.println(motor->readRegister(TMC5160_Reg::DRV_STATUS), HEX);

  /* Power stage tuning */
  TMC5160::PowerStageParameters powerParams;
  Serial.println();
  Serial.println("Start power stage tuning ? ");
  if (readCommandToken("yn", 2, 1) == 'y')
    tunePowerStage(&powerParams);
  else
    getPowerStageParams(&powerParams);

  /* Motor tuning */
  TMC5160::MotorParameters motorParams;
  Serial.println();
  tuneMotorCurrent(&motorParams);

  /* stealthChop tuning */
  Serial.println();
  Serial.println("Start stealthChop PWM mode tuning ? ");
  if (readCommandToken("yn", 2, 0) == 'y')
    tuneStealthChop(&powerParams, &motorParams);
  else
    getStealthChopParams(&motorParams);


  Serial.println("Driver and motor tuning is finished. To use the determined parameters you can");
  Serial.println("copy and paste the following code snippet: ");
  Serial.println();

  Serial.println("TMC5160::PowerStageParameters powerStageParams;");
  Serial.println("TMC5160::MotorParameters motorParams;");
  Serial.print("powerStageParams.drvStrength = ");
  Serial.print(powerParams.drvStrength);
  Serial.println(";");
  Serial.print("powerStageParams.bbmTime = ");
  Serial.print(powerParams.bbmTime);
  Serial.println(";");
  Serial.print("powerStageParams.bbmClks = ");
  Serial.print(powerParams.bbmClks);
  Serial.println(";");
  Serial.print("motorParams.globalScaler = ");
  Serial.print(motorParams.globalScaler);
  Serial.println(";");
  Serial.print("motorParams.irun = ");
  Serial.print(motorParams.irun);
  Serial.println(";");
  Serial.print("motorParams.ihold = ");
  Serial.print(motorParams.ihold);
  Serial.println(";");
  Serial.print("motorParams.freewheeling = ");
  Serial.print(motorParams.freewheeling); //TODO print human readable values
  Serial.println(";");
  Serial.print("motorParams.pwmOfsInitial = ");
  Serial.print(motorParams.pwmOfsInitial);
  Serial.println(";");
  Serial.print("motorParams.pwmGradInitial = ");
  Serial.print(motorParams.pwmGradInitial);
  Serial.println(";");
  Serial.println("motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);");


  Serial.println();
  while (Serial.available())
    Serial.read();
  Serial.println("Press any key to begin operation with the specified parameters.");
  while (!Serial.available());

  motor->begin(powerParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);

  // //TODO TEMP, include in begin/tuning
  // TMC5160_Reg::SHORT_CONF_Register shortConf = { 0 };
  // shortConf.s2vs_level = 6; //Default value
  // shortConf.s2g_level = 8; //Increased value at high voltages
  // shortConf.shortfilter = 1; //Default value
  // motor->writeRegister(TMC5160_Reg::SHORT_CONF, shortConf.value);

  motor->setCurrentPosition(0);
  motor->setAcceleration(800);
  motor->setMaxSpeed(400);
  motor->setTargetPosition(4000);
}

void loop()
{
  //TODO serial commands => velocity / position operation and feedback (driver state, position, etc)

  //Run 1000 steps every time a serial line is sent.
  if (Serial.available())
  {
    while (Serial.available())
      Serial.read();

    motor->setTargetPosition(motor->getCurrentPosition() + 1000);
  }

  static unsigned long lastFeedbackTime = millis();
  if (millis() - lastFeedbackTime > 500)
  {
    lastFeedbackTime = millis();
    char buffer[11];
    Serial.print("GSTAT: ");
    sprintf(buffer, "0x%08x", motor->readRegister(TMC5160_Reg::GSTAT));
    Serial.print(buffer);
    Serial.print("\tDRV_STATUS: ");
    sprintf(buffer, "0x%08x", motor->readRegister(TMC5160_Reg::DRV_STATUS));
    Serial.println(buffer);
  }
}

void tunePowerStage(TMC5160::PowerStageParameters *powerParams)
{
  /*while (Serial.available())
    Serial.read();
  Serial.println("Please disconnect motor and press any key when ready.");
  while (!Serial.available()); */

  TMC5160_Reg::GSTAT_Register gstat = {0};
  gstat.value = motor->readRegister(TMC5160_Reg::GSTAT);
  if (!gstat.reset)
  {
    while (Serial.available())
      Serial.read();
    Serial.println("Warning: the driver is not in its reset state. You may want to power cycle it then press any key.");
    while (!Serial.available());
  }

  Serial.println("Starting operation with lowest current setting possible.");

  //Set short detectors to highest sensitivity
  TMC5160_Reg::SHORT_CONF_Register shortConf = {0};
  shortConf.s2vs_level = 4;
  shortConf.s2g_level = 2;
  shortConf.shortfilter = 0;
  motor->writeRegister(TMC5160_Reg::SHORT_CONF, shortConf.value);

  TMC5160::MotorParameters motorParams;
  motorParams.globalScaler = 32;
  motorParams.irun = 0;
  motorParams.ihold = 0;
  motor->begin(*powerParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  //motor->writeRegister(TMC5160_Reg::IO_INPUT_OUTPUT, 0);

  delay(200);

  //Check driver error state
  gstat.value = motor->readRegister(TMC5160_Reg::GSTAT);
  if (gstat.drv_err || gstat.uv_cp)
  {
    Serial.println();
    Serial.println("/!\\ Driver error !");
    printDriverStatus();
    startPowerStageTroubleshooting();
  }

  Serial.println();
  Serial.println("You should monitor one of the bridge outputs (motor coil connections) using a scope.");
  Serial.println("Let's choose the MOSFET gate driver current. This should be the lowest setting giving slopes < 100ns");

  TMC5160_Reg::DRV_CONF_Register drvConf = { 0 };
  // Initial values
  drvConf.bbmclks = 4;
  drvConf.drvstrength = 2;
  motor->writeRegister(TMC5160_Reg::DRV_CONF, drvConf.value);

  char answer = 0;
  while (answer != 'd')
  {
    Serial.print("Current DRVSTRENGTH value: ");
    Serial.print(drvConf.drvstrength);
    Serial.print(". Press +/- to increase / decrease or d when done.");
    answer = readCommandToken("+-d", 3);

    switch (answer)
    {
      case '+':
        drvConf.drvstrength = constrain(drvConf.drvstrength+1, 0, 3);
        break;

      case '-':
        drvConf.drvstrength = constrain(drvConf.drvstrength-1, 0, 3);
        break;

      default:
        break;
    }
    motor->writeRegister(TMC5160_Reg::DRV_CONF, drvConf.value);
  }

  Serial.println("");
  Serial.println("Let's tune the Break Before Make time. This should be as short as possible but still avoid overlapping of low side and high side MOSFETs.");
  Serial.println("If the switching slope is in the range 40-80ns, the smallest setting (0, 0) can be used.");
  Serial.println("You should check the bridge output for a small plateau at -1.2V or VM + 1.2V and tune its length");
  Serial.println("Keep 30% of reserve !");
  answer = 0;
  while (answer != 'd')
  {
    Serial.print("Current BBMCLKS: ");
    Serial.print(drvConf.bbmclks);
    Serial.print(". Press +/- to increase / decrease BBM raw value or d when done.");
    answer = readCommandToken("+-d", 3);

    switch (answer)
    {
      case '+':
        drvConf.bbmclks = constrain(drvConf.bbmclks+1, 0, 15);
        break;

      case '-':
        drvConf.bbmclks = constrain(drvConf.bbmclks-1, 0, 15);
        break;

      default:
        break;
    }
    motor->writeRegister(TMC5160_Reg::DRV_CONF, drvConf.value);
  }
  if (drvConf.bbmclks <= 4)
  {
    Serial.println();
    Serial.println("BBMCLKS has a small value => BBMTIME can be used for fine tuning.");
    Serial.println("Keep 30% of reserve !");
    drvConf.bbmtime = 24;
    drvConf.bbmclks = 0;
    motor->writeRegister(TMC5160_Reg::DRV_CONF, drvConf.value);

    answer = 0;
    while (answer != 'd')
    {
      Serial.print("Current BBMTIME: ");
      Serial.print(drvConf.bbmtime);
      Serial.print(". Press +/- to increase / decrease or d when done.");
      answer = readCommandToken("+-d", 3);

      switch (answer)
      {
        case '+':
          drvConf.bbmtime = constrain(drvConf.bbmtime+1, 0, 24);
          break;

        case '-':
          drvConf.bbmtime = constrain(drvConf.bbmtime-1, 0, 24);
          break;

        default:
          break;
      }
      motor->writeRegister(TMC5160_Reg::DRV_CONF, drvConf.value);
    }
  }

  Serial.println();
  Serial.println("Power stage tuning is done.");
  Serial.print("Final DRV_CONF register value: 0x");
  Serial.println(drvConf.value);
  Serial.print("DRVSTRENGTH: ");
  Serial.print(drvConf.drvstrength);
  Serial.print(", BBMTIME: ");
  Serial.print(drvConf.bbmtime);
  Serial.print(", BBMCLKS: ");
  Serial.println(drvConf.bbmclks);

  powerParams->drvStrength = drvConf.drvstrength;
  powerParams->bbmTime = drvConf.bbmtime;
  powerParams->bbmClks = drvConf.bbmclks;
}

void getPowerStageParams(TMC5160::PowerStageParameters *powerParams)
{
  while (Serial.available())
    Serial.read();

  Serial.print("MOSFET gate driver current DRVSTRENGTH ? (Default: ");
  Serial.print(powerParams->drvStrength);
  Serial.println(")");

  while (!Serial.available());
  if (isDigit(Serial.peek()))
    powerParams->drvStrength = constrain(Serial.parseInt(), 0, 3);

  while (Serial.available())
    Serial.read();

  Serial.print("Break before make time BBMTIME ? (Default: ");
  Serial.print(powerParams->bbmTime);
  Serial.println(")");

  while (!Serial.available());
  if (isDigit(Serial.peek()))
    powerParams->bbmTime = constrain(Serial.parseInt(), 0, 24);

  while (Serial.available())
    Serial.read();

  Serial.print("Break before make clock cycles BBMCLKS ? (Default: ");
  Serial.print(powerParams->bbmClks);
  Serial.println(")");

  while (!Serial.available());
  if (isDigit(Serial.peek()))
    powerParams->bbmClks = constrain(Serial.parseInt(), 0, 15);

  Serial.print("Using DRVSTRENGTH: ");
  Serial.print(powerParams->drvStrength);
  Serial.print(", BBMTIME: ");
  Serial.print(powerParams->bbmTime);
  Serial.print(", BBMCLKS: ");
  Serial.println(powerParams->bbmClks);
}

void tuneMotorCurrent(TMC5160::MotorParameters *motorParams)
{
  while (Serial.available())
    Serial.read();

  float senseResistor = 0.05;
  Serial.print("Sense resistors value ? (Default: ");
  Serial.print(senseResistor);
  Serial.println(" ohms)");

  while (!Serial.available());
  if (isDigit(Serial.peek()))
    senseResistor = Serial.parseFloat();

  float maxRmsCurrent = 0.33 / (senseResistor * sqrt(2.0));
  Serial.print(senseResistor);
  Serial.print("ohms => Max RMS current ");
  Serial.print(maxRmsCurrent);
  Serial.println("A");

  while (Serial.available())
    Serial.read();

  float motorCurrent = 1.5;
  Serial.print("Motor phase current ? (Default: ");
  Serial.print(motorCurrent);
  Serial.println("A)");

  while (!Serial.available());
  if (isDigit(Serial.peek()))
    motorCurrent = Serial.parseFloat();

  motorParams->globalScaler = constrain(floor(motorCurrent * 256.0 / maxRmsCurrent), 32, 256);
  Serial.print(motorCurrent);
  Serial.print("A => global scaler set to ");
  Serial.println(motorParams->globalScaler);

  motorParams->irun = constrain(floor(motorCurrent * 31.0 / (maxRmsCurrent * (float)motorParams->globalScaler / 256.0)), 0, 31);
  Serial.print("I_RUN set to ");
  Serial.println(motorParams->irun);

  while (Serial.available())
    Serial.read();

  float motorCurrentReduction = 0.5;
  Serial.print("Motor current standstill reduction ? Set to 0 for freewheeling / passive braking (Default: ");
  Serial.print(motorCurrentReduction);
  Serial.println(")");

  while (!Serial.available());
  if (isDigit(Serial.peek()))
    motorCurrentReduction = constrain(Serial.parseFloat(), 0.0, 1.0);

  motorParams->ihold = constrain(floor((float)motorParams->irun * motorCurrentReduction), 0, 31);
  Serial.print("I_HOLD set to ");
  Serial.println(motorParams->ihold);

  if (motorParams->ihold == 0)
  {
    Serial.println("Freewheeling options : 'n' for normal operation, 'f' for freewheeling, 'l' for passive braking using LS drivers");
    switch (readCommandToken("nfl", 3, 0))
    {
      case 'n': motorParams->freewheeling = TMC5160_Reg::FREEWHEEL_NORMAL; break;
      case 'f': motorParams->freewheeling = TMC5160_Reg::FREEWHEEL_ENABLED; break;
      case 'l': motorParams->freewheeling = TMC5160_Reg::FREEWHEEL_SHORT_LS; break;
    }
  }

  Serial.println("Motor current tuning is done.");
}


void tuneStealthChop(TMC5160::PowerStageParameters *powerParams, TMC5160::MotorParameters *motorParams)
{
  Serial.println();
  while (Serial.available())
    Serial.read();
  Serial.println("/!\\ WARNING : the determined parameters are only valid for this motor, at this operating voltage.");
  Serial.println("Please set the desired operating voltage now.");
  Serial.println("The tuning procedure will start a medium speed motion for at least 2 full turns.");
  Serial.println("Make sure that there is no mechanical issue then press any key.");
  while (!Serial.available());

  //TODO TBL !! /!\ lower current limit, depends on motor and operating voltage
  //TODO PWM_REG ?

  Serial.println();
  Serial.println("Starting operation with the determined current and default stealthChop values.");

  //Reset short detection if necessary
  TMC5160_Reg::SHORT_CONF_Register shortConf = {0};
  shortConf.s2vs_level = 6;
  shortConf.s2g_level = 6;
  shortConf.shortfilter = 1;
  motor->writeRegister(TMC5160_Reg::SHORT_CONF, shortConf.value);

  motor->begin(*powerParams, *motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  motor->writeRegister(TMC5160_Reg::TPOWERDOWN, 32); //For Phase #1, the motor needs to be at standstill, with the run current.

  Serial.println("Starting automatic tuning phase #1...");
  motor->setAcceleration(200);
  motor->setRampMode(TMC5160::VELOCITY_MODE);
  motor->writeRegister(TMC5160_Reg::XACTUAL, 0);
  motor->setMaxSpeed(0.025);

  TMC5160_Reg::PWM_AUTO_Register pwmAuto = { 0 };
  TMC5160_Reg::PWM_SCALE_Register pwmScale = { 0 };

  do {
    pwmAuto.value = motor->readRegister(TMC5160_Reg::PWM_AUTO);
    pwmScale.value = motor->readRegister(TMC5160_Reg::PWM_SCALE);
    Serial.print("PWM_SCALE_AUTO: ");
    Serial.print(pwmScale.pwm_scale_auto);
    Serial.print(", PWM_OFS_AUTO: ");
    Serial.println(pwmAuto.pwm_ofs_auto);
    delay(100);
  } while ( !(abs((int32_t) motor->readRegister(TMC5160_Reg::XACTUAL)) > 10 && pwmScale.pwm_scale_auto == 0));  //Wait at least 10 steps.

  motor->stop();
  pwmAuto.value = motor->readRegister(TMC5160_Reg::PWM_AUTO);
  pwmScale.value = motor->readRegister(TMC5160_Reg::PWM_SCALE);
  Serial.print("Phase #1 should be OK. PWM_OFS_AUTO: ");
  Serial.println(pwmAuto.pwm_ofs_auto);

  Serial.println();
  Serial.println("Starting automatic tuning phase #2...");
  motor->setAcceleration(400);

  float tuningMaxSpeed = 200.0;// 200 steps / sec => 60RPM on a 200 steps motor
  motor->setMaxSpeed(tuningMaxSpeed);

  TMC5160_Reg::RAMP_STAT_Register rampStat = { 0 };

  do {
    pwmAuto.value = motor->readRegister(TMC5160_Reg::PWM_AUTO);
    pwmScale.value = motor->readRegister(TMC5160_Reg::PWM_SCALE);
    rampStat.value = motor->readRegister(TMC5160_Reg::RAMP_STAT);

    Serial.print("PWM_SCALE_AUTO: ");
    Serial.print(pwmScale.pwm_scale_auto);
    Serial.print(", PWM_SCALE_SUM: ");
    Serial.print(pwmScale.pwm_scale_sum);
    Serial.print(", PWM_GRAD_AUTO: ");
    Serial.println(pwmAuto.pwm_grad_auto);

    if (rampStat.velocity_reached)
    {
      if (pwmScale.pwm_scale_sum == 255 || pwmScale.pwm_scale_sum >= 4 * pwmAuto.pwm_ofs_auto)
      {
        tuningMaxSpeed /= 2.0;
        motor->setMaxSpeed(tuningMaxSpeed);
        Serial.println("Decreasing max speed.");
      }
      else if (pwmScale.pwm_scale_sum <= floor(1.5 * (float)pwmAuto.pwm_ofs_auto))
      {
        tuningMaxSpeed *= 1.5;
        motor->setMaxSpeed(tuningMaxSpeed);
        Serial.println("Increasing max speed.");
      }
    }

    delay(100);
  } while ( !(pwmScale.pwm_scale_auto == 0 && rampStat.velocity_reached));
  motor->stop();

  pwmAuto.value = motor->readRegister(TMC5160_Reg::PWM_AUTO);
  Serial.print("Phase #2 is finished. PWM_GRAD_AUTO: ");
  Serial.println(pwmAuto.pwm_grad_auto);

  //TODO add timeout and try again the whole procedure if necessary.

  Serial.println();
  Serial.println("/!\\ WARNING : You must run this tuning procedure again if the operating voltage changes.");
  //TODO it is advised to run at least the partial procedure during startup
  motorParams->pwmOfsInitial = pwmAuto.pwm_ofs_auto;
  motorParams->pwmGradInitial = pwmAuto.pwm_grad_auto;
}


void getStealthChopParams(TMC5160::MotorParameters *motorParams)
{
  while (Serial.available())
    Serial.read();

  Serial.print("Initial PWM_OFS value ? (Default: ");
  Serial.print(motorParams->pwmOfsInitial);
  Serial.println(")");

  while (!Serial.available());
  if (isDigit(Serial.peek()))
    motorParams->pwmOfsInitial = constrain(Serial.parseInt(), 0, 255);

  while (Serial.available())
    Serial.read();

  Serial.print("Initial PWM_GRAD value ? (Default: ");
  Serial.print(motorParams->pwmGradInitial);
  Serial.println(")");

  while (!Serial.available());
  if (isDigit(Serial.peek()))
    motorParams->pwmGradInitial = constrain(Serial.parseInt(), 0, 255);


  Serial.print("Using PWM_OFS: ");
  Serial.print(motorParams->pwmOfsInitial);
  Serial.print(", PWM_GRAD: ");
  Serial.println(motorParams->pwmGradInitial);
}


void startPowerStageTroubleshooting()
{
  Serial.println();
  Serial.println("Do you want to troubleshoot the power stage ? This will disable then enable again while you look at the scope.");
  if (readCommandToken("yn", 2, 1))
  {
    while (true)
    {
      while (Serial.available())
        Serial.read();
      Serial.println();
      Serial.println("/!\\ Make sure that there is no real short condition and use a current limited power supply !");
      Serial.println("Press any key to disable then enable again the driver.");
      while (!Serial.available());

      //TODO add to library !
      TMC5160_Reg::CHOPCONF_Register chopConf = {0};
      chopConf.value = motor->readRegister(TMC5160_Reg::CHOPCONF);
      chopConf.toff = 0; //Disable driver
      motor->writeRegister(TMC5160_Reg::CHOPCONF, chopConf.value);

      delay(10);

      chopConf.toff = 5; //default
      motor->writeRegister(TMC5160_Reg::CHOPCONF, chopConf.value);

      delay(100);

      Serial.println("Driver status: ");
      printDriverStatus();
    }
  }
  else
  {
    while (true);
  }
}