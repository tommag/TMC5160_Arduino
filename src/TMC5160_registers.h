/*
MIT License

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

#ifndef TMC5160_REGISTERS_H
#define TMC5160_REGISTERS_H

#include <Bitfield.h>

namespace TMC5160_Reg {

  /* Register addresses */
  enum {
    /* General configuration registers */
    GCONF             = 0x00, // Global configuration flags
    GSTAT             = 0x01, // Global status flags
    IFCNT             = 0x02, // UART transmission counter
    SLAVECONF         = 0x03, // UART slave configuration
    IO_INPUT_OUTPUT   = 0x04, // Read input / write output pins
    X_COMPARE         = 0x05, // Position comparison register
    OTP_PROG          = 0x06, // OTP programming register
    OTP_READ          = 0x07, // OTP read register
    FACTORY_CONF      = 0x08, // Factory configuration (clock trim)
    SHORT_CONF        = 0x09, // Short detector configuration
    DRV_CONF          = 0x0A, // Driver configuration
    GLOBAL_SCALER     = 0x0B, // Global scaling of motor current
    OFFSET_READ       = 0x0C, // Offset calibration results

    /* Velocity dependent driver feature control registers */
    IHOLD_IRUN        = 0x10, // Driver current control
    TPOWERDOWN        = 0x11, // Delay before power down
    TSTEP             = 0x12, // Actual time between microsteps
    TPWMTHRS          = 0x13, // Upper velocity for stealthChop voltage PWM mode
    TCOOLTHRS         = 0x14, // Lower threshold velocity for switching on smart energy coolStep and stallGuard feature
    THIGH             = 0x15, // Velocity threshold for switching into a different chopper mode and fullstepping

    /* Ramp generator motion control registers */
    RAMPMODE          = 0x20, // Driving mode (Velocity, Positioning, Hold)
    XACTUAL           = 0x21, // Actual motor position
    VACTUAL           = 0x22, // Actual  motor  velocity  from  ramp  generator
    VSTART            = 0x23, // Motor start velocity
    A_1               = 0x24, // First acceleration between VSTART and V1
    V_1               = 0x25, // First acceleration/deceleration phase target velocity
    AMAX              = 0x26, // Second acceleration between V1 and VMAX
    VMAX              = 0x27, // Target velocity in velocity mode
    DMAX              = 0x28, // Deceleration between VMAX and V1
    D_1               = 0x2A, // Deceleration between V1 and VSTOP
      //Attention:  Do  not  set  0  in  positioning  mode, even if V1=0!
    VSTOP             = 0x2B, // Motor stop velocity
      //Attention: Set VSTOP > VSTART!
      //Attention:  Do  not  set  0  in  positioning  mode, minimum 10 recommend!
    TZEROWAIT         = 0x2C, // Waiting time after ramping down to zero velocity before next movement or direction inversion can start.
    XTARGET           = 0x2D, // Target position for ramp mode

    /* Ramp generator driver feature control registers */
    VDCMIN            = 0x33, // Velocity threshold for enabling automatic commutation dcStep
    SW_MODE           = 0x34, // Switch mode configuration
    RAMP_STAT         = 0x35, // Ramp status and switch event status
    XLATCH            = 0x36, // Ramp generator latch position upon programmable switch event

    /* Encoder registers */
    ENCMODE           = 0x38, // Encoder configuration and use of N channel
    X_ENC             = 0x39, // Actual encoder position
    ENC_CONST         = 0x3A, // Accumulation constant
    ENC_STATUS        = 0x3B, // Encoder status information
    ENC_LATCH         = 0x3C, // Encoder position latched on N event
    ENC_DEVIATION     = 0x3D, // Maximum number of steps deviation between encoder counter and XACTUAL for deviation warning

    /* Motor driver registers */
    MSLUT_0_7         = 0x60, // Microstep table entries. Add 0...7 for the next registers
    MSLUTSEL          = 0x68, // Look up table segmentation definition
    MSLUTSTART        = 0x69, // Absolute current at microstep table entries 0 and 256
    MSCNT             = 0x6A, // Actual position in the microstep table
    MSCURACT          = 0x6B, // Actual microstep current
    CHOPCONF          = 0x6C, // Chopper and driver configuration
    COOLCONF          = 0x6D, // coolStep smart current control register and stallGuard2 configuration
    DCCTRL            = 0x6E, // dcStep automatic commutation configuration register
    DRV_STATUS        = 0x6F, // stallGuard2 value and driver error flags
    PWMCONF           = 0x70, // stealthChop voltage PWM mode chopper configuration
    PWM_SCALE         = 0x71, // Results of stealthChop amplitude regulator.
    PWM_AUTO          = 0x72, // Automatically determined PWM config values
    LOST_STEPS        = 0x73  // Number of input steps skipped due to dcStep. only with SD_MODE = 1
  };

  /* Register bit fields */

  /* General configuration register */
  union GCONF_Register {
    uint32_t value;
    BitField< 0> recalibrate; // Zero crossing recalibration during driver disable
    BitField< 1> faststandstill; // Timeout for step execution until standstill detection
    BitField< 2> en_pwm_mode; // Enable stealthChop voltage PWM mode
    BitField< 3> multistep_filt; // Enable step input filtering for stealthChop optimization with external step source
    BitField< 4> shaft; // Normal / inverse motor direction
    BitField< 5> diag0_error; // Enable DIAG0 active on driver errors: Over temperature (ot), short to GND (s2g), undervoltage chargepump (uv_cp)
    BitField< 6> diag0_otpw; // Enable DIAG0 active on driver over temperature prewarning (otpw)
    BitField< 7> diag0_stall_step; // SD_MODE=1: enable DIAG0 active on motor stall. SD_MODE=0: enable DIAG0 as STEP output
    BitField< 8> diag1_stall_dir; // SD_MODE=1: enable DIAG1 active on motor stall. SD_MODE=0: enable DIAG1 as DIR output
    BitField< 9> diag1_index; // Enable DIAG1 active on index position
    BitField<10> diag1_onstate; // Enable DIAG1 active when chopper is on
    BitField<11> diag1_steps_skipped; // Enable output toggle when steps are skipped in dcStep mode
    BitField<12> diag0_int_pushpull; // Enable SWN_DIAG0 push pull output
    BitField<13> diag1_poscomp_pushpull; // Enable SWP_DIAG1 push pull output
    BitField<14> small_hysteresis; // Set small hysteresis for step frequency comparison
    BitField<15> stop_enable; // Enable emergency stop: ENCA_DCIN stops the sequencer when tied high
    BitField<16> direct_mode; // Enable direct motor coil current and polarity control
    BitField<17> test_mode; // Not for normal use
  };

  /* Global status flags */
  union GSTAT_Register {
    uint32_t value;
    BitField<0> reset; // Indicates that the IC has been reset since the last read access to GSTAT
    BitField<1> drv_err; // Indicates that the driver has been shut down due to overtemperature or short circuit detection since the last read access
    BitField<2> uv_cp; // Indicates an undervoltage on the charge pump. The driver is disabled in this case.
  };

  /* UART slave configuration */
  union SLAVECONF_Register {
    uint32_t value;
    BitField<0, 8> slaveaddr; // Address of unit for the UART interface. The address becomes incremented by one when the external address pin NAI is active.
    BitField<8, 4> senddelay; // Number of bit times before replying to a register read in UART mode. Set > 1 with multiple slaves.
  };

  /* Read input pins */
  union IOIN_Register {
    uint32_t value;
    BitField<0> refl_step;
    BitField<1> refr_dir;
    BitField<2> encb_dcen_cfg4;
    BitField<3> enca_dcin_cfg5;
    BitField<4> drv_enn;
    BitField<5> enc_n_dco_cfg6;
    BitField<6> sd_mode; // 1=External step and dir source
    BitField<7> swcomp_in;
    BitField<24, 8> version;
  };

  /* OTP programming */
  union OTP_PROG_Register {
    uint32_t value;
    BitField<0, 3> otpbit; // Selection of OTP bit to be programmed
    BitField<4, 2> otpbyte; // Selection of OTP byte. Set to 00
    BitField<8, 8> otpmagic; // Set to 0xBD to program.
  };

  /* OTP configuration memory */
  union OTP_READ_Register {
    uint32_t value;
    BitField<0, 5> otp_fclktrim; // Reset default for FCLKTRIM (frequency source calibration)
    BitField<5> otp_S2_level; // Reset default for Short detection levels
    BitField<6> otp_bbm; // Reset default for DRVCONF.BBMCLKS
    BitField<7> otp_tbl; // Reset default for TBL
  };

  /* Short detector configuration */
  union SHORT_CONF_Register {
    uint32_t value;
    BitField< 0, 4> s2vs_level; // Short to VS detector for low side FETs sensitivity
    BitField< 8, 4> s2g_level; // Short to GND detector for high side FETs sensitivity
    BitField<16, 2> shortfilter; // Spike filtering bandwidth for short detection
    BitField<18>    shortdelay; // Short detection delay
  };

  /* Driver configuration */
  union DRV_CONF_Register {
    uint32_t value;
    BitField< 0, 5> bbmtime; // Break before make delay (0 to 24)
    BitField< 8, 4> bbmclks; // Digital BBM Time in clock cycles
    BitField<16, 2> otselect; // Selection of over temperature level for bridge disable
    BitField<18, 2> drvstrength; // Selection of gate drivers current
    BitField<20, 2> filt_isense; // Filter time constant of sense amplifier to suppress ringing and coupling from second coil operation
  };

  /* Offset calibration result */
  union OFFSET_READ_Register {
    uint32_t value;
    BitField<0, 8> phase_b;
    BitField<8, 8> phase_a;
  };

  /* Driver current control */
  union IHOLD_IRUN_Register {
    uint32_t value;
    BitField< 0, 5> ihold; // Standstill current (0=1/32...31=32/32)
    BitField< 8, 5> irun; // Motor run current (0=1/32...31=32/32). Should be between 16 and 31 for best performance.
    BitField<16, 4> iholddelay; // Controls the number of clock cycles for motor power down when entering standstill
  };

  /* Switch mode configuration */
  union SW_MODE_Register {
    uint32_t value;
    BitField< 0> stop_l_enable; // Enable automatic motor stop during active left reference switch input
    BitField< 1> stop_r_enable; // Enable automatic motor stop during active right reference switch input
    BitField< 2> pol_stop_l; // Sets the active polarity of the left reference switch input (1=inverted, low active, a low level on REFL stops the motor)
    BitField< 3> pol_stop_r; // Sets the active polarity of the right reference switch input (1=inverted, low active, a low level on REFR stops the motor
    BitField< 4> swap_lr; // Swap the left and the right reference switch inputs
    BitField< 5> latch_l_active; // Activate latching of the position to XLATCH upon an active going edge on REFL
    BitField< 6> latch_l_inactive; // Activate latching of the position to XLATCH upon an inactive going edge on REFL
    BitField< 7> latch_r_active; // Activate latching of the position to XLATCH upon an active going edge on REFR
    BitField< 8> latch_r_inactive; // Activate latching of the position to XLATCH upon an inactive going edge on REFR
    BitField< 9> en_latch_encoder; // Latch encoder position to ENC_LATCH upon reference switch event
    BitField<10> sg_stop; // Enable stop by stallGuard2 (also available in dcStep mode). Disable to release motor after stop event.
    BitField<11> en_softstop; // Enable soft stop upon a stop event (uses the deceleration ramp settings)
  };

  /* Ramp status and switch event status */
  union RAMP_STAT_Register {
    uint32_t value;
    BitField< 0> status_stop_l; // Reference switch left status (1=active)
    BitField< 1> status_stop_r; // Reference switch right status (1=active)
    BitField< 2> status_latch_l; // Latch left ready (enable position latching using SWITCH_MODE settings latch_l_active or latch_l_inactive)
    BitField< 3> status_latch_r; // Latch right ready (enable position latching using SWITCH_MODE settings latch_r_active or latch_r_inactive)
    BitField< 4> event_stop_l; // Signals an active stop left condition due to stop switch.
    BitField< 5> event_stop_r; // Signals an active stop right condition due to stop switch.
    BitField< 6> event_stop_sg; // Signals an active StallGuard2 stop event.
    BitField< 7> event_pos_reached; // Signals that the target position has been reached (position_reached becoming active).
    BitField< 8> velocity_reached; // Signals that the target velocity is reached.
    BitField< 9> position_reached; // Signals that the target position is reached.
    BitField<10> vzero; // Signals that the actual velocity is 0.
    BitField<11> t_zerowait_active; // Signals that TZEROWAIT is active after a motor stop. During this time, the motor is in standstill.
    BitField<12> second_move; // Signals that the automatic ramp required moving back in the opposite direction
    BitField<13> status_sg; // Signals an active stallGuard2 input from the coolStep driver or from the dcStep unit, if enabled.
  };

  /* Encoder configuration and use of N channel */
  union ENCMODE_Register {
    uint32_t value;
    BitField< 0> pol_A; // Required A polarity for an N channel event (0=neg., 1=pos.)
    BitField< 1> pol_B; // Required B polarity for an N channel event (0=neg., 1=pos.)
    BitField< 2> pol_N; // Defines active polarity of N (0=low active, 1=high active)
    BitField< 3> ignore_AB; // Ignore A and B polarity for N channel event
    BitField< 4> clr_cont; // Always latch or latch and clear X_ENC upon an N event
    BitField< 5> clr_once; // Latch or latch and clear X_ENC on the next N event following the write access
    BitField< 6, 2> sensitivity; // N channel event sensitivity
    BitField< 8> clr_enc_x; // Clear encoder counter X_ENC upon N-event
    BitField< 9> latch_x_act; // Also latch XACTUAL position together with X_ENC.
    BitField<10> enc_sel_decimal; // Encoder prescaler divisor binary mode (0) / decimal mode (1)
  };

  /* Encoder status information */
  union ENC_STATUS_Register {
    uint32_t value;
    BitField<0> n_event; // N event detected
    BitField<1> deviation_warn; //Deviation between X_ACTUAL and X_ENC detected
  };

  /* Chopper and driver configuration */
  union CHOPCONF_Register {
    uint32_t value;
    BitField< 0, 4> toff; // Off time setting controls duration of slow decay phase. 0 : Driver disabled.
    BitField< 4, 3> hstrt_tfd; // chm=0: hysteresis start value HSTRT, chm=1: fast decay time setting bits 0:2
    BitField< 7, 4> hend_offset; // chm=0: hysteresis low value HEND, chm=1: sine wave offset
    BitField<11>    tfd_3; // chm=1: fast decay time setting bit 3
    BitField<12>    disfdcc; // chm=1: disable current comparator usage for termi- nation of the fast decay cycle
    BitField<13>    rndtf; // enable random modulation of chopper TOFF time
    BitField<14>    chm; // Chopper mode (0=standard - spreadCycle ; 1=constant off time with fast decay time)
    BitField<15, 2> tbl; // Comparator blank time select.
    BitField<17>    vsense; // Select resistor voltage sensitivity (0=Low sensitivity ; 1=High sensitivity)
    BitField<18>    vhighfs; // Enable switching to fullstep when VHIGH is exceeded.
    BitField<19>    vhighchm; // Enable switching to chm=1 and fd=0 when VHIGH is exceeded
    BitField<20, 4> tpfd; // passive fast decay time
    BitField<24, 4> mres; // Set microstep resolution
    BitField<28>    intpol; // Enable interpolation to 256 microsteps with an external motion controller
    BitField<29>    dedge; // Enable double edge step pulses
    BitField<30>    diss2g; // Disable short to GND protection
    BitField<31>    diss2vs; // Disable short to supply protection
  };

  /* coolStep smart current control and stallGuard2 configuration */
  union COOLCONF_Register {
    uint32_t value;
    BitField< 0, 4> semin; // Minimum stallGuard2 value for smart current control and smart current enable
    BitField< 5, 2> seup; // Current increment step width
    BitField< 8, 4> semax; // stallGuard2 hysteresis value for smart current control
    BitField<13, 2> sedn; // Current decrement step speed
    BitField<15>    seimin; // Minimum current for smart current control
    BitField<16, 7> sgt; // stallGuard2 threshold value
    BitField<24>    sfilt; // Enable stallGuard2 filter
  };

  /* dcStep automatic commutation configuration register */
  union DCCTRL_Register {
    uint32_t value;
    BitField< 0, 10> dc_time; // Upper PWM on time limit for commutation
    BitField<16,  8> dc_sg; // Max. PWM on time for step loss detection using dcStep stallGuard2 in dcStep mode.
  };

  /* stallGuard2 value and driver error flags */
  union DRV_STATUS_Register {
    uint32_t value;
    BitField< 0, 9> sg_result; // stallGuard2 result or motor temperature estimation in stand still
    BitField<12>    s2vsa; // short to supply indicator phase A
    BitField<13>    s2vsb; // short to supply indicator phase B
    BitField<14>    stealth; // stealthChop indicator
    BitField<15>    fsactive; // Full step active indicator
    BitField<16, 5> cs_actual; // Actual motor current / smart energy current
    BitField<24>    stallguard; // stallGuard2 status
    BitField<25>    ot; // overtemperature flag
    BitField<26>    otpw; // overtemperature pre- warning flag
    BitField<27>    s2ga; // short to ground indicator phase A
    BitField<28>    s2gb; // short to ground indicator phase B
    BitField<29>    ola; // open load indicator phase A
    BitField<30>    olb; // open load indicator phase B
    BitField<31>    stst; // standstill indicator
  };

  /* stealthChop voltage PWM mode chopper configuration */
  union PWMCONF_Register {
    uint32_t value;
    BitField< 0, 8> pwm_ofs; // User defined PWM amplitude (offset)
    BitField< 8, 8> pwm_grad; // User defined PWM amplitude (gradient)
    BitField<16, 2> pwm_freq; // PWM frequency selection
    BitField<18>    pwm_autoscale; // Enable PWM automatic amplitude scaling
    BitField<19>    pwm_autograd; // PWM automatic gradient adaptation
    BitField<20, 2> freewheel; // Stand still option when motor current setting is zero (I_HOLD=0).
    BitField<24, 4> pwm_reg; // Regulation loop gradient
    BitField<28, 4> pwm_lim; // PWM automatic scale amplitude limit when switching on
  };

  /* Results of stealthChop amplitude regulator */
  union PWM_SCALE_Register {
    uint32_t value;
    BitField< 0, 8> pwm_scale_sum; // Actual PWM duty cycle
    BitField<16, 9> pwm_scale_auto; // Result of the automatic amplitude regulation based on current measurement.
  };

  /* stealthChop automatically generated values read out */
  union PWM_AUTO_Register {
    uint32_t value;
    BitField< 0, 8> pwm_ofs_auto; // Automatically determined offset value
    BitField<16, 8> pwm_grad_auto; // Automatically determined gradient value
  };

  /* Register field values */
  enum RAMPMODE_Values {
    POSITIONING_MODE  = 0x00,	// using all A, D and V parameters
    VELOCITY_MODE_POS = 0x01,	// positive VMAX, using AMAX acceleration
    VELOCITY_MODE_NEG = 0x02,	// negative VMAX, using AMAX acceleration
    HOLD_MODE         = 0x03	// velocity remains unchanged, unless stop event occurs
  };

  enum PWMCONF_freewheel_Values {
    FREEWHEEL_NORMAL   = 0x00, // Normal operation
    FREEWHEEL_ENABLED  = 0x01, // Freewheeling
    FREEWHEEL_SHORT_LS = 0x02, // Coil shorted using LS drivers
    FREEWHEEL_SHORT_HS = 0x03  // Coil shorted using HS drivers
  };

  enum ENCMODE_sensitivity_Values {
    ENCODER_N_NO_EDGE       = 0x00, // N channel active while the N event is valid
    ENCODER_N_RISING_EDGE   = 0x01, // N channel active when the N event is activated
    ENCODER_N_FALLING_EDGE  = 0x02, // N channel active when the N event is de-activated
    ENCODER_N_BOTH_EDGES    = 0x03  // N channel active on N event activation and de-activation
  };
}

#endif // TMC5160_REGISTERS_H
