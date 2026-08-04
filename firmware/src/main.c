/*
 * Hydra Delay Taptempo Buddy - a "FV1 Buddy" adaptation for [Hydra multi-headed can delay](https://www.pedalpcb.com/product/pcb238/) pedal.
 * This code is an ATtiny402 Tap Tempo module firmware for the Spin Semiconductors FV1 DSP used in Hydra delay - calibreated for it.
 *
 * August 2026
 * by Vlastimil Elias from Elvis Alive Tone
 *
 * MODs (marked in source code by `MOD:` comment):
 * - no "48kHz Clock Output" output (and related TCB0 timer config), pin is used for "Momentary Tap Tempo Button Input" instead
 * - Reset/UPDI pin stays unused/unchanged (was "Momentary Tap Tempo Button Input"), no high voltage programming necessary
 * - "Tempo Division Switch" functionality changed to select tempo tapping for head 2 (eight head) or 4 (quarter head). Implementation changed from ADC to digital input pin.
 * - `calib` function removed, Hydra's max and min delay times are hardcoded
 * - `Speed Pot` and "Output" functionality reversed when compared to the original "Time" functionality, to adjust to the Hydra, where higher pot value is higher speed -> shorter delay
 * - corrected computiong of `pwm` value from `divtempo`
 * - main loop processing time optimalization (debounce() called only once) and bunch of patches and changes
 * - changed EEPROM storing/retrieving code to work (`avr/eeprom.h` provided code doesn't work!)
 * - code comments added and improved
 *
 * Pinout (changed):
 * 1: VDD
 * 2: Momentary Tap Tempo Button Digital Input // MOD: used for "48kHz Clock Output" before
 * 3: LED+ Output
 * 4: Speed Potentiometer Analog Input // MOD: was "Time" before, reversed
 * 5: Tempo Division Switch (On-On) Digitsal Input // MOD: was analog before for three states
 * 6: Reset/UPDI, MOD: used for "Momentary Tap Tempo Button Input" before
 * 7: PWM Output // MOD: reversed for "Speed" pot
 * 8: GND
 *
 *
 * From the original FV1 Buddy code:
 *
 * January 2022
 * by Antoine Ricoux for Electric Canary
 *
 *  https://electric-canary.com/FV1Buddy
 *  support@electric-canary.com
 *
 * This code is shared under a BY-NC-SA Creative Commons License
 * Go here for complete license : https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 20000000UL
#include <util/delay.h>
#include <stdlib.h>

// MOD: `#define CLK_PIN 6` removed
#define PWM_PIN 3 // max voltage is shortest delay (max Speed)!
#define TAP_PIN 6 // MOD: changed from 0 to 6 so we do not need to reconfigure UPDI pin
#define LED_PIN 7
#define POT_PIN 1           // higher voltage higher Speed -> shorter delay!
#define DIV_PIN 2           // connected to ground - head 2, on VCC by pull-up res or wire - head 4
#define DEBOUNCE_TIME 900   // Tap tempo button debounce time [us]
#define POT_MAX_VALUE 0x3FF // 10bit ADC max value to convert pot

// MOD: changed EEPROM addresses to point into EEPROM
const uint16_t EEPROM_TAP = 0x1400;   // one byte to store `tap` variable
const uint16_t EEPROM_TEMPO = 0x1401; // two bytes to store `mstempo` variable

// MOD: constants for `pwm` value computation from `divtempo`
const uint16_t c_pwm_max = 999;
const uint16_t c_delay_max = 920;   // max delay from Hydra on head 4 [ms] - Speed pot on minimum
const uint16_t c_delay_min = 150;   // min delay from Hydra on head 4 [ms] - Speed pot on maximum
const uint16_t c_delay_range = 770; // Computed as `c_delay_max` - `c_delay_min` [ms]

// MOD: store firmware revision info into uC binary code for further reference.
// Must be volatile to be kept by compiler.
// Starts by `rev ` to find it more easily in the binary.
volatile char revision[] = "rev_1";

volatile uint16_t pot;       // current Pot value is stored here from ADC by interrupt handler
volatile uint16_t pwm = 500; // current value of PWM output (between 0 and PWM_MAX) to be set into TCA in its interrupt handler
volatile uint16_t ms;        // time [ms] counter for Tap button pressed length and delayed `tap` reset in EEPROM respectively. Incremented in TCA interrupt.
volatile uint16_t ledms;     // time [ms] counter for LED blinking. Incremented in TCA interrupt.

// This set of fuses will deactivate the reset, it will be impossible to reprogram the chip without high voltage programming
// MOD: commented out as we do not need to change fuses, and it is even done another way in platform.io
// FUSES = {
// .WDTCFG = 0x00,  // WDTCFG {PERIOD=OFF, WINDOW=OFF} - DEFAULT VALUE
// .BODCFG = 0x00,  // BODCFG {SLEEP=DIS, ACTIVE=DIS, SAMPFREQ=1KHz, LVL=BODLEVEL0} - DEFAULT VALUE
// .OSCCFG = 0x02,  // OSCCFG {FREQSEL=20MHZ, OSCLOCK=CLEAR} - DEFAULT VALUE
// .TCD0CFG = 0x00, // TCD0CFG {CMPA=CLEAR, CMPB=CLEAR, CMPC=CLEAR, CMPD=CLEAR, CMPAEN=CLEAR, CMPBEN=CLEAR, CMPCEN=CLEAR, CMPDEN=CLEAR} - DEFAULT VALUE
// .SYSCFG0 = 0xF6, // SYSCFG0 {EESAVE=CLEAR, RSTPINCFG=GPIO, CRCSRC=NOCRC} // MOD: RSTPINCFG config changed from GPIO to UPDI, so flag value changed from 0xF2 to 0xF6 which is DEFAULT VALUE
// .SYSCFG1 = 0x07, // SYSCFG1 {SUT=64MS} - DEFAULT VALUE
// .APPEND = 0x00,  // APPEND {APPEND=User range:  0x0 - 0xFF} - DEFAULT VALUE
// .BOOTEND = 0x00, // BOOTEND {BOOTEND=User range:  0x0 - 0xFF} - DEFAULT VALUE
//};

void IO_Init(void)
{
    // PWM & LED as outputs
    PORTA.DIRSET = (1 << PWM_PIN) | (1 << LED_PIN);
    // pull up for div switch & tap button
    PORTA_PIN6CTRL |= PORT_PULLUPEN_bm;         // Tap tempo button input pin // MOD: changed from 0 to 6
    PORTA_PIN2CTRL |= PORT_PULLUPEN_bm;         // Division switch pin // MOD: changed from ADC to digital pin only
    PORTA_PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc; // Pot pin
}

// configure ADC
void ADC_Config(void)
{
    // reference to VCC with the right sampling capacitor
    ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV32_gc;
    // ADC0.CTRLB = ADC_SAMPNUM_ACC32_gc;
    // Start with ADC channel 1 for Tempo Pot
    ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc; // MOD: there was `ADC_MUXPOS_AIN2_gc` which is not channel 1
    // enable result ready interrupt
    ADC0.INTCTRL = ADC_RESRDY_bm;
    // 10bits freerun & enable
    ADC0.CTRLA |= ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc | ADC_FREERUN_bm;
    // Start conversion
    ADC0.COMMAND = ADC_STCONV_bm;
}

// interrupt handler for ADC
ISR(ADC0_RESRDY_vect)
{
    switch (ADC0.MUXPOS)
    {
    case ADC_MUXPOS_AIN1_gc:
        pot = ADC0.RES; // MOD: there was `divsw` read, 'pot' was read from second channel, which is incorrect!
        break;

    default:
        ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
        break;
    }

    // Clear interrupt flag
    ADC0.INTFLAGS = ADC_RESRDY_bm;
}

// configure ms timer for tap tempo
void TCA_Config(void)
{
    // Enable interrupts sei(); // MOD: removed here as it is called in main() before this method is called
    // Enable PWM output 0 (PA3), single slope PWM
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;
    // Set period to 1000 (20kHz for 20MHz clock)
    TCA0.SINGLE.PER = c_pwm_max;
    // set initial ratio to 50%
    TCA0.SINGLE.CMP0 = 500;
    // Enable overflow interrupt
    TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm;
    // Enable timer
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;
}

// interrupt handler for TCA timer - time counters and PWM output
ISR(TCA0_OVF_vect)
{
    // set pwm output value
    TCA0.SINGLE.CMP0BUF = pwm;

    // count ms
    static uint8_t count;
    count++;
    // this interrupt is called at 20kHz frequency
    if (count >= 20)
    {
        count = 0;
        ms++;
        ledms++;
    }

    // Clear interrupt flag
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

// MOD: removed TCB0 timer initialization used for CLK generation

// Tap button debounce with 900us time - returns 1 if Tap button is currently pressed
uint8_t debounce(void)
{
    if (!(PORTA.IN & (1 << TAP_PIN)))
    {
        _delay_us(DEBOUNCE_TIME);
        if (!(PORTA.IN & (1 << TAP_PIN)))
        {
            return (1);
        }
    }
    return (0);
}

// MOD: Div switch debounce with 900us time, conversion into `divmult` value
uint8_t divmult_from_diwsw(void)
{
    if (!(PORTA.IN & (1 << DIV_PIN)))
    {
        _delay_us(DEBOUNCE_TIME);
        if (!(PORTA.IN & (1 << DIV_PIN)))
        {
            // pin connected to ground - head 2
            return 2;
        }
    }
    // pin on VCC by pull-up res or wire - head 4
    return 1;
}

// one blink of LED used in calib() - this method takes 300ms!
void blink(void)
{
    PORTA.OUTTGL = (1 << LED_PIN);
    _delay_ms(150);
    PORTA.OUTTGL = (1 << LED_PIN);
    _delay_ms(150);
}

// MOD: persist NVM Page changes into EEPROM
void eeprom_persist(void)
{
    while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm)
        ; // Wait for EEPROM not busy.
    // disable interrupts as next code is CPU cycle sensitive
    cli();
    CPU_CCP = CCP_SPM_gc; // Unlock NVMCTRL.CTRLA write protection.
    NVMCTRL.CTRLA = NVMCTRL_CMD_PAGEERASEWRITE_gc;
    // reenable interrupts
    sei();
}

// MOD: correct calculation of the Speed `pwm` value from `pot` value and `c_delay_range` constant
uint16_t pot_to_pwm(uint16_t pot)
{
    return pot * (POT_MAX_VALUE / c_delay_range);
}

// MOD: compute correct of the Speed `pwm` value from `divtempo` [ms] and Hydra measured dealay time range constants.
// `pwm` value must be opposite to `divtempo` time, as higher pwm is higher Speed -> shorter delay!
uint16_t divtempo_to_pwm(uint16_t divtempo)
{
    return (uint16_t)(((uint32_t)(c_delay_max - divtempo) * c_pwm_max) / c_delay_range);
}

// MOD: check `divtempo` value is in range of `c_delay_min` .. `c_delay_max` and correct it if not
uint16_t divtempo_range(uint16_t divtempo)
{
    if (divtempo > c_delay_max)
    {
        return c_delay_max;
    }
    else if (divtempo < c_delay_min)
    {
        return c_delay_min;
    }
    else
    {
        return divtempo;
    }
}

int main(void)
{
    // Unlocking protected registers and setting main clock to 20MHz
    CPU_CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0;
    CLKCTRL.MCLKLOCK |= CLKCTRL_LOCKEN_bm;

    // init other values
    uint8_t currentstate = 0;     // Current state of Tap button in the cycle of main loop // MOD: added this variable to shorten main loop processing time
    uint8_t laststate = 0;        // Laststate of Tap button
    uint8_t nbtap = 0;            // Number of subsequent taps during current tapping
    uint8_t tapping = 0;          // Tapping currently in progress (1) or not (0)
    uint16_t divtempo = 500;      // Current delay time in [ms] - computed from `mstempo` and `divmult` switch state
    uint16_t previouspot;         // Previous Time Pot value to be able to detect change 0-1024
    uint8_t eeprom_reset_tap = 0; // Flag for delayed `tap` reset in EEPROM
    uint8_t updown = 0;           // direction for Ramp feature

    // MOD: read values from EEPROM
    while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm) // Wait for EEPROM not busy.
        ;
    uint8_t tap = *(uint8_t *)(EEPROM_TAP);         // Current control status - 1 for tap tempo, 0 for pot control
    uint16_t mstempo = *(uint16_t *)(EEPROM_TEMPO); // Currently tapped in tempo in ms
    // MOD: check values from EEPROM and patch them just in case
    if (mstempo < c_delay_min || mstempo > c_delay_max)
    {
        mstempo = 500;
    }
    if (tap < 0 || tap > 1)
    {
        tap = 0;
    }

    // enable interrupts
    sei();
    IO_Init();
    TCA_Config();
    ADC_Config();

    // wait for ADC etc
    _delay_ms(500);

    // Initialize from values stored in EEPROM
    uint8_t divmult = divmult_from_diwsw(); // MOD: read multiplier for tempo based on Div switch state
    if (tap == 1)
    {
        previouspot = pot;
        // MOD: compute current divtempo, patch mstempo if necessary
        divtempo = divtempo_range(mstempo * divmult);
        if (divtempo == c_delay_min || divtempo == c_delay_max)
        {
            mstempo = divtempo / divmult;
        }

        // MOD: correct calculation of the PWM
        pwm = divtempo_to_pwm(divtempo);
    }
    else
    {
        // set high value so code in main loop kicks-in and set real value
        previouspot = 60000;
    }

    // Main loop
    while (1)
    {

        // TEMPO DIV SWITCH handling
        // MOD: changed from ADC to normal input pin as two states is enough. And change is used during next taping, not immediately also.
        divmult = divmult_from_diwsw();

        // TIME POT handling
        // if pot move of more than 5%, changing to pot control // MOD: 5% instead of 7%
        if ((tap == 1 && tapping == 0 && abs(previouspot - pot) >= 50) || (tap == 0 && tapping == 0 && abs(previouspot - pot) >= 10)) // MOD: min pot move must be higher than 1 to debounce it a bit, 10 represents 1% of pot change
        {
            // MOD: function defined
            pwm = pot_to_pwm(pot);
            previouspot = pot;
            // MOD: delay EEPROM change due to power-off pot value changes
            if (tap == 1)
            {
                // set flag and reset ms counter for delayed EEPROM write
                eeprom_reset_tap = 1;
                TCA0.SINGLE.CNT = 0; // reseting counter and ms
                ms = 0;
            }
            tap = 0;
        }

        // MOD: handle delayed `tap` reset in EEPROM due to potentional power-off Pot value changes
        if (eeprom_reset_tap == 1)
        {
            // check if tap still 0
            if (tap == 0)
            {
                //  check delay elapsed
                if (ms > 1000)
                {
                    // write change to EEPROM
                    *(uint8_t *)(EEPROM_TAP) = 0;
                    eeprom_persist();
                    eeprom_reset_tap = 0;
                }
            }
            else
            {
                eeprom_reset_tap = 0;
            }
        }

        // MOD: added this variable to shorten main loop processing time, all tests of it later use `==` now
        currentstate = debounce();

        // TAP button handling
        if (currentstate == 0 && laststate == 0) // Tap button keeps off
        {
            if (nbtap > 1) // if too long between taps, persist values and resets tapping process
            {
                if (ms > (3 * mstempo))
                {
                    // write changed values into EEPROM
                    *(uint8_t *)(EEPROM_TAP) = 1;
                    *(uint16_t *)(EEPROM_TEMPO) = mstempo;
                    eeprom_persist();

                    // reset state machine
                    tap = 1;
                    ms = 0;
                    nbtap = 0;
                    tapping = 0;
                }
            }
            else if (nbtap == 1 && ms > 1500) // if tapped only once and timeout, reset state machine // MOD: timeout time set to constant 1.5s which is cca 2x max delay time
            {
                ms = 0;
                nbtap = 0;
                tapping = 0;
            }
        }
        else if (currentstate == 0 && laststate == 1) // Tap button just released
        {
            laststate = 0;
            PORTA.OUTCLR = (1 << LED_PIN); // switch LED off
            previouspot = pot;
        }
        else if (currentstate == 1 && laststate == 0) // Tap button just pressed
        {
            if (nbtap == 0) // first tap
            {
                TCA0.SINGLE.CNT = 0; // starts counting
                ms = 0;
                nbtap++;
                laststate = 1;
                tapping = 1;
            }

            if (nbtap != 0 && ms >= 50) // not first tap // NOTE: seems here is debouncing in `ms >` condition! // MOD: changed `ms >= 100` to `ms >= 50`
            {
                if (TCA0.SINGLE.CNT >= 500)
                {
                    ms++;
                } // round up value if timer counter more than 500us

                if (nbtap == 1) // if second tap, tempo = time elapsed between the 2 button press // MOD: changed `nbtap == 1` to `nbtap == 2`
                {
                    mstempo = ms;
                    divtempo = mstempo * divmult;
                }
                else // if more than second tap, average every tap
                {
                    mstempo = (mstempo + ms) / 2;
                    divtempo = (divtempo + (mstempo * divmult)) / 2;
                }

                // MOD: keep `divtempo` boundaries, correct `mstempo` to matches boundaries
                divtempo = divtempo_range(divtempo);
                if (divtempo == c_delay_min || divtempo == c_delay_max)
                {
                    mstempo = divtempo / divmult;
                }

                // MOD: correct calculation of the `pwm`
                pwm = divtempo_to_pwm(divtempo);

                nbtap++; // updating number of tap and last state of tap button
                laststate = 1;
                TCA0.SINGLE.CNT = 0; // reseting counter and ms
                ms = 0;
                ledms = 0;
                tap = 1;                       // now in tap control mode
                PORTA.OUTSET = (1 << LED_PIN); // swith LED on
            }
        }
        else if (currentstate == 1 && laststate == 1) // Tap button keeps on
        {
            if (ms >= 1000) // if button pressed more than 1s
            {
                // Ramp feature - gradually increase/decrease delay time across whole range while button pressed, fluctuation speed depends on `pot` value
                while (debounce())
                {
                    if (updown == 0)
                    {
                        divtempo += 1;
                    }

                    else
                    {
                        divtempo -= 1;
                    }

                    // MOD: correct calculation of the `pwm` value from `divtempo`
                    pwm = divtempo_to_pwm(divtempo);

                    // MOD: correct boundaries used
                    if (divtempo == c_delay_min || divtempo == c_delay_max)
                    {
                        updown ^= 1;
                        PORTA.OUTTGL = (1 << LED_PIN);
                    }

                    // MOD: pot interpretation corrected as it is marked as Speed, so higher value must be faster ramping
                    // MOD: ramping slowed down by multiplier constant scaling that pot value, debounce() 900us and code execution time is also added to each step time.
                    // It is then multiplied by c_delay_range (cca 700) in each direction. So we are at about 1,5s to 5s full ramping period, depending on `pot` value.
                    for (uint16_t i = 0; i < ((POT_MAX_VALUE - pot) * 2); i++)
                    {
                        _delay_us(1);
                    }
                }
                if (tap == 1)
                {
                    divtempo = mstempo * divmult;
                    // MOD: correct calculation of the `pwm`
                    pwm = divtempo_to_pwm(divtempo);
                }
                else
                {
                    // MOD: function defined
                    pwm = pot_to_pwm(pot);
                }
                updown = 0;
                previouspot = pot;
            }
        }

        // LED CONTROL
        if (tap != 1 && tapping == 0) // keep the light on when on Pot control
        {
            PORTA.OUTSET = (1 << LED_PIN);
        }
        else if (tapping == 1 && nbtap == 1 && laststate == 1) // keep the light off during long button press waiting, until handler starts to act
        {
            PORTA.OUTCLR = (1 << LED_PIN);
        }
        else if (tap == 1 && tapping == 0 && currentstate == 0) // handle blinking in Tap mode out of the tapping process
        {
            if (ledms >= (mstempo - 4)) // turns LED on every downbeat
            {
                ledms = 0;
                PORTA.OUTSET = (1 << LED_PIN);
            }

            if (ledms >= (mstempo / 2)) // MOD: turns LED off at the 50% duty cycle
            {
                PORTA.OUTCLR = (1 << LED_PIN);
            }
        }
    }
}
