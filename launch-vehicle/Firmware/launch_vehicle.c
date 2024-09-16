/*
 *  Launch Vehicle firmware
 *
 *  (c) 2022-2023 Noora Halme
 * 
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Compile for logic board revision 7 or newer with module chaining?
//#define LOGIC_BOARD_R7 1

// LED blink pattern and PIT divisor for cycling through it.
#define LED_BLINK_PATTERN  0b100000
#define LED_BLINK_DIVISOR  96

// Macro for generating a bitmask for a single bit in position 'bit'
#define BITMASK(bit) (1<<bit)

// TPIC6C595 shift register control lines.
#define SR_G_PORT PORTF
#define SR_G_BIT 2
#define SR_SRCK_PORT PORTA
#define SR_SRCK_BIT 0
#define SR_RCK_PORT PORTA
#define SR_RCK_BIT 1
#define SR_SER_IN_PORT PORTA
#define SR_SER_IN_BIT 2

// Clock input jack.
#ifdef LOGIC_BOARD_R7
#define CLOCK_PORT PORTC
#define CLOCK_BIT 3
#define CLOCK_ISR_VECT PORTC_PORT_vect
#define CLOCK_ISR_VECT_NUM PORTC_PORT_vect_num
#else
#define CLOCK_PORT PORTD
#define CLOCK_BIT 7
#define CLOCK_ISR_VECT PORTD_PORT_vect
#define CLOCK_ISR_VECT_NUM PORTD_PORT_vect_num
#endif

// Header pins for chaining multiple modules on revision 7 board.
#ifdef LOGIC_BOARD_R7
#define CHAIN_CLKIN_PORT PORTC
#define CHAIN_CLKIN_BIT 0
#define CHAIN_CLKOUT_PORT PORTC
#define CHAIN_CLKOUT_BIT 1
#endif

// ISR pin mask for clock input port.
#ifdef LOGIC_BOARD_R7
#define CLOCK_ISR_MASK (BITMASK(CLOCK_BIT) | BITMASK(CHAIN_CLKIN_BIT))
#else
#define CLOCK_ISR_MASK BITMASK(CLOCK_BIT)
#endif

// Stage switch GPIO port and pins.
#define SW_STAGE_PORT PORTD
#define SW_STAGE_BIT 2

// LED mapping on the two cascaded shift registers.
#define LED_COUNT 16
typedef enum led_addr {
  // U2
  LED_NC1,   LED_NC2,   LED_CH1_A, LED_CH1_B,
  LED_CH2_A, LED_CH2_B, LED_CH3_A, LED_CH3_B,
  // U1
  LED_STAGE, LED_SEL3,  LED_SEL2,  LED_SEL1, 
  LED_NC3,   LED_MUTE1, LED_MUTE2, LED_MUTE3
} led_addr_t;


// Mute switch GPIO ports and pins.
typedef enum mute_sw {
  SW_MUTE1=0, SW_MUTE2, SW_MUTE3
} mute_sw_t;
const __flash uint8_t mute_sw_bit[3] = { 5, 3, 0 };
PORT_t *mute_sw_port[3] = { &PORTD, &PORTD, &PORTD };
const __flash led_addr_t mute_sw_led[3] = { LED_MUTE1, LED_MUTE2, LED_MUTE3 };

// Input select switch GPIO ports and pins.
typedef enum sel_sw {
  SW_SEL1=0, SW_SEL2, SW_SEL3
} sel_sw_t;
const __flash uint8_t sel_sw_bit[3] = { 6, 4, 1 };
PORT_t *sel_sw_port[3] = { &PORTD, &PORTD, &PORTD };
const __flash led_addr_t sel_sw_led[3] = { LED_CH1_A, LED_CH2_A, LED_CH3_A };

// Mute control lines to DG4053.
const __flash uint8_t mute_ctrl_bit[3] = { 3, 4, 5 };
PORT_t *mute_ctrl_port[3] = { &PORTF, &PORTF, &PORTF };

// Input select control lines to DG4053.
const __flash uint8_t sel_ctrl_bit[3] = { 3, 4, 5 };
PORT_t *sel_ctrl_port[3] = { &PORTA, &PORTA, &PORTA };


// LED state variables.
uint8_t led_blink = LED_BLINK_PATTERN;
uint8_t led_blink_acc = 0;
volatile uint8_t led_state[LED_COUNT];


// Switch states
uint8_t mute_sw_state[3];
uint8_t sel_sw_state[3];

// Staging status
volatile uint8_t staging_armed;
volatile uint8_t stage_held;
volatile uint16_t stage_held_ticks;
volatile uint8_t staged_mute[3];
volatile uint8_t staged_sel[3];

//
// Update the 595 shift registers with LED states.
//
void update_leds(void)
{
  uint8_t i;
  volatile uint8_t led_temp_state[LED_COUNT];

  // Update temporary LED state array to create blink patterns.
  for (i=0; i<LED_COUNT; i++) led_temp_state[i] = led_state[i];
  if (staging_armed || stage_held) {
    for (i=0; i<3; i++) if (staged_mute[i]) led_temp_state[LED_MUTE1 + i] ^= led_blink & 1;
    for (i=0; i<3; i++) if (staged_sel[i]) led_temp_state[LED_SEL1 - i] ^= led_blink & 1;
    led_temp_state[LED_STAGE] = staging_armed;
  }

  // Pull RCK low while we fill the shift register.
  SR_RCK_PORT.OUTCLR = BITMASK(SR_RCK_BIT);

  // Shift all bits into the shift register.
  for (i=0; i<LED_COUNT; i++)
  {
    // Setup data in SER_IN.
    if (led_temp_state[i]) {
      SR_SER_IN_PORT.OUTSET = BITMASK(SR_SER_IN_BIT);
    } else {
      SR_SER_IN_PORT.OUTCLR = BITMASK(SR_SER_IN_BIT);
    }

    // Strobe SRCK.
    SR_SRCK_PORT.OUTSET = BITMASK(SR_SRCK_BIT);
    SR_SRCK_PORT.OUTCLR = BITMASK(SR_SRCK_BIT);
  }  

  // Strobe RCK to latch shift register bits to storage register.
  SR_RCK_PORT.OUTSET = BITMASK(SR_RCK_BIT);
  SR_RCK_PORT.OUTCLR = BITMASK(SR_RCK_BIT);
}


//
// Return the selected input for a channel.
//
uint8_t get_input(uint8_t channel)
{
  volatile PORT_t *port;
  uint8_t port_state;

  port = sel_ctrl_port[channel];
  port_state = port->OUT;
  return (port_state >> sel_ctrl_bit[channel]) & 1;
}


//
// Set the active input for a channel.
//
void set_input(uint8_t channel, uint8_t input)
{
  volatile PORT_t *port;

  // Set/clear control pin to analog switch.
  port = sel_ctrl_port[channel];
  if (input) {
    port->OUTSET = BITMASK(sel_ctrl_bit[channel]);
  } else {
    port->OUTCLR = BITMASK(sel_ctrl_bit[channel]);
  }

  // Update indicator LED state for the channel.
  led_state[sel_sw_led[channel]] = input^1;
  led_state[sel_sw_led[channel]+1] = input;
}


//
// Switch the input on a channel
//
void toggle_input(uint8_t channel)
{
  uint8_t input;

  input = get_input(channel);
  set_input(channel, input^1);
}


//
// Return the output muting state for a channel.
//
uint8_t get_mute(uint8_t channel)
{
  volatile PORT_t *port;
  uint8_t port_state;

  port = mute_ctrl_port[channel];
  port_state = port->OUT;
  return (port_state >> mute_ctrl_bit[channel]) & 1;
}


//
// Set the output muting state for a channel.
//
void set_mute(uint8_t channel, uint8_t state)
{
  volatile PORT_t *port;

  // Set/clear control pin to analog switch.
  port = mute_ctrl_port[channel];
  if (state) {
    port->OUTSET = BITMASK(mute_ctrl_bit[channel]);
  } else {
    port->OUTCLR = BITMASK(mute_ctrl_bit[channel]);
  }

  // Update mute button indicator LED state for the channel.
  led_state[mute_sw_led[channel]] = state;
}


//
// Toggle the output muting state on a channel.
//
void toggle_mute(uint8_t channel)
{
  uint8_t state;

  state = get_mute(channel);
  set_mute(channel, state^1);
}


//
// Perform staged changes
//
void perform_staging(void)
{
  uint8_t i;

  // Perform the changes 
  for (i=0; i<3; i++) {
    if (staged_mute[i]) toggle_mute(i);
    if (staged_sel[i]) toggle_input(i);
  }

  // Clear the staged changes
  for(i=0; i<3; i++) {
    staged_mute[i]=0;
    staged_sel[i]=0;
  }
  staging_armed=0;
}


//
// Interrupt handler for programmable interrupt timer.
//
ISR(RTC_PIT_vect) {  
  volatile int i, state, changes;
  volatile PORT_t *port;

  // Check first if stage button is held for staging.
  port = &SW_STAGE_PORT;
  state = ~(port->IN) & BITMASK(SW_STAGE_BIT);
  if (state) {
    if (!stage_held)
    {
      // Stage button went from up to being held down.
      stage_held=1;
      stage_held_ticks=0;
    } else {
      // If the tick counter passes 1000, clear the staged changes and unarm.
      if (stage_held_ticks >= 1000 && staging_armed) {
         staging_armed = 0;
         for(i=0; i<3; i++) {
           staged_mute[i] = 0;
          staged_sel[i] = 0;
        }
      }
      stage_held_ticks++;
    }
  } else {
    // Stage button is up - was it previously held down?
    if (stage_held) {
      if (staging_armed) {
        // Changes were armed, commit them.
        perform_staging();
      } else {
        // If we have changes waiting, arm staging.
        for (i=0,changes=0; i<3; i++) {
          changes |= staged_mute[i];
          changes |= staged_sel[i];
        }
        if (changes) staging_armed=1;
      }
    }
    stage_held = 0;
  }

  // Read select and mute switch states
  for (i=0; i<3; i++) {
    if (stage_held) {
      // Stage held - any changes will be staged.
      port = mute_sw_port[i];
      state = (~(port->IN) & BITMASK(mute_sw_bit[i]));
      if (!mute_sw_state[i] && state) staged_mute[i]^=1;
      mute_sw_state[i] = state;

      port = sel_sw_port[i];
      state = (~(port->IN) & BITMASK(sel_sw_bit[i]));
      if (!sel_sw_state[i] && state) staged_sel[i]^=1;
      sel_sw_state[i] = state;      
    } else {
      // Not staging - apply changes immediately.
      port = mute_sw_port[i];
      state = (~(port->IN) & BITMASK(mute_sw_bit[i]));
      if (!mute_sw_state[i] && state) toggle_mute(i);
      mute_sw_state[i] = state;
      
      port = sel_sw_port[i];
      state = (~(port->IN) & BITMASK(sel_sw_bit[i]));
      if (!sel_sw_state[i] && state) toggle_input(i);
      sel_sw_state[i] = state;
    }
  }

  // Adjust LED states and update shift registers.
  update_leds();

  // Rotate 6-bit LED blink pattern if accumulator overflows.
  led_blink_acc++;
  if (led_blink_acc >= LED_BLINK_DIVISOR) {
    led_blink <<= 1;
    led_blink = (led_blink & 0x3f) | (led_blink>>6);
    led_blink_acc = 0;
  }

  // Clear port interrupt flag.
  RTC.PITINTFLAGS = 1;
}


//
// Interrupt handler for clock input sensing.
//
ISR(CLOCK_ISR_VECT) {
  volatile PORT_t *port;

  // Was the interrupt fired from clock pulse?
  port = &CLOCK_PORT;
  if (port->INTFLAGS & CLOCK_ISR_MASK) {
    if (staging_armed && !stage_held) {
      // Staging armed and button is not held, perform staging.
      perform_staging();
    }

    // Clear interrupt flags from clock input pins.
    port->INTFLAGS |= CLOCK_ISR_MASK;
  }
}


//
// Main program
//
int main(void) {
  uint8_t i;
  volatile PORT_t *port;

  // Set direction bits and pullups for tactile switch input pins.
  for (i=0; i<3; i++) {
    port = mute_sw_port[i];
    port->DIRCLR = BITMASK(mute_sw_bit[i]);
    (&port->PIN0CTRL)[mute_sw_bit[i]] |= PORT_PULLUPEN_bm;
    mute_sw_state[i] = 0;

    port = sel_sw_port[i];
    port->DIRCLR = BITMASK(sel_sw_bit[i]);
    (&port->PIN0CTRL)[sel_sw_bit[i]] |= PORT_PULLUPEN_bm;
    sel_sw_state[i] = 0;
  }

  // Set direction bit and pullup for stage switch.
  port = &SW_STAGE_PORT;
  SW_STAGE_PORT.DIRCLR = BITMASK(SW_STAGE_BIT);
  (&port->PIN0CTRL)[SW_STAGE_BIT] |= PORT_PULLUPEN_bm;

  // Set directon bits for analog switch select lines.
   for (i=0; i<3; i++) {
    port = mute_ctrl_port[i];
    port->DIRSET = BITMASK(mute_ctrl_bit[i]);

    port = sel_ctrl_port[i];
    port->DIRSET = BITMASK(sel_ctrl_bit[i]);
   } 

  // Set direction bits for shift register control lines.
  SR_G_PORT.DIRSET = BITMASK(SR_G_BIT);
  SR_SRCK_PORT.DIRSET = BITMASK(SR_SRCK_BIT);
  SR_RCK_PORT.DIRSET = BITMASK(SR_RCK_BIT);
  SR_SER_IN_PORT.DIRSET = BITMASK(SR_SER_IN_BIT);

  // Set direction bit and interrupt sensing for clock input.
  port = &CLOCK_PORT;
  CLOCK_PORT.DIRCLR = BITMASK(CLOCK_BIT);
  (&port->PIN0CTRL)[CLOCK_BIT] |= PORT_ISC_FALLING_gc;

  // Set direction bits and sensing for clock chaining pins.
#ifdef LOGIC_BOARD_R7
  port = &CHAIN_CLKIN_PORT;
  CHAIN_CLKIN_PORT.DIRCLR = BITMASK(CHAIN_CLKIN_BIT);
  (&port->PIN0CTRL)[CHAIN_CLKIN_BIT] |= PORT_ISC_FALLING_gc;
  CHAIN_CLKOUT_PORT.DIRCLR = BITMASK(CHAIN_CLKOUT_BIT);
#endif

  // Clear staging status.
  staging_armed = 0;
  stage_held = 0;
  for(i=0; i<3; i++) {
    staged_mute[i] = 0;
    staged_sel[i] = 0;
  }

  // Delay for 100ms to allow I/O pins with pullups to settle high.
  _delay_ms(100);

  // Initialize module to default settings.
  for (i=0; i<3; i++) {
    set_input(i, 0);
    set_mute(i, 0);
  }
  update_leds();

  // Enable output buffers in shift registers.
  SR_G_PORT.OUTCLR = BITMASK(SR_G_BIT);

  // Enable PIT IRQ.
  RTC.PITINTCTRL = 1;
  while( RTC.PITSTATUS & 0x01 );

  // Use internal 32kHz RTC clock and enable PIT with 32 cycle period. This will
  // give us a timer interrupt every 1ms.
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
  RTC.PITCTRLA = RTC_PERIOD_CYC32_gc | RTC_PITEN_bm; 

  // Enable sleep mode and set SMODE to PDOWN.
  SLPCTRL.CTRLA = 0x05;

  // Set level 1 interrupt vector to PORTD.
  CPUINT.LVL1VEC = CLOCK_ISR_VECT_NUM;

  // Enable interrupts.
  sei();
  
  // Enter sleep mode.
  while(1) {
    asm("sleep");
  }
}