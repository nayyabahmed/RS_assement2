// These #defines act like a find-and-replace
// in your code, and make your code more readable.
// Note that there is no #define for E0_B:.
// it's a non-standard pin, check out setupEncoder0().
#define E1_A_PIN  7
#define E1_B_PIN  23
#define E0_A_PIN  26


#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define SQR_t 3270  // The time to move 50 cm at 40,40 power
#define CPR 1440     // Counts per revolution of the wheel
#define WC 21.98     // Wheel circumference in cm.
#define CF50 3276   // Counts for 50cm

#define TURN_PWR 40
#define PWR 40
float l_power;
float r_power;

float s;
float x;
bool t = true;

unsigned long time_now;
unsigned long time_elapsed;
unsigned long last_timestamp;

// Volatile Global variables used by Encoder ISR.
volatile long count_er; // used by encoder to count the rotation
volatile bool oldE1_A;  // used by encoder to remember prior state of A
volatile bool oldE1_B;  // used by encoder to remember prior state of B

volatile long count_el; // used by encoder to count the rotation
volatile bool oldE0_A;  // used by encoder to remember prior state of A
volatile bool oldE0_B;  // used by encoder to remember prior state of B



// put your setup code here, to run once:
void setup() {

  // These two function set up the pin
  // change interrupts for the encoders.
  // If you want to know more, find them
  // at the end of this file.
  setupEncoder0();
  setupEncoder1();
  // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial l_power and r_power values.
  l_power = 0;
  r_power = 0;

  // Initialise the Serial communication
  // so that we can inspect the values of
  // our encoder using the Monitor.
  Serial.begin( 9600 );

  // Wait for serial connection to establish
  delay(1000);

  // Flag up reset to Serial monitor
  Serial.println("*** RESET ***");

  last_timestamp = millis();
}



// put your main code here, to run repeatedly:
void loop() {

  time_now = millis();
  time_elapsed = time_now - last_timestamp;
  //go_forward(SQR_t);
  //count_move();
  square_e_move();
  



  // Send power PWM to pins, to motor drivers.
  analogWrite( L_PWM_PIN, l_power );
  analogWrite( R_PWM_PIN, r_power );
  print_count();  //prints the counts for both wheels

}

void square_e_move(){
  
  if ( count_er < CF50 )
  {
    powerInput(30,31);
  }
  else if(count_er< ( CF50+CPR) ) {
    powerInput(0,31);
  }
  else if( count_er< ( 2*CF50+CPR ) ) {
    powerInput(30,31);
  }
  else if( count_er< ( 2*(CF50+CPR) ) ) {
    powerInput(0,31);
  }
   else if(count_er< ( 3*CF50+CPR*2 ) ) {
    powerInput(30,31);
  }
   else if(count_er< ( 3*(CF50+CPR) ) ) {
    powerInput(0,31);
  }
   else if( count_er< ( 4*CF50+3*CPR ) ) {
    powerInput(30,31);
  }
   else if( count_er< ( 4*(CF50+CPR) ) ){
    powerInput(0,31);
  }
  else{
    powerInput(0,0);    
  }
  
}

void count_move() {
  bool c1 = (count_el < 2000 || count_er < 2000 );
  bool c2 = (count_el < 4000 || count_er < 4000);
  bool c3 = (count_el > 2000 || count_er > 2000);
  bool c4 = (count_el > 0 || count_er > 0);

  if (t)
  {
    if (c1 )
    {
      s = count_el / 100;
      x = 20 + s;
      powerInput(x, x);
    }
    else if (c2)
    {
      s = count_el / 500;
      powerInput(x - s, x - s);
    }
    else {
      t = false;
      s=0;
    }
  }

  else
  {

    if (c3 )
    {
      s = s+0.1;
      x = -20-s;
      powerInput(x, x);
    }
    else if (c4)
    {
      x=x+0.1;
      powerInput(x, x);
    }
    else {
      powerInput(0, 0);
    }

  }

}


void print_count() {
  // Output the count values for our encoders
  // with a comma seperation, which allows for
  // two lines to be drawn on the Plotter.
  //
  // NOTE: count_eland count_er values are now
  //       automatically updated by the ISR when
  //       the encoder pins change.
  //
  Serial.print( count_el);
  Serial.print( ", ");
  Serial.println( count_er );

  // short delay so that our plotter graph keeps
  // some history.
  delay( 2 );
}


void powerInput(float l, float r)
{
  bool c1 = ( (l >= 0) && (r >= 0) );
  bool c2 = ( (l < 0) && (r < 0) );
  bool c3 = ( (l < 0) && (r >= 0) );
  bool c4 = ( (l >= 0) && (r < 0) );

  if (c1)
  {
    l_power = l;
    r_power = r;
    digitalWrite( L_DIR_PIN, LOW  );
    digitalWrite( R_DIR_PIN, LOW );
  }

  if (c2)
  {
    l_power = l * -1;
    r_power = r * -1;
    digitalWrite( L_DIR_PIN, HIGH  );
    digitalWrite( R_DIR_PIN, HIGH );
  }

  if (c3)
  {
    l_power = l * -1;
    r_power = r ;
    digitalWrite( L_DIR_PIN, HIGH  );
    digitalWrite( R_DIR_PIN, LOW );
  }

  if (c4)
  {
    l_power = l;
    r_power = r * -1;
    digitalWrite( L_DIR_PIN, LOW );
    digitalWrite( R_DIR_PIN, HIGH );
  }

}

void go_forward(int t)
{

  if (time_elapsed < t)
  {
    powerInput(41, 40);
  }

  /*else if (time_elapsed < (t + 11000))
    {
    powerInput(-30, -32);

    }*/
  else
  {
    powerInput(0, 0);
  }

}

// This ISR handles just Encoder 1
// ISR to read the Encoder1 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR( INT6_vect ) {
  // First, Read in the new state of the encoder pins.
  // Standard pins, so standard read functions.
  boolean newE1_B = digitalRead( E1_B_PIN );
  boolean newE1_A = digitalRead( E1_A_PIN );

  // Some clever electronics combines the
  // signals and this XOR restores the
  // true value.
  newE1_A ^= newE1_B;

  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:
  //
  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;
  state = state | ( newE1_A  << 3 );
  state = state | ( newE1_B  << 2 );
  state = state | ( oldE1_A  << 1 );
  state = state | ( oldE1_B  << 0 );


  // This is an inefficient way of determining
  // the direction.  However it illustrates well
  // against the lecture slides.
  if ( state == 1 ) {
    count_er = count_er - 1;  // cw?
  } else if ( state == 2 ) {
    count_er = count_er + 1;  // ccw
  } else if ( state == 4 ) {
    count_er = count_er + 1;  // ccw
  } else if ( state == 7  ) {
    count_er = count_er - 1;  // cw
  } else if ( state == 8  ) {
    count_er = count_er - 1;  // cw
  } else if ( state == 11  ) {
    count_er = count_er + 1;  // ccw
  } else if ( state == 13  ) {
    count_er = count_er + 1;  // ccw
  } else if ( state == 14  ) {
    count_er = count_er - 1;  // cw
  }

  // Save current state as old state for next call.
  oldE1_A = newE1_A;
  oldE1_B = newE1_B;

}


// This ISR handles just Encoder 0
// ISR to read the Encoder0 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR( PCINT0_vect ) {

  // First, Read in the new state of the encoder pins.

  // Mask for a specific pin from the port.
  // Non-standard pin, so we access the register
  // directly.
  // Reading just PINE would give us a number
  // composed of all 8 bits.  We want only bit 2.
  // B00000100 masks out all but bit 2
  boolean newE0_B = PINE & (1 << PINE2);
  //boolean newE0_B = PINE & B00000100;  // Does same as above.

  // Standard read fro the other pin.
  boolean newE0_A = digitalRead( E0_A_PIN ); // 26 the same as A8

  // Some clever electronics combines the
  // signals and this XOR restores the
  // true value.
  newE0_A ^= newE0_B;



  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:
  //
  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;
  state = state | ( newE0_A  << 3 );
  state = state | ( newE0_B  << 2 );
  state = state | ( oldE0_A  << 1 );
  state = state | ( oldE0_B  << 0 );

  // This is an inefficient way of determining
  // the direction.  However it illustrates well
  // against the lecture slides.
  if ( state == 1 ) {
    count_el = count_el - 1; // cw?
  } else if ( state == 2 ) {
    count_el = count_el + 1; // ccw
  } else if ( state == 4 ) {
    count_el = count_el + 1; // ccw
  } else if ( state == 7  ) {
    count_el = count_el - 1; // cw
  } else if ( state == 8  ) {
    count_el = count_el - 1; // cw
  } else if ( state == 11  ) {
    count_el = count_el + 1; // ccw
  } else if ( state == 13  ) {
    count_el = count_el + 1; // ccw
  } else if ( state == 14  ) {
    count_el = count_el - 1; // cw
  }

  // Save current state as old state for next call.
  oldE0_A = newE0_A;
  oldE0_B = newE0_B;
}





/*
   This setup routine enables interrupts for
   encoder1.  The interrupt is automatically
   triggered when one of the encoder pin changes.
   This is really convenient!  It means we don't
   have to check the encoder manually.
*/
void setupEncoder1() {

  // Initialise our count value to 0.
  count_er = 0;

  // Initialise the prior A & B signals
  // to zero, we don't know what they were.
  oldE1_A = 0;
  oldE1_B = 0;

  // Setup pins for encoder 1
  pinMode( E1_A_PIN, INPUT );
  pinMode( E1_B_PIN, INPUT );

  // Now to set up PE6 as an external interupt (INT6), which means it can
  // have its own dedicated ISR vector INT6_vector

  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit low, preserve other bits
  EIMSK = EIMSK & ~(1 << INT6);
  //EIMSK = EIMSK & B1011111; // Same as above.

  // Page 89, 11.1.2 External Interrupt Control Register B – EICRB
  // Used to set up INT6 interrupt
  EICRB |= ( 1 << ISC60 );  // using header file names, push 1 to bit ISC60
  //EICRB |= B00010000; // does same as above

  // Page 90, 11.1.4 External Interrupt Flag Register – EIFR
  // Setting a 1 in bit 6 (INTF6) clears the interrupt flag.
  EIFR |= ( 1 << INTF6 );
  //EIFR |= B01000000;  // same as above

  // Now that we have set INT6 interrupt up, we can enable
  // the interrupt to happen
  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit high, preserve other bits
  EIMSK |= ( 1 << INT6 );
  //EIMSK |= B01000000; // Same as above

}

void setupEncoder0() {

  // Initialise our count value to 0.
  count_el = 0;

  // Initialise the prior A & B signals
  // to zero, we don't know what they were.
  oldE0_A = 0;
  oldE0_B = 0;

  // Setting up E0_PIN_B:
  // The Romi board uses the pin PE2 (port E, pin 2) which is
  // very unconventional.  It doesn't have a standard
  // arduino alias (like d6, or a5, for example).
  // We set it up here with direct register access
  // Writing a 0 to a DDR sets as input
  // DDRE = Data Direction Register (Port)E
  // We want pin PE2, which means bit 2 (counting from 0)
  // PE Register bits [ 7  6  5  4  3  2  1  0 ]
  // Binary mask      [ 1  1  1  1  1  0  1  1 ]
  //
  // By performing an & here, the 0 sets low, all 1's preserve
  // any previous state.
  DDRE = DDRE & ~(1 << DDE6);
  //DDRE = DDRE & B11111011; // Same as above.

  // We need to enable the pull up resistor for the pin
  // To do this, once a pin is set to input (as above)
  // You write a 1 to the bit in the output register
  PORTE = PORTE | (1 << PORTE2 );
  //PORTE = PORTE | 0B00000100;

  // Encoder0 uses conventional pin 26
  pinMode( E0_A_PIN, INPUT );
  digitalWrite( E0_A_PIN, HIGH ); // Encoder 0 xor

  // Enable pin-change interrupt on A8 (PB4) for encoder0, and disable other
  // pin-change interrupts.
  // Note, this register will normally create an interrupt a change to any pins
  // on the port, but we use PCMSK0 to set it only for PCINT4 which is A8 (PB4)
  // When we set these registers, the compiler will now look for a routine called
  // ISR( PCINT0_vect ) when it detects a change on the pin.  PCINT0 seems like a
  // mismatch to PCINT4, however there is only the one vector servicing a change
  // to all PCINT0->7 pins.
  // See Manual 11.1.5 Pin Change Interrupt Control Register - PCICR

  // Page 91, 11.1.5, Pin Change Interrupt Control Register
  // Disable interrupt first
  PCICR = PCICR & ~( 1 << PCIE0 );
  // PCICR &= B11111110;  // Same as above

  // 11.1.7 Pin Change Mask Register 0 – PCMSK0
  PCMSK0 |= (1 << PCINT4);

  // Page 91, 11.1.6 Pin Change Interrupt Flag Register – PCIFR
  PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.

  // Enable
  PCICR |= (1 << PCIE0);
}
