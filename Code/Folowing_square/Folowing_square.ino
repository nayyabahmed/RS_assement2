// Pin definitions, to make the
// code easier to read.
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define TURN_PWR 40
#define PWR 40
#define SQR_t 3270

bool t = true;

unsigned long time_now;
unsigned long time_elapsed;
unsigned long last_timestamp;


// Variables to remember our
// motor power for Left and Right.
// Byte stores 0 to 255
float l_power;
float r_power;

// Setup, only runs once when powered on.
void setup() {

  // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial l_power and r_power values.
  l_power = 0;
  r_power = 0;


  // Start up the Serial for debugging.
  Serial.begin(9600);
  delay(1000);
  // Print reset so we can catch any reset error.
  Serial.println(" ***Reset*** ");

  last_timestamp = millis();

}








void move_in_square(int t)
{
  int r = 1500;

  if (time_elapsed < t)
  {
    powerInput(PWR, PWR);
  }
  else if (time_elapsed < (t + r) )
  {
    powerInput(0, TURN_PWR);
  }

  else if (time_elapsed < (t * 2 + r))
  {
    powerInput(PWR, PWR);
  }

  else if (time_elapsed < (t * 2 + r * 2))
  {
    powerInput(0, TURN_PWR);
  }
  else if (time_elapsed < (t * 3 + r * 2))
  {
    powerInput(PWR, PWR + 2);
  }
  else if (time_elapsed < (t * 3 + r * 3))
  {
    powerInput(0, TURN_PWR);
  }
  else if (time_elapsed < (t * 4 + r * 3))
  {
    powerInput(PWR, PWR);
  }
  else if (time_elapsed < (t * 4 + r * 4))
  {
    powerInput(0, TURN_PWR);
  }
  else
  {
    powerInput(0, 0);
  }
}



// put your main code here, to run repeatedly:
void loop() {

  time_now = millis();
  time_elapsed = time_now - last_timestamp;


  // Adjust power. e.g., increment by 4 on every loop()
  //speedChange();

  go_forward(SQR_t); //Start and move for a period and then comes back to statrting point
  //move_in_square(SQR_t); // function moves the robot in a square an then stops

  // Send power PWM to pins, to motor drivers.
  analogWrite( L_PWM_PIN, l_power );
  analogWrite( R_PWM_PIN, r_power );

  print_debug();// function for serail printing of states

}

void print_debug()
{
  Serial.print("The power to left motor: ");
  Serial.print(l_power);
  Serial.print("\n");

  Serial.print("The power to right motor: ");
  Serial.print(r_power);
  Serial.print("\n");

  delay(250); // Brief pause
}

void go_forward(int t) 
{

  if (time_elapsed < 700)
  {
    powerInput(-40, 40);
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

void speedChange()
{
  if (t)
  {
    l_power = l_power + 4;
    r_power = r_power + 4;
    if (l_power > 200)
    { t = false;
    }

  }
  if (!t)
  {

    l_power = l_power - 4;
    r_power = r_power - 4;
    if (l_power < 10)
    {
      t = true;
    }
  }
}
