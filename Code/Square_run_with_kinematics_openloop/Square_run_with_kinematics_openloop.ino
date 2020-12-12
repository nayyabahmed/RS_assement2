#include "pid.h"
#include "kinematics.h"
#include "LineSensor.h"
#include "encoders.h"
#include "motor.h"

// These #defines act like a find-and-replace
// in your code, and make your code more readable.
// Note that there is no #define for E0_B:.
// it's a non-standard pin, check out setupEncoder0().
#define E1_A_PIN  7
#define E1_B_PIN  23
#define E0_A_PIN  26


#define L_PWM_PIN 10 // Motor Pins.
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define SQR_t 3270  // The time to move 50 cm at 40,40 power
#define CPR 1440     // Counts per revolution of the wheel
#define WC 21.98     // Wheel circumference in cm.
#define CF10 3276   // Counts for 10cm


float l_power;
float r_power;






float count_el_change;
float count_el_old;
float speed_el;


unsigned long spd_update_ts;
unsigned long pid_update_ts;
unsigned long dir_switch_ts;
unsigned long pose_print_ts;

long          last_el_count;
long          last_er_count;
unsigned long pwr_ramp_ts;

float el_spd;
float er_spd;

float pwr;
float pwr_inc;
int dir;
float spd_avg;
float demand;
float dl;
float dr;
float last_error;  // error for drivative term
float integral_error; // for integral term
float pwr_l;
float pwr_r;

// Experiment with your gains slowly, one by one.
float Kp_left = 50; //Proportional gain
float Kd_left = 0; //Derivative gain
float Ki_left = 0.165; //Integral gain


float Kp_right = 50; //Proportional gain
float Kd_right = 0; //Derivative gain
float Ki_right = 0.165; //Integral gain

// Clas instances.
Motor_c       L_Motor( L_PWM_PIN, L_DIR_PIN);                       // To set left motor power.
Motor_c       R_Motor( R_PWM_PIN, R_DIR_PIN);
PID_c         left_PID(Kp_left, Ki_left, Kd_left);
PID_c         right_PID(Kp_right, Ki_right, Kd_right);
Kinematics_c  RomiPose;

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

  // Make sure motors are off so the robot
  // stays still.
  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );

  // Initialise the Serial communication
  // so that we can inspect the values of
  // our encoder using the Monitor.
  Serial.begin( 9600 );

  // Wait for serial connection to establish
  delay(1500);

  // Flag up reset to Serial monitor
  Serial.println("*** RESET ***");

  //last_timestamp = micros();
  spd_update_ts = millis();
  pid_update_ts = millis();
  dir_switch_ts = millis();

  pwr_ramp_ts = millis();

  pose_print_ts = millis();
  el_spd =      0;
  er_spd =      0;
  pwr =         0;
  pwr_inc =    10;
  dir =         1;
  spd_avg =     0;
  demand =    0.8;
  dl =        0.8;
  dr =        0.8;
  last_error =  0;

  integral_error = 0;
  last_el_count =  0;
  last_er_count =  0;
  pwr_l =          0;
  pwr_r =          0;

  left_PID.reset();
  right_PID.reset();
  // We set the robot to start kinematics
  // X = 0, Y = 0, Theta = 0
  RomiPose.setPose( 0, 0, 0.02 );

}



// put your main code here, to run repeatedly:
void loop() {
  // Always update kinematics
  RomiPose.update( count_el, count_er );

  printPose();
  speed_update();
  square_run();


  // output_signal <----PID-- demand, measurement
  unsigned long pid_update_dt = millis() - pid_update_ts;
  if ( pid_update_dt > 20) {
    pwr_l = left_PID.update(dl, el_spd);
    pwr_r = right_PID.update(dr, er_spd);
  }
  
  L_Motor.setPower(pwr_l);
  R_Motor.setPower(pwr_r);

 


  delay(1);



  //print_count();  //prints the counts for both wheels

} // End of main loop

void square_run(){
   //X,Y and theta are coordinates from global frame
  //The first "if" is checking if we reached 500 mm in X,
  //the rest two conditions are for interlocking that if once we,
  //reach 500 mm in X then this "if" does not run again
  if ( RomiPose.x < 500  && RomiPose.y < 50 && RomiPose.theta < 1  ) {
    //pwr_l,pwr_r are left and right motor PID output
    dl = 0.8;

  }
  //Rotating 90 deg or 1.57 rad, at (X , Y)= (500 , 0) mm.
  //The theta value of 1.3 is to control overshoot
  //as actually romi will rotate more, to around 1.6 radian
  else if (RomiPose.theta < 1.3 ) {
    dl = -0.8; //reversing demand for left wheel to trun on spot
  }
  // Checking if we reach Y=500mm. The other check in theta
  //is for interlocking so that this if does not run once we reach Y=500 mm
  else if (RomiPose.y < 500 & RomiPose.theta < 3.2  ) {
    dl = 0.8;
  }
  //Rotating additional 90 deg or 1.57 rad, at (X , Y)= (500 , 500) mm.
  //The theta value of 3 rad is to control overshoot in rotation
  else if (RomiPose.theta < 3 ) {
    dl = -0.8; //reversing demand for left wheel to trun on spot
  }
  //Now we are going back in X direction. We are stopping at x>30
  //to compansate for overshoot. Rest of two conditions are for interlocking
  //so that this "else if" does not run once we reach x=30
  else if (RomiPose.x > 30 && RomiPose.theta > 3 && RomiPose.y > 300 ) {
    dl = 0.8;

  }
  //Rotating additional 90 deg or 1.57 rad, at (X , Y)= (0 , 500) mm.
  //The theta value of 4.5 rad is to control overshoot in rotation
  else if (RomiPose.theta < 4.5 ) {
    dl = -0.8; //reversing demand for left wheel to trun on spot

  }
  //Now we are going back in Y direction. We are stopping at y>30
  //to compansate for undershoot. Rest of  condition is for interlocking
  //so that else if does not run once we reach y = -10
  else if (RomiPose.y > -10 && RomiPose.theta > 4 ) {
    dl = 0.8;

  }

  //Rotating additional 90 deg or 1.57 rad, at (X , Y)= (0 , 0) mm.
  //We get full 2*pi or 6.2 radian rotation here!
  else if ( RomiPose.theta < 6.2 ) {
    dl = -0.8; //reversing demand for left wheel to trun on spot

  }
  // Stopping
  else {
    dl = 0;
    dr = 0;
  }


}

void printPose() {
  unsigned long pose_print_dt = millis() - pose_print_ts;

  if (pose_print_dt > 100) {
    pose_print_ts = millis();
    Serial.print(RomiPose.x);
    Serial.print(",");
    Serial.print(RomiPose.y);
    Serial.print( "," );
    Serial.println( RomiPose.theta );
  }
}

void speed_update() {
  unsigned long spd_update_dt = millis() - spd_update_ts;

  if (spd_update_dt > 20) {
    spd_update_ts = millis();

    long el_diff = count_el - last_el_count;
    last_el_count = count_el;

    el_spd = (float)el_diff / (float)spd_update_dt;

    long er_diff = count_er - last_er_count;
    last_er_count = count_er;

    er_spd = (float)er_diff / (float)spd_update_dt;
    //spd_avg = (spd_avg* .8) + (eo_spd * .2) ;
    //Serial.print(spd_avg);
    //Serial.print(",");
    //Serial.print( el_spd );
    //Serial.print( "," );
    //Serial.print( er_spd );
    //Serial.print( "," );
    //Serial.println( Faward_Bias );

  }

 }


void print_count() {
  Serial.print( count_el);
  Serial.print( ", ");
  Serial.println( count_er );

  delay( 2 );
}
