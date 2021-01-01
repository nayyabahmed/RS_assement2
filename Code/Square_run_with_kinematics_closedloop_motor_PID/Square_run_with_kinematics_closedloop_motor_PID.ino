#include "pid.h"
#include "kinematics.h"
#include "LineSensor.h"
#include "encoders.h"
#include "motor.h"


#define E1_A_PIN  7   // Encoder Pins
#define E1_B_PIN  23
#define E0_A_PIN  26

#define LINE_LEFT_PIN 22   // Line Sensor left pin
#define LINE_CENTRE_PIN 21 // Line Sensor centre pin
#define LINE_RIGHT_PIN 20   // Line Sensor right pin

#define L_PWM_PIN 10 // Motor Pins.
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

// Flags used for a basic finite state machine
#define INITIAL_STATE     0
#define FOLLOW_SQUARE     1
#define RESTTING          2     

int STATE = INITIAL_STATE;  // System starts by driving straight

unsigned long spd_update_ts;
unsigned long pid_update_ts;
unsigned long pose_print_ts;
unsigned long initial_state_ts;

// variables for speed estimate
long          last_el_count;
long          last_er_count;
float el_spd;
float er_spd;

// variables for speed demand
float dl;
float dr;

// variables for power to motors
float pwr_l;
float pwr_r;

// Gains for motor PID
float Kp_left = 50; //Proportional gain
float Kd_left = 0; //Derivative gain
float Ki_left = 0.165; //Integral gain
float Kp_right = 50; //Proportional gain
float Kd_right = 0; //Derivative gain
float Ki_right = 0.165; //Integral gain

// Clas instances.
Motor_c       L_Motor( L_PWM_PIN, L_DIR_PIN);                       // To set left motor power.
Motor_c       R_Motor( R_PWM_PIN, R_DIR_PIN);
LineSensor_c  line_left(LINE_LEFT_PIN);
LineSensor_c  line_centre(LINE_CENTRE_PIN);
LineSensor_c  line_right(LINE_RIGHT_PIN);
PID_c         left_PID(Kp_left, Ki_left, Kd_left);
PID_c         right_PID(Kp_right, Ki_right, Kd_right);
Kinematics_c  RomiPose;

// put your setup code here, to run once:
void setup() {

  // These two function set up the pin
  // change interrupts for the encoders.
  setupEncoder0();
  setupEncoder1();

  // Make sure motors are off so the robot
  // stays still.
  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );

  // Initialise the Serial communication
  Serial.begin( 9600 );
  delay(1500);
  // Flag up reset to Serial monitor
  Serial.println("*** RESET ***");
  
   //calibrating line sensors
  line_left.calibrate();  
  line_centre.calibrate();
  line_right.calibrate();

  spd_update_ts = millis(); //Initialize time stamps
  pid_update_ts = millis();
  pose_print_ts = millis();
  initial_state_ts = millis();
  
  el_spd =         0; 
  er_spd =         0;
  last_el_count =  0;
  last_er_count =  0;
  
  dl =        0.8;
  dr =        0.8;
  pwr_l =     0;
  pwr_r =     0;

  // We set the robot to start kinematics
  // X = 0, Y = 0, Theta = 0.05
  RomiPose.setPose( 0, 0, 0.05 );

 }

// put your main code here, to run repeatedly:
void loop() {
  // Always update kinematics
  RomiPose.update( count_el, count_er );
  printPose();  //print pose parameters
  
  // We run the appropriate function for the current
  // state of our robot.
  if ( STATE == INITIAL_STATE  ) {
    intialisingBeeps();
  }
 else if ( STATE == FOLLOW_SQUARE  ) {
  if (RomiPose.theta<6.2){
  speed_update(); //get update on speed
  pidUpdate();
  square_run(500,500); //provide lenght of square in mm  
  }
  else{
   STATE = 2;}
 }    
 else if ( STATE == RESTTING){
  L_Motor.setPower(0);
  R_Motor.setPower(0);
  Serial.println("Restting...........");
  delay(500);  
  }
  
  //print_count();  //prints the counts for both wheels
  delay(1);

} // End of main loop

void intialisingBeeps() {
// fuction for state 0
  unsigned long initial_state_dt = millis() - initial_state_ts;
  if ( initial_state_dt < 5000 ) {

    analogWrite(6, 10);
    delay(100);
    analogWrite(6, 0);
    delay(900);
    
  } else {  
    
  left_PID.reset();
  right_PID.reset();
  
  STATE =   1;
  }
}

void pidUpdate(){
// output_signal <----PID-- demand, measurement  
  unsigned long pid_update_dt = millis() - pid_update_ts;
  
  if ( pid_update_dt > 10) {
    pid_update_ts= millis();
    
    pwr_l = left_PID.update(dl, el_spd);
    pwr_r = right_PID.update(dr, er_spd);
    
  L_Motor.setPower(pwr_l);
  R_Motor.setPower(pwr_r);
  
  }
 }

void square_run(int X, int Y){
   //X,Y and theta are coordinates from global frame
  //The first "if" is checking if we reached 500 mm in X,
  //the rest two conditions are for interlocking that if once we,
  //reach 500 mm in X then this "if" does not run again
  if ( RomiPose.x < X  && RomiPose.y < 50 && RomiPose.theta < 1  ) {
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
  
  else if (RomiPose.y < Y & RomiPose.theta < 3.2  ) {
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
  else if (RomiPose.x > 20 && RomiPose.theta > 3 && RomiPose.y > 300 ) {
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
