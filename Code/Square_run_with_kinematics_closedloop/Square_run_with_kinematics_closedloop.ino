#include "pid.h"
#include "kinematics.h"
#include "LineSensor.h"
#include "encoders.h"
#include "motor.h"


#define E1_A_PIN       7   // Encoder Pins
#define E1_B_PIN      23
#define E0_A_PIN      26

#define LINE_LEFT_PIN   22   // Line Sensor left pin
#define LINE_CENTRE_PIN 21 // Line Sensor centre pin
#define LINE_RIGHT_PIN  20   // Line Sensor right pin

#define L_PWM_PIN   10 // Motor Pins.
#define L_DIR_PIN   16
#define R_PWM_PIN    9
#define R_DIR_PIN   15

// Flags used for a basic finite state machine
#define INITIAL_STATE     0
#define FOLLOW_SQUARE     1
#define RESTING           2     

int STATE = INITIAL_STATE;  // System starts by driving straight

unsigned long spd_update_ts;
unsigned long pid_update_ts;
unsigned long pose_print_ts;
unsigned long initial_state_ts;
unsigned long T_update_ts; 

// variables for speed estimate
long          last_el_count;
long          last_er_count;
float el_spd;
float er_spd;

// variables for speed demand
float dl_forward;
float dr_forward;
float d_turn;
float T; //Theta required value


// variables for power to motors
float pwr_l;
float pwr_r;

int state_sqr=0;
#define First_LEG   0
#define Second_LEG  1
#define Third_LEG   2
#define Fourth_LEG  3
#define Last_LEG    4

// Gains for motor PID
#define Kp_left      30 //Proportional gain
#define Kd_left      0 //Derivative gain
#define Ki_left     0.7 //Integral gain
#define Kp_right     30 //Proportional gain
#define Kd_right     0 //Derivative gain
#define Ki_right    0.7 //Integral gain
#define H_PGAIN     1.2
#define H_IGAIN   0.002
#define H_DGAIN    -2.5


// Clas instances.
Motor_c       L_Motor( L_PWM_PIN, L_DIR_PIN);                       // To set left motor power.
Motor_c       R_Motor( R_PWM_PIN, R_DIR_PIN);
LineSensor_c  line_left(LINE_LEFT_PIN);
LineSensor_c  line_centre(LINE_CENTRE_PIN);
LineSensor_c  line_right(LINE_RIGHT_PIN);
PID_c         left_PID(Kp_left, Ki_left, Kd_left);
PID_c         right_PID(Kp_right, Ki_right, Kd_right);
PID_c         H_PID(H_PGAIN, H_IGAIN , H_DGAIN );
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
  //line_left.calibrate();  
  //line_centre.calibrate();
  //line_right.calibrate();

  spd_update_ts = millis(); //Initialize time stamps
  pid_update_ts = millis();
  pose_print_ts = millis();
  initial_state_ts = millis();
  T_update_ts =   millis();
  
  el_spd =         0; 
  er_spd =         0;
  last_el_count =  0;
  last_er_count =  0;
  
  dl_forward =  .7;
  dr_forward =  .7;
  d_turn =       0;
  T =            0;
  pwr_l =        0;
  pwr_r =        0;

  // We set the robot to start kinematics
  // X = 0, Y = 0, Theta = 0.05
  RomiPose.setPose( 0, 0, 0 );

 }

// put your main code here, to run repeatedly:
void loop() {
  // Always update kinematics
  RomiPose.update( count_el, count_er );
  printPose();  //print pose parameters
  
  if ( STATE == INITIAL_STATE  ) {
    intialisingBeeps();
  }
 else if ( STATE == FOLLOW_SQUARE  ) {
  if (RomiPose.theta<7){
  speed_update(); //get update on speed
  pidUpdate();
  square_run(500,500); //provide lenght of square in mm  
  }
  else{
   STATE = 2;}
 }    
 else if ( STATE == RESTING){
  L_Motor.setPower(0);
  R_Motor.setPower(0);
  //Serial.println("Resting...........");
  delay(500);  
  }
  
  //print_count();  //prints the counts for both wheels
  delay(1);

} // End of main loop

void intialisingBeeps() {
// fuction for state 0
  unsigned long initial_state_dt = millis() - initial_state_ts;
  if ( initial_state_dt < 3000 ) {

    analogWrite(6, 10);
    delay(100);
    analogWrite(6, 0);
    delay(900);
    
  } else {  
    
  left_PID.reset();
  right_PID.reset();
  H_PID.reset();
  
  STATE =   1;
  }
}

void pidUpdate(){
// output_signal <----PID-- demand, measurement  
  unsigned long pid_update_dt = millis() - pid_update_ts;
  
  if ( pid_update_dt > 20) {
    pid_update_ts= millis();
    //d_turn = H_PID.update(0.05,RomiPose.theta);
    pwr_l = left_PID.update(dl_forward-d_turn, el_spd);
    pwr_r = right_PID.update(dr_forward+d_turn, er_spd);
    //left_PID.printComponents();
    //Serial.println(dl_forward-d_turn);

    
  L_Motor.setPower(pwr_l);
  R_Motor.setPower(pwr_r);
  
  }
 }

void square_run(int X, int Y){

  if(state_sqr == First_LEG){
    if(RomiPose.x < X-40){
      //Taking Tan to avoid big radin value when less 
      //than Zero. Tan will be + and - around 0 radian
     d_turn = H_PID.update(0,tan(RomiPose.theta));     
    }
    else{
    state_sqr = 1; 
    }
  }

  if(state_sqr == Second_LEG){
    if(RomiPose.y < Y-40){
      //Sending required Theta value to T_Update 
      T_update(1.57);
      
     //Chek for edge case where theta is less
     //than Zero deg, convert it to negative value.
     float t = RomiPose.theta;
     if(t>5){
      t = TWO_PI-t;
     } else{
       t=t;
     }
  
     d_turn = H_PID.update(T,t); 
     //Serial.println(T);   
    }
    else{
    state_sqr = 2; 
    }
  }
  
  if(state_sqr == Third_LEG){
    if(RomiPose.x > 40 ){
     T_update(3.14);
     d_turn = H_PID.update(T,RomiPose.theta);    
    }
    else{
    state_sqr = 3; 
    } 
  }

  if(state_sqr == Fourth_LEG){
    if(RomiPose.y > 40 ){
     T_update(4.71);
     d_turn = H_PID.update(T,RomiPose.theta);    
    }
    else{
    state_sqr = 4; 
    }  
  }
    
  if(state_sqr == Last_LEG){
   if(RomiPose.theta < 6.2 ){
     T_update(6.2);
     d_turn = H_PID.update(T,RomiPose.theta);      
    }
  else{
    STATE = 2;
    }
  }


}

void T_update(float d){
    unsigned long T_update_dt = millis()-T_update_ts;
     if( T_update_dt > 3 ){
      T_update_ts = millis();               
      if( T < d ){
      T = T+.01;
      }
      else{
        T = d;
      }

     }
}

void printPose() {
  unsigned long pose_print_dt = millis() - pose_print_ts;

  if (pose_print_dt > 100) {
    pose_print_ts = millis();
    Serial.print(RomiPose.x);
    Serial.print("\t");
    Serial.print(RomiPose.y);
    Serial.print( "\t" );
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
