#include <Wire.h>
#include <LSM6.h>
#define FS_2 0.061
#define FS_245 8.75 

float SensorData, KalmanFilterData;
float Xt, Xt_update, Xt_prev;
float Pt, Pt_update, Pt_prev;
float Kt, R, Q;


float bias;


LSM6 imu;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  
  imu.enableDefault();
  //imu.writeReg(LSM6::CTRL1_XL, 0b01011000);

  float c=0;
  float sum;
  float reading;
  
  while ( c<100 )
  {
   imu.read();
  reading = imu.g.z * 8.75;
  sum = reading+sum;
  c=c+1;  
  }
  bias = sum/c;
  
  Serial.print("The sum of readings is: ");
  Serial.println(sum);
  Serial.print("The bias of readings is: ");
  Serial.println(bias);
  delay(500);

  R = 100;
  Q = 1;
  Pt_prev = 1;

  imu.read();
   
  
  SensorData=(imu.g.z * 8.75)-bias;

  Xt_prev=SensorData;
  
  
}

void loop()
{
  //read_imu_raw();
  //read_imu_cov();
  //read_g_z();
  kalman_s();
  


}

void kalman_s()
{
  imu.read();
  float reading = imu.g.z * 8.75;
  
  SensorData=reading-bias;

  Xt_update = Xt_prev;
  Pt_update = Pt_prev + Q;
  Kt = Pt_update / (Pt_update + R);
  Xt = Xt_update + ( Kt * (SensorData - Xt_update));
  Pt = (1 - Kt) * Pt_update;
  Xt_prev = Xt; 
  Pt_prev = Pt;
  KalmanFilterData = Xt;
  //Serial.print(SensorData);
  //Serial.print(",");
  Serial.println(KalmanFilterData);
  //Serial.println();
  delay(1); 
}

void read_g_z()
{ 
  imu.read();
  Serial.println(imu.g.z*FS_245-bias);

  delay(100); 
}

void read_imu_cov(){
  imu.read();

  Serial.print("A: ");
  Serial.print(imu.a.x*FS_2);
  Serial.print(" ");
  Serial.print(imu.a.y*FS_2);
  Serial.print(" ");
  Serial.print(imu.a.z*FS_2);
  Serial.print("\t G:  ");
  Serial.print(imu.g.x*FS_245);
  Serial.print(" ");
  Serial.print(imu.g.y*FS_245);
  Serial.print(" ");
  Serial.println(imu.g.z*FS_245);

  delay(100);  
}


void read_imu_raw(){

  imu.read();

  Serial.print("A: ");
  Serial.print(imu.a.x);
  Serial.print(" ");
  Serial.print(imu.a.y);
  Serial.print(" ");
  Serial.print(imu.a.z);
  Serial.print("\t G:  ");
  Serial.print(imu.g.x);
  Serial.print(" ");
  Serial.print(imu.g.y);
  Serial.print(" ");
  Serial.println(imu.g.z);

  delay(100);
}
