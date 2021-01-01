#ifndef _IMU_CALC_
#define _IMU_CALC_

#define VEL_GAIN   10000
#define OFFSET_DOWN_VAL 1.5
#define OFFSET_UP_VAL 5.0
#define ANG_COEF 0.13
#define ROT_MEAN 44.15
#define ROT_LIMIT 1.0

class imuCalculator {

  private:
    uint64_t timer;
    float accOffsetDown{OFFSET_DOWN_VAL};
    float accOffsetUp{OFFSET_UP_VAL};
  public:
    float acceleration{0};
    float distance{0};
    float velocity{0};
    float rotation{0};
    float Xnew{0}, Ynew {0}; // Y - coordinate

    imuCalculator() {
      //default ctor.
    }

    imuCalculator(float offsetUp, float offsetDown) :
      accOffsetUp{offsetUp}, accOffsetDown{offsetDown}  {

    }

    auto getAcc()->float {
      return acceleration;
    }

    void calcVelDist() {
      double dt_ = (double)(micros() - timer) / 1000000.0; // Calculate delta time
      timer = micros();
      velocity =  acceleration * dt_ * VEL_GAIN;
      distance += 0.5 * velocity * dt_;
    }

    void getCoordinate (float rot, float acc) {
      //calculate x - y position
      static float last_distance {0};
      getAccelMean(acc);
      updateRotation(rot);
      Ynew += (distance - last_distance) * sin(rotation * PI / 180);
      Xnew += (distance - last_distance) * cos(rotation * PI / 180);
      last_distance = distance;
    }

    void updateRotation(float rot) {
      if (((rot - ROT_MEAN) > ROT_LIMIT) || ((rot - ROT_MEAN) < -ROT_LIMIT)) {
        rotation += (rot - ROT_MEAN) * ANG_COEF;
      }
    }

    bool getAccelMean(float acc) {
      //Get the mean of value between two limit.
      static float collectValue{ 0 };
      static int numberReading{ 0 };
      static bool trigger{ false };

      //Update the distance and velocity params
      calcVelDist();
      if ((acc > accOffsetDown) && (acc < accOffsetUp)) {
        trigger = true;
        collectValue += acc;
        numberReading++;
        return true;
      } else {
        if (trigger && (acc < accOffsetDown)) {
          //get the mean of values
          collectValue /= (float)numberReading;
          acceleration = collectValue;
          //Reset number of reading val
          numberReading = 0;
          collectValue = 0;
          trigger = false;
          return true;
        }
      }
      return false;
    }

    boolean turnDegree(float desiredAngle) {
      static float initialAngle{ 0 };
      static bool  enterTrigger{ true };

      if (enterTrigger) {
        initialAngle = rotation;
        enterTrigger = false;
      }

      float locAngle = (rotation - initialAngle);
      
      if (desiredAngle > 0) {
        //compare the angle difference for positive values
        if (desiredAngle > locAngle) {
          return false;
        } else {
          enterTrigger = true;
          return true;
        }
      } else if (desiredAngle < 0) {
        //compare the angle difference for positive values
        if (desiredAngle < locAngle) {
          return false;
        }
        else {
          enterTrigger = true;
          return true;
        }
      }
      return false;
    }
};

#endif _IMU_CALC_


