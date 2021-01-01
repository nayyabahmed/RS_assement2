#ifndef _COMPLEMENTARY_FILTER_
#define _COMPLEMENTARY_FILTER_

#define WEIGHT 0.93

template<typename cls>
class ComplementFilter {

  private:
    uint32_t timer_;

    /* IMU Data */
    cls * imu_;
    double accX_, accY_, accZ_;
    double gyroX_, gyroY_, gyroZ_;
    double gyroXangle_, gyroYangle_, gyroZangle_; // Angle calculate using the gyro only
    double compAngleX_, compAngleY_, compAngleZ_; // Calculated angle using a complementary filter
    double kalAngleX_, kalAngleY_, kalAngleZ_; // Calculated angle using a Kalman filter

    float weight_combination = WEIGHT;

  public:

    ComplementFilter(cls const & imu_sensor) {
      imu_ = &imu_sensor;
      timer_ = micros();
    }

    void setWeight(float w) {
      if ( w > 1 || w < 0)
        return;
      weight_combination = w;
    }

    void updateImuValues() {
      imu_->read();
      accX_ = imu_->a.x;
      accY_ = imu_->a.y;
      accZ_ = imu_->a.z;

      gyroX_ = imu_->g.x;
      gyroY_ = imu_->g.y;
      gyroZ_ = imu_->g.z;
    }

    float getFilteredX() {
      return compAngleX_;
    }

    float getFilteredY() {
      return compAngleY_;
    }
    float getFilteredZ() {
      return compAngleZ_;
    }

    void updateFilter() {

      updateImuValues();

      double dt_ = (double)(micros() - timer_) / 1000000.0; // Calculate delta time
      timer_ = micros();

      float pitch = atan (accX_ / sqrt(accY_ * accY_ + accZ_ * accZ_)) * RAD_TO_DEG;
      float roll =  atan (accY_ / sqrt(accX_ * accX_ + accZ_ * accZ_)) * RAD_TO_DEG;
      float yaw =   atan (accZ_ / sqrt(accX_ * accX_ + accZ_ * accZ_)) * RAD_TO_DEG;

      double gyroXrate_ = gyroX_ * 0.00875; // Convert to deg/s
      double gyroYrate_ = gyroY_ * 0.00875; // Convert to deg/s
      double gyroZrate_ = gyroZ_ * 0.00875; // Convert to deg/s

      gyroXangle_ += gyroXrate_ * dt_; // Calculate gyro angle without any filter
      gyroYangle_ += gyroYrate_ * dt_;
      gyroZrate_  += gyroZrate_ * dt_;

      // Calculate the angle using a Complimentary filter
      compAngleX_ = weight_combination * (compAngleX_ + gyroXrate_ * dt_) + (1 - weight_combination) * roll;
      compAngleY_ = weight_combination * (compAngleY_ + gyroYrate_ * dt_) + (1 - weight_combination) * pitch;
      compAngleZ_ = weight_combination * (compAngleZ_ + gyroZrate_ * dt_) + (1 - weight_combination) * yaw;

      if (gyroXangle_ < -180 || gyroXangle_ > 180)
        gyroXangle_ = kalAngleX_;
      if (gyroYangle_ < -180 || gyroYangle_ > 180)
        gyroYangle_ = kalAngleY_;
      if (gyroZangle_ < -180 || gyroZangle_ > 180)
        gyroZangle_ = kalAngleZ_;

    }
};

#endif


