#include <Arduino.h>
#include "filter_utils.h"

double dxToMilimeter(int dx, int countsPerInch){
  double countsPerMilimeter = (double) countsPerInch / 25.4;
  double dx_mm = (double) dx/countsPerMilimeter;
  return dx_mm;
}


TargetTrackingLoopFilter::TargetTrackingLoopFilter(double kp_, double ki_, int countsPerInch_){
  //https://www.embeddedrelated.com/showarticle/530.php
  lastTimeMicros = micros();
  pos = 0.0;
  pos_est = 0.0;
  vel_est = 0.0;
  vel_int = 0.0;
  kp = kp_;
  ki = ki_;
  countsPerInch = countsPerInch_;
}

double TargetTrackingLoopFilter::update(int dx){
  // TOOD add calibration here 
  
  pos += dxToMilimeter(dx, countsPerInch);
  unsigned long currentTimeMicros = micros(); 
  // Overflows after 71.58 min
  if (currentTimeMicros < lastTimeMicros){
    lastTimeMicros = currentTimeMicros;
  }
  double dt =  ((double)(currentTimeMicros - lastTimeMicros))/1e6; 
  lastTimeMicros = currentTimeMicros;
  
  pos_est += vel_est*dt;
  double pos_error = pos - pos_est;
  vel_int += pos_error * ki * dt; 
  vel_est = pos_error*kp + vel_int;
  
  double vel_est_ = dxToMilimeter(dx, countsPerInch)/dt;
  return vel_est_;
}

VelocityFilter::VelocityFilter(double a_, double b_, int countsPerInch_){
  lastTimeMicros = micros();
  countsPerInch = countsPerInch_;
  a = a_;
  b = b_;
}

double VelocityFilter::update(int dx){
  unsigned long currentTimeMicros = micros(); 
  // Overflows after 71.58 min
  if (currentTimeMicros < lastTimeMicros){
    lastTimeMicros = currentTimeMicros;
  }
  double dt =  ((double)(currentTimeMicros - lastTimeMicros))/1e6; 
  lastTimeMicros = currentTimeMicros;

  double vel_est_ = dxToMilimeter(dx, countsPerInch)/dt;
  double cal_vel_est = vel_est_/(b + a*vel_est_);
  return cal_vel_est;
}


FilterSensorData::FilterSensorData(double kp_, double ki_, int countsPerInch_)
    : ttlf_dx_1(0.0, 1.0, countsPerInch_),
      ttlf_dy_1(0.0, 1.0, countsPerInch_),
      ttlf_dx_2(0.0, 1.0, countsPerInch_),
      ttlf_dy_2(0.0, 1.0, countsPerInch_),
      ttlf_dx_3(0.0, 1.0, countsPerInch_),
      ttlf_dy_3(0.0, 1.0, countsPerInch_) {
  d_Sensor = 20.3; // [mm] from center to sensor. 
}

void FilterSensorData::update_calibration(int cal_nr_1, int cal_nr_2, double value){
  if (cal_nr_1 == 0){
    d_Sensor = value;
    Serial.print("radius is updated to: ");
    Serial.println(d_Sensor);
    return;
  }
  if (cal_nr_1 == 1){
    if (cal_nr_2 == 0) {
      ttlf_dx_1.update_a(value);
      return;
    }
    if (cal_nr_2 == 1) {
      ttlf_dx_1.update_b(value);
      return;
    }
    if (cal_nr_2 == 2) {
      ttlf_dy_1.update_a(value);
      return;
    }
    if (cal_nr_2 == 3) {
      ttlf_dy_1.update_b(value);
      return;
    }
  }

  if (cal_nr_1 == 2){
    if (cal_nr_2 == 0) {
      ttlf_dx_2.update_a(value);
      return;
    }
    if (cal_nr_2 == 1) {
      ttlf_dx_2.update_b(value);
      return;
    }
    if (cal_nr_2 == 2) {
      ttlf_dy_2.update_a(value);
      return;
    }
    if (cal_nr_2 == 3) {
      ttlf_dy_2.update_b(value);
      return;
    }
  }

  if (cal_nr_1 == 3){
    if (cal_nr_2 == 0) {
      ttlf_dx_3.update_a(value);
      return;
    }
    if (cal_nr_2 == 1) {
      ttlf_dx_3.update_b(value);
      return;
    }
    if (cal_nr_2 == 2) {
      ttlf_dy_3.update_a(value);
      return;
    }
    if (cal_nr_2 == 3) {
      ttlf_dy_3.update_b(value);
      return;
    }
  }


}



void FilterSensorData::update(SensorDataDelta sensor_1, 
                              SensorDataDelta sensor_2, 
                              SensorDataDelta sensor_3){
  // Velocity tracking loop
  double vx_1 = ttlf_dx_1.update(sensor_1.dx);
  double vy_1 = ttlf_dy_1.update(sensor_1.dy);

  double vx_2 = ttlf_dx_2.update(sensor_2.dx);
  double vy_2 = ttlf_dy_2.update(sensor_2.dy);

  double vx_3 = ttlf_dx_3.update(sensor_3.dx);
  double vy_3 = ttlf_dy_3.update(sensor_3.dy);


  vx = sqrt(3.0)/6.0 * vx_1 - 1.0/6.0 * vy_1 - sqrt(3.0)/6.0 * vx_2 - 1.0/6.0 * vy_2 + 1.0/3.0 * vy_3;
  vy = -1.0/6.0 * vx_1 - sqrt(3.0)/6.0 * vy_1 - 1.0/6.0 * vx_2 + sqrt(3.0)/6.0 * vy_2 + 1.0/3.0 * vx_3;
  omega = 1.0/(3.0*d_Sensor) * (vx_1 + vx_2 + vx_3);

  // calculate error for each sensor 
  double vx_1_hat = cos(PI/6)*vx - sin(PI/6)*vy + d_Sensor*omega;
  double vy_1_hat = -sin(PI/6)*vx - cos(PI/6)*vy;
  double vx_2_hat = -cos(-PI/6)*vx + sin(-PI/6)*vy + d_Sensor*omega;
  double vy_2_hat = sin(-PI/6)*vx + cos(-PI/6)*vy;
  double vx_3_hat = vy + d_Sensor*omega;
  double vy_3_hat = vx;

  double e_1 = sqrt(pow(vx_1-vx_1_hat, 2) + pow(vy_1-vy_1_hat, 2));
  double e_2 = sqrt(pow(vx_2-vx_2_hat, 2) + pow(vy_2-vy_2_hat, 2));
  double e_3 = sqrt(pow(vx_3-vx_3_hat, 2) + pow(vy_3-vy_3_hat, 2));
  double e_t = sqrt(pow(e_1, 2) + pow(e_2, 2) + pow(e_3, 2));
  double v_1 = sqrt(pow(vx_1, 2) + pow(vy_1, 2)); 
  double v_2 = sqrt(pow(vx_2, 2) + pow(vy_2, 2));
  double v_3 = sqrt(pow(vx_3, 2) + pow(vy_3, 2));

  double v_t = sqrt(pow(v_1, 2) + pow(v_2, 2) + pow(v_3, 2));;

  double e_r = e_t/(v_t + 1);

  double e_treashold = 0.2;
  // Check if error ratio is too high, indicating that a sensor is not reading surface 
  if (e_r > e_treashold){
    // Assume that the sensor with lowest velocity is the incorrect one.
    // For example if a sensor is outside the object it would read 0 velocity. 
    if (v_1 < v_2 && v_1 < v_3){
        //Serial.println("Sensor 1");
        vx = - sqrt(3.0)/8.0 * vx_2 - 3.0/8.0 * vy_2 + sqrt(3.0)/8.0 * vx_3 + 5.0/8.0 * vy_3;
        vy =   -3.0/8.0 * vx_2 + 7*sqrt(3.0)/24.0 * vy_2 + 3.0/8.0 * vx_3 - sqrt(3.0)/24.0 * vy_3;
        omega = 1.0/(2.0*d_Sensor) * vx_2 -  sqrt(3.0)/(6.0*d_Sensor) * vy_2 + 1.0/(2.0*d_Sensor) * vx_3 + sqrt(3.0)/(6.0*d_Sensor) * vy_3;
    }
    if (v_2 < v_1 && v_2 < v_3){
        //Serial.println("Sensor 2");
        vx =  sqrt(3.0)/8.0 * vx_1 - 3.0/8.0 * vy_1 - sqrt(3.0)/8.0 * vx_3 + 5.0/8.0 * vy_3;
        vy =   -3.0/8.0 * vx_1 - 7*sqrt(3.0)/24.0 * vy_1 + 3.0/8.0 * vx_3 + sqrt(3.0)/24.0 * vy_3;
        omega = 1.0/(2.0*d_Sensor) * vx_1 +  sqrt(3.0)/(6.0*d_Sensor) * vy_1 + 1.0/(2.0*d_Sensor) * vx_3 - sqrt(3.0)/(6.0*d_Sensor) * vy_3;
    }
    if (v_3 < v_2 && v_3 < v_1){
        //Serial.println("Sensor 3");
        vx =  sqrt(3.0)/4.0 * vx_1 - 1.0/4.0 * vy_1 - sqrt(3.0)/4.0 * vx_2 - 1.0/4.0 * vy_2;
        vy =   - sqrt(3.0)/3.0 * vy_1 + sqrt(3.0)/3.0 * vy_2;
        omega = 1.0/(2.0*d_Sensor) * vx_1 -  sqrt(3.0)/(6.0*d_Sensor) * vy_1 + 1.0/(2.0*d_Sensor) * vx_2 + sqrt(3.0)/(6.0*d_Sensor) * vy_2;
    }
  }

}