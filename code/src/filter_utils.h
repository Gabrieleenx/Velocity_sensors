#pragma once

double dxToMilimeter(int dx, int countsPerInch);

struct SensorDataDelta {
  int dx;
  int dy;
  void reset(){
    dx= 0;
    dy = 0;
  }
};


class TargetTrackingLoopFilter {
  private:
    double pos;
    double pos_est;
    double vel_est;
    double vel_int;
    double ki;
    double kp;
    int countsPerInch;
    unsigned long lastTimeMicros;

  public:
    TargetTrackingLoopFilter(double kp_, double ki_, int countsPerInch_);
    double update(int dx);
};

class VelocityFilter {
  private:
    int countsPerInch;
    unsigned long lastTimeMicros;
    double a;
    double b;

  public:
    VelocityFilter(double a_, double b_, int countsPerInch_);
    double update(int dx);
    void update_a(double a_){
      a = a_;
      Serial.print("a in filter updated to: ");
      Serial.println(a,5);
    }
    void update_b(double b_){
      b = b_;
      Serial.print("b in filter updated to: ");
      Serial.println(b,5);
    }
};

class FilterSensorData{
    private:
        VelocityFilter ttlf_dx_1;
        VelocityFilter ttlf_dy_1;
        VelocityFilter ttlf_dx_2;
        VelocityFilter ttlf_dy_2;
        VelocityFilter ttlf_dx_3;
        VelocityFilter ttlf_dy_3;
        double d_Sensor;

        double vx;
        double vy;
        double omega;
    public:
        FilterSensorData(double kp_, double ki_, int countsPerInch_);

        void update(SensorDataDelta sensor_1, 
                    SensorDataDelta sensor_2, 
                    SensorDataDelta sensor_3);
        
        double get_vx(){
            return vx;
        }

        double get_vy(){
            return vy;
        }

        double get_omega(){
            return omega;
        }

        void update_calibration(int cal_nr_1, int cal_nr_2, double value);

};
