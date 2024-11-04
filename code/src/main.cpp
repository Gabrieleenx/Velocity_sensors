#include <Arduino.h>
#include "esp_task_wdt.h"
#include "FreeRTOS.h"
#include "task.h"
#include <EEPROM.h>
#include "filter_utils.h"
const int data_pin_sensor_1 = 4;
const int clk_pin_sensor_1 = 5;

const int data_pin_sensor_2 = 6;
const int clk_pin_sensor_2 = 7;

const int data_pin_sensor_3 = 15;
const int clk_pin_sensor_3 = 16;

const String sensor_name = "sensor_2";

int counts_per_inch = 3200; // should be 1600, but measurements say otherwise. sensor resulution.

const double d = 0.026; // [m] from center to sensor. 

int correct_prod_id1 = 0x31; // ID of sensor 

int individual_sensor = 0;

const uint8_t sensor_reg_dx = 0x03; // 0x03
const uint8_t sensor_reg_dy = 0x04; // 0x04
const uint8_t sensor_reg_motion = 0x02;
const uint8_t sensor_reg_PId1 =0x00;
unsigned long lastTimeMicros_;

const double tracking_loop_Kp = 40;
const double tracking_loop_Ki = 500;

bool connected_sensor_1 = 0;
bool connected_sensor_2 = 0;
bool connected_sensor_3 = 0;

// FreeRTOS handles for the tasks and mutex
TaskHandle_t sensorTaskHandle;
TaskHandle_t serialTaskHandle;
SemaphoreHandle_t mutex;

// Global data storage


SensorDataDelta sensor_1_delta = {0, 0};
SensorDataDelta sensor_2_delta = {0, 0};
SensorDataDelta sensor_3_delta = {0, 0};

// Useful functions

void setAllDataInput(){
  pinMode (data_pin_sensor_1, INPUT); 
  pinMode (data_pin_sensor_2, INPUT); 
  pinMode (data_pin_sensor_3, INPUT); 
}

void setAllDataOutput(){
  pinMode (data_pin_sensor_1, OUTPUT); 
  pinMode (data_pin_sensor_2, OUTPUT); 
  pinMode (data_pin_sensor_3, OUTPUT); 
}


void setAllClkOutput(){
  pinMode (clk_pin_sensor_1, OUTPUT); 
  pinMode (clk_pin_sensor_2, OUTPUT); 
  pinMode (clk_pin_sensor_3, OUTPUT); 
}

void digitalWriteDataAll(int w){
 
  digitalWrite(data_pin_sensor_3, w);
  digitalWrite(data_pin_sensor_2, w);
  digitalWrite(data_pin_sensor_1, w);
          
}

void digitalWriteClkAll(int w){
  digitalWrite(clk_pin_sensor_1, w);
  digitalWrite(clk_pin_sensor_3, w); 
  digitalWrite(clk_pin_sensor_2, w);  
}



void sensorWriteReg(uint8_t address, uint8_t data){
  // Set MSB high
  address |= 0x80;
  setAllDataOutput();
  int t_hold = 5;
  for (int i=7; i>=0; i--){
    digitalWriteClkAll(LOW);
    digitalWriteDataAll(address & (1 << i));
    delayMicroseconds(t_hold);
    digitalWriteClkAll(HIGH);
    delayMicroseconds(t_hold);
  }
  for (int i=7; i>=0; i--){
    digitalWriteClkAll(LOW);
    digitalWriteDataAll(data & (1 << i));
    delayMicroseconds(t_hold);
    digitalWriteClkAll(HIGH);
    delayMicroseconds(t_hold);
  }
}

void sensorReadReg(const uint8_t address, int* r){
  setAllDataOutput();
  int t_hold = 5;

  for (int i=7; i>=0; i--){
    digitalWriteClkAll(LOW);
    digitalWriteDataAll(address & (1 << i));
    delayMicroseconds(t_hold);
    digitalWriteClkAll(HIGH);
    delayMicroseconds(t_hold);
  }

  digitalWriteDataAll(LOW); 
  delayMicroseconds(2);
  setAllDataInput();
  delayMicroseconds(2);
  
  for (int i=7; i>=0; i--){        
    digitalWriteClkAll(LOW);
    delayMicroseconds(t_hold);
    digitalWriteClkAll(HIGH);
    delayMicroseconds(t_hold);
    
    r[2] |= (digitalRead(data_pin_sensor_3) << i);
    r[1] |= (digitalRead(data_pin_sensor_2) << i);
    r[0] |= (digitalRead(data_pin_sensor_1) << i);
    
  }

}


void reSynchSensor(){
  // This assumes same procedure as PAW3204 
  setAllDataOutput();
  digitalWriteDataAll(LOW);
  setAllDataInput(); // set in HI-Z
  digitalWriteClkAll(HIGH);
  delayMicroseconds(5);
  digitalWriteClkAll(LOW); 
  delayMicroseconds(2);
  digitalWriteClkAll(HIGH); 
  delay(5); // t_siwtt, timeout watchdog 1.7 ms
  sensorWriteReg(0x85, 0x21); // reg: 85 data: 0x21   00100001 write operation mode    force wake up with disabled sleep
  sensorWriteReg(0x86, 0x06); // 86 06   write configuration      CPI 1600  // 00000110
}


void getProdID(signed char* prod_id){
  int r[3] = {0,0,0};
  sensorReadReg(sensor_reg_PId1, r);
  prod_id[0] = (signed char) r[0];
  prod_id[1] = (signed char) r[1];
  prod_id[2] = (signed char) r[2];
}


void sensorDx(signed char* dx){
  int r[3] = {0,0,0};
  sensorReadReg(sensor_reg_dx, r);
  dx[0] = (signed char) r[0];
  dx[1] = (signed char) r[1];
  dx[2] = (signed char) r[2];
} 

void sensorDy(signed char* dy){
  int r[3] = {0,0,0};
  sensorReadReg(sensor_reg_dy, r);
  dy[0] = (signed char) r[0];
  dy[1] = (signed char) r[1];
  dy[2] = (signed char) r[2];
} 

int extractBit7(int value) {
  return (value >> 7) & 1;
}

void sensorMotion(int* motion){
  int r[3] = {0,0,0};
  sensorReadReg(sensor_reg_motion, r);
  motion[0] = extractBit7(r[0]);
  motion[1] = extractBit7(r[1]);  
  motion[2] = extractBit7(r[2]);
}

void setSensorSettings(){
  // set up 3 sensors
  int i = 0;
  signed char sensor_prod_id[] = {0,0,0};

  while(i < 100){
    reSynchSensor();
    getProdID(sensor_prod_id);
    if (sensor_prod_id[0] == correct_prod_id1){
      connected_sensor_1 = 1;
    }
    else{
      Serial.print("Chip 1 prodid");
      Serial.println(sensor_prod_id[0]);   
    }
    if (sensor_prod_id[1] == correct_prod_id1){
      connected_sensor_2 = 1;
    }
    else{
      Serial.print("Chip 2 prodid");
      Serial.println(sensor_prod_id[1]);   
    }
    if (sensor_prod_id[2] == correct_prod_id1){
      connected_sensor_3 = 1;
    }    else{
      Serial.print("Chip 3 prodid");
      Serial.println(sensor_prod_id[2]);   
    }

    if (connected_sensor_1 == 1 && connected_sensor_2 == 1 && connected_sensor_3 == 1){
      break;
    }
    i++;
    delay(1);
  }
  sensorWriteReg(0x85, 0x21); // reg: 85 data: 0x21   00100001 write operation mode    force wake up with disabled sleep
  sensorWriteReg(0x86, 0x86); // chip reset 
  delayMicroseconds(3); 
  sensorWriteReg(0x85, 0x21); // reg: 85 data: 0x21   00100001 write operation mode    force wake up with disabled sleep
  sensorWriteReg(0x86, 0x06); // 86 06   write configuration      CPI 1600  // 00000110
}

void readSerial(String& msg, FilterSensorData &sensor_filter){
  while (Serial.available()) {
    // get the new byte:
    char ch = Serial.read();
    msg += ch;
    // end of user input
    if (ch == '\n') {
      if (msg[0] == 'n'){
      // send name of sensor 
      Serial.print("n");
      Serial.print("/t");
      Serial.println(sensor_name); 
      }
      if (msg[0] == 'c'){
        individual_sensor = msg[1] - '0';
      }
      if (msg[0] == 'k'){
        /* 
        cal_nr = {0,x: radious, 1,0: s_1_x_a, 1,1: s_1_x_b,
                  1,2: s_1_y_a, 1,3: s_1_y_b,
                  2,0: s_2_x_a, 2,1: s_2_x_b,
                  2,2: s_2_y_a, 2,3: s_2_y_b,
                  3,0: s_3_x_a, 3,1: s_3_x_b,
                  3,2: s_3_y_a, 3,3: s_3_y_b,}
        */
        int cal_nr_1 = msg[1] - '0';
        int cal_nr_2 = msg[2] - '0';
        String substr = msg.substring(3);

        double value = substr.toDouble();
        sensor_filter.update_calibration(cal_nr_1, cal_nr_2, value);
      }
      msg = "";
      return;
    }
  }
}



void sensorUpdate(){
  signed char dx[] = {0,0,0};
  signed char dy[] = {0,0,0};
  int motion[] = {0,0,0};

  // check that sensor is in sync by reading the sensor id
  signed char sensor_prod_id[] = {0,0,0};
  getProdID(sensor_prod_id);
  if (sensor_prod_id[0] == correct_prod_id1 && sensor_prod_id[1] == correct_prod_id1 && sensor_prod_id[2] == correct_prod_id1){
    
    sensorMotion(motion);

    sensorDy(dy);
    sensorDx(dx);
    if( xSemaphoreTake( mutex, ( TickType_t ) 10 ) == pdTRUE ){
      sensor_1_delta.dx += (int) dx[0];
      sensor_1_delta.dy += (int) dy[0];

      sensor_2_delta.dx += (int) dx[1];
      sensor_2_delta.dy += (int) dy[1];

      sensor_3_delta.dx += (int) dx[2];
      sensor_3_delta.dy += (int) dy[2];
      xSemaphoreGive( mutex );
    }
    vTaskDelay(1);

  }
  else{
    Serial.print("n");
    Serial.print("/t");
    Serial.println("resynch");
    Serial.print(sensor_prod_id[0]);
    Serial.print(" ");
    Serial.print(sensor_prod_id[1]);
    Serial.print(" ");
    Serial.println(sensor_prod_id[2]);
    reSynchSensor();
    return;
  }
}


void sensorLoop(void *pvParameters) {
  setSensorSettings();
  while (true) {
    sensorUpdate();
  }
}


void serialTask(void *pvParameters) {
  SensorDataDelta sensor_1_delta_copy = {0, 0};
  SensorDataDelta sensor_2_delta_copy = {0, 0};
  SensorDataDelta sensor_3_delta_copy = {0, 0};
  FilterSensorData sensor_filter(80, 300, counts_per_inch);
  String msg = "";
  while (true) {

    if( xSemaphoreTake( mutex, ( TickType_t ) 10 ) == pdTRUE ){
      sensor_1_delta_copy = sensor_1_delta;
      sensor_1_delta.reset();
      sensor_2_delta_copy = sensor_2_delta;
      sensor_2_delta.reset();
      sensor_3_delta_copy = sensor_3_delta;
      sensor_3_delta.reset();
      xSemaphoreGive( mutex );
    }
    if (individual_sensor == 0){
      sensor_filter.update(sensor_1_delta_copy, sensor_2_delta_copy, sensor_3_delta_copy);
      Serial.print("v");
      Serial.print("/t");
      Serial.print(sensor_filter.get_vx(), 4);
      Serial.print("/t");
      Serial.print(sensor_filter.get_vy(), 4);
      Serial.print("/t");
      Serial.println(sensor_filter.get_omega(), 4);
    }
    else{
      unsigned long currentTimeMicros = micros(); 
      // Overflows after 71.58 min
      if (currentTimeMicros < lastTimeMicros_){
        lastTimeMicros_ = currentTimeMicros;
      }
      double dt =  ((double)(currentTimeMicros - lastTimeMicros_))/1e6; 
      lastTimeMicros_ = currentTimeMicros;
      double vx = 0.0;
      double vy = 0.0;
      double omega = 0.0;
      if (individual_sensor == 1){
        vx = dxToMilimeter(sensor_1_delta_copy.dx, counts_per_inch)/dt;
        vy = dxToMilimeter(sensor_1_delta_copy.dy, counts_per_inch)/dt;
      }
      if (individual_sensor == 2){
        vx = dxToMilimeter(sensor_2_delta_copy.dx, counts_per_inch)/dt;
        vy = dxToMilimeter(sensor_2_delta_copy.dy, counts_per_inch)/dt;
      }
      if (individual_sensor == 3){
        vx = dxToMilimeter(sensor_3_delta_copy.dx, counts_per_inch)/dt;
        vy = dxToMilimeter(sensor_3_delta_copy.dy, counts_per_inch)/dt;
      }


      Serial.print("v");
      Serial.print("/t");
      Serial.print(vx, 4);
      Serial.print("/t");
      Serial.print(vy, 4);
      Serial.print("/t");
      Serial.println(sensor_filter.get_omega(), 4);

    }
   
    readSerial(msg, sensor_filter);
    vTaskDelay(6);
  }
}


void setup() {
  Serial.begin(115200);
  //Serial.println("Serial start");
  mutex = xSemaphoreCreateMutex();

  delay(10);

  // set pins 
  setAllClkOutput();
  setAllDataInput();

  //Serial.println("Begin setup");
  delay(1000);
  //Serial.println("Setup done");

  xTaskCreatePinnedToCore(
    sensorLoop,       // Task function
    "SensorTask",     // Name of the task
    2048,               // Stack size (bytes)
    NULL,               // Parameter to pass
    1,                  // Task priority
    &sensorTaskHandle,// Task handle
    0                   // Core to pin the task to (0 or 1)
  );
  
  xTaskCreatePinnedToCore(
    serialTask,       // Task function
    "SerialTask",     // Name of the task
    2048,               // Stack size (bytes)
    NULL,               // Parameter to pass
    1,                  // Task priority
    &serialTaskHandle, // Task handle
    1                   // Core to pin the task to (0 or 1)
  );
}

void loop() { 
// Do nothing 
}

