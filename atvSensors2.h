#ifndef atvSensors2_h
#define atvSensors2_h

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>
#include <SparkFun_u-blox_GNSS_v3.h> 
#include <driver/pcnt.h>
#define PCNT_UNIT    PCNT_UNIT_0
SFE_UBLOX_GNSS GPS_driver;
Adafruit_LSM303_Accel_Unified accel_driver = Adafruit_LSM303_Accel_Unified(12345);
Adafruit_LIS2MDL mag_driver = Adafruit_LIS2MDL(54321);
/*
class containing sensor configuration as well as sensor values.
*/
class atvSensors2 {                                      
  public:
    atvSensors2(int velocity_pin, int steering_pot_pin, int hitch_pin, int frequency){
      pinMode(velocity_pin,INPUT);
      pinMode(steering_pot_pin,INPUT);
      pinMode(hitch_pin,INPUT_PULLUP);
      this->velocity_pin = velocity_pin;
      this->steering_pot_pin = steering_pot_pin;
      this->hitch_pin = hitch_pin;
      period_ms = 1000/frequency;
    }         //class constructor
    struct mag_data{                                                 //struct containg magnotometer values  
      float x;
      float y;
      float z;
    };
    struct accel_data{                                               //struct containing accelerometer values
      float x;  
      float y;
      float z;
    };
    struct position_data{                                            //struct containing gps information
      float latitude;
      float longitude;
    };
    void init(){
      configureAccelerometer();
      configureMagnotometer();
      configureEncoder();
      configureSteeringPot();
    }                                                            //intitlization of sensors method and isr

    void update(){
      sensors_event_t a,m;
      accel_driver.getEvent(&a);
      accel.x = a.acceleration.x;
      accel.y = a.acceleration.y;
      accel.z = a.acceleration.z;

      mag_driver.getEvent(&m);
      mag.x = m.magnetic.x;
      mag.y = m.magnetic.y;
      mag.z = m.magnetic.z;

      steering_pot = analogRead(steering_pot_pin);
      calcVelocity();
      delay(period_ms);


    }                                                                      //update sensors method, updates all sensors readings
                                                                  //returns current saved accelerometr data  
    void printAll(){
      Serial.print("accel.x = ");
      Serial.print(accel.x);
      Serial.print(" accel.y = ");
      Serial.print(accel.y);
      Serial.print(" accel.z = ");
      Serial.println(accel.z);

      Serial.print("Mag.x = ");
      Serial.print(mag.x);
      Serial.print(" Mag.y = ");
      Serial.print(mag.y);
      Serial.print(" mag.z = ");
      Serial.println(mag.z);

      Serial.print("Steering Pot: ");
      Serial.println(steering_pot);

      Serial.print("mph: ");
      Serial.println(velocity_mph);
    }                                                               //prints all current saved sensor data


   

  private:
    int velocity_pin;                                          //pin declaration for velocity encoder
    int steering_pot_pin;                                      //pin declaration for Linear Actuator Potentiometer
    int hitch_pin;                                             //pin declararion for hitch pin
    int period_ms;
    static volatile bool  hitch_flag;                               //variable intilization for the hitch limit switch 
    unsigned int steering_pot;

    float wheel_diameter_inches = 18.0;                             //diamter of wheel hub
    float slots = 60.0;                                             //number of slots on encoder
    float circumference_inches = 3.1415926 * wheel_diameter_inches;     //circumference of wheel diameter
    float velocity_mph = 0.00;
    mag_data       mag = {0,0,0};                                       //initizialtion of mag,accel,pos data
    accel_data     accel = {0,0,0};
    position_data position = {0,0};
    void configureAccelerometer(){
      
      if (!accel_driver.begin()) {
        Serial.println("LSM303 Accelerometer not detected. Check connections!");
        while (1);
      }
      else{
        lsm303_accel_mode_t accel_mode = LSM303_MODE_HIGH_RESOLUTION;                                       //lsm303_accel_mode_t is an alias type def, it uses enum data type
        accel_driver.setMode(accel_mode);                                                                //set accel to high resolution mode
        Serial.print("Accelerometer Intialized! Mode: ");
        accel_mode = accel_driver.getMode();
        switch(accel_mode){
          case(LSM303_MODE_LOW_POWER):
            Serial.println("low power mode :(");
            break;
          case(LSM303_MODE_NORMAL):
            Serial.println("normal mode :(");
            break;
          case(LSM303_MODE_HIGH_RESOLUTION):
            Serial.println("High Resolution Mode!");      
          }
      }
    }

    void configureMagnotometer(){                                                            //configure magnoometer set sample rate to 100 hz
      if (!mag_driver.begin()) {
        Serial.println("LSM303 Magnotometer not detected. Check connections!");
        while (1);
      }
      else{
        Serial.print("Magnotometer intialized, frequenzy: ");
        lis2mdl_rate_t mag_rate = LIS2MDL_RATE_100_HZ;
        mag_driver.setDataRate(mag_rate);
        mag_rate = mag_driver.getDataRate();
        switch(mag_rate){
          case(LIS2MDL_RATE_10_HZ):
            Serial.println("10 Hz");
            break;
          case(LIS2MDL_RATE_20_HZ):
            Serial.println("20 HZ");
            break;
          case(LIS2MDL_RATE_50_HZ):
            Serial.println("50 Hz");
            break;
          case(LIS2MDL_RATE_100_HZ):
            Serial.println("100 Hz");
            break;      
          }
      }
    }   
    void configureEncoder(){                                                              //configure PCNT peripheral on esp-32
      Serial.print("Intializing PCNT Module: ");
      pcnt_config_t pcnt_config = {
        .pulse_gpio_num = velocity_pin,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768
    };
      pcnt_unit_config(&pcnt_config);
      pcnt_counter_clear(PCNT_UNIT);
      pcnt_counter_resume(PCNT_UNIT);
      uint32_t status;
      esp_err_t err = pcnt_get_event_status(PCNT_UNIT, &status);

      if (err == ESP_ERR_INVALID_STATE) {
        Serial.println("Failed to Initialize PCNT"); // PCNT is NOT initialized
      }
      else{
        Serial.println("Sucessfully Intialized");
      }
    }   
                

    void configureSteeringPot(){                                                        //check to see if linear actuator potentiometer is correctly wired
      Serial.print("checking linear actuator potentiomter");
      steering_pot = analogRead(steering_pot_pin);
      if(steering_pot > 3000 or steering_pot < 1000){
        Serial.println("Pot value out of range, check wiring");
      }
      else{
        Serial.println("Success!");
      }
    }

    void configureHitch(){                                                                    //check to see if hitch pin is properly wired
      Serial.print("Checking Hitch Switch: ");
      if(digitalRead(hitch_pin)){
        Serial.println("failed check switch");
      }
      else{
        Serial.println("switch operational");
      }
    }

    void configureGPS(){                                                                          //configure gps
      Serial.print("Intializing GPS: ");
      if(!GPS_driver.begin()){
        Serial.println("Failed to intialize, check wiring");
      }
      else{
        Serial.println("success!")
      }
    }

    void calcVelocity(){                                                                              //calculated velocity based off of time interval and pcnt
      int16_t count = 0;
      float time_seconds = (float)period_ms / 1000;
      pcnt_get_counter_value(PCNT_UNIT, &count);
      float revolutions =  (float)count/slots;
      float distance_inches = revolutions * circumference_inches;
      velocity_mph = distance_inches/time_seconds * 3600.0/(12.0*5280.0);
      pcnt_counter_clear(PCNT_UNIT);
    } 


};


#endif


//define a public variable which controls time at which we are updating