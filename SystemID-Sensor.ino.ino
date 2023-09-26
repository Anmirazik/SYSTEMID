/*macro definitions of Rotary angle sensor and LED pin*/
#include <LiquidCrystal_I2C.h>
#include <Stepper.h>

const int motor_speed = 7;
const int motor_step = 500;
Stepper motor(2048, 5, 7, 6, 8);

LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 16 column and 2 rows

#define ROTARY_ANGLE_SENSOR A0
#define LED 3  //the Grove - LED is connected to PWM pin D3 of Arduino
#define ADC_REF 5 //reference voltage of ADC is 5v.If the Vcc switch on the seeeduino
                    //board switches to 3V3, the ADC_REF should be 3.3
#define GROVE_VCC 5 //VCC of the grove interface is normally 5v
#define FULL_ANGLE 360 //full value of the rotary angle is 300 degrees

void setup()
{
    Serial.begin(9600);
    randomSeed(analogRead(A2)); // Seed the random number generator
    pinMode(ROTARY_ANGLE_SENSOR, INPUT);
    pinMode(LED,OUTPUT);
    lcd.init(); // initialize the lcd
    lcd.backlight();   
}

void loop()
{
  const unsigned long eventInterval_cw = 1000; //5 minute interval 
  const unsigned long eventInterval_ccw = 2000; //5 minute interval 
  unsigned long previousTime = 0;   
  float voltage;
  int sensor_value = analogRead(ROTARY_ANGLE_SENSOR);
  voltage = (float)sensor_value*ADC_REF/1023;
  float degrees = ((voltage*FULL_ANGLE)/GROVE_VCC)+29;
    double rads = degrees*(3.142/180);
    double v_bo = cos(rads)*(1.076) -(1.2);
    double angular_speed = (motor_speed*0.2)/0.683;
    double angular_speed_callibrate = angular_speed + (angular_speed*10/100);
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= eventInterval_cw) //publish every 5 minute 
    {
    Serial.println("");
    motor.setSpeed(motor_speed);
    motor.step(500);
    previousTime = currentTime;
    lcd.clear();
    lcd.setCursor(0, 2);
    lcd.print("MOTOR SPEED:");
    lcd.print(0);
    lcd.print("rad/s");
    lcd.setCursor(0, 3);
    lcd.print("ANGULAR AB:");
    lcd.print(0);
    lcd.print("rad/s");
    Serial.print("MOTOR SPEED:");
    Serial.print(0);
    Serial.println("rad/s");
    Serial.print("ANGULAR AB:");
    Serial.print(0);
    Serial.println("rad/s");
    motor.setSpeed(motor_speed);
    motor.step(-500); //ke atas
    }

    lcd.setCursor(0, 2);
    lcd.print("MOTOR SPEED:");
    lcd.print(motor_speed);
    lcd.print("rad/s");
    lcd.setCursor(0, 3);
    lcd.print("ANGULAR AB:");
    lcd.print(angular_speed_callibrate);
    lcd.print("rad/s");
    Serial.println(""); 
    // lcd.setCursor(0, 2);
    Serial.print("MOTOR SPEED:");
    Serial.print(motor_speed);
    Serial.println("rad/s");
    // lcd.setCursor(0, 3);
    Serial.print("ANGULAR AB:");
    Serial.print(angular_speed_callibrate);
    Serial.println("rad/s");
}
    // lcd.clear();
    // lcd.setCursor(0, 0);
    // lcd.print("MOTOR SPEED:");
    // lcd.print(0);
    // lcd.print("rad/s");
    // lcd.setCursor(0, 2);
    // lcd.print("ANGULAR AB:");
    // lcd.print(0);
    // lcd.print("rad/s");
    


  // if (motor_step == 500 || motor_step == -500) {
  //   lcd.clear();
  //   lcd.setCursor(0, 0);
  //   lcd.print("MOTOR SPEED:");
  //   lcd.print(0);
  //   lcd.print("rad/s");
  //   lcd.setCursor(0, 2);
  //   lcd.print("ANGULAR AB:");
  //   lcd.print(0);
  //   lcd.print("rad/s");
  // }

  // else
  // {


    // if (currentTime - previousTime >= eventInterval_ccw) //publish every 5 minute 
    // {
    // motor.setSpeed(20);
    // motor.step(-500);// pusing -360 
    // previousTime = currentTime;  
    // }

    // Serial.print("The angle between the mark and the starting position: ");
    // Serial.println(degrees);
    // Serial.print(" Angular Speed :");
    // Serial.println(angular_speed);
    // Serial.print(" Rads :");
    // Serial.println(rads);



  // }
    // delay(1000);
    // lcd.clear();
    // delay(1000);
    // int brightness;
    // brightness = map(degrees, 0, FULL_ANGLE, 0, 255);
    // analogWrite(LED,brightness);
    // delay(500);
    // lcd.clear();
    // lcd.setCursor(0, 0);
    // lcd.print("MOTOR SPEED:");
    // lcd.print(0);
    // lcd.print("rad/s");
    // lcd.setCursor(0, 2);
    // lcd.print("ANGULAR AB:");
    // lcd.print(0);
    // lcd.print("rad/s");


