/*macro definitions of Rotary angle sensor and LED pin*/
#include <LiquidCrystal_I2C.h>

#include <Stepper.h>

const int motorspeed = 10;
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
  float randomNumber = random(4400, 5100) / 100.0;
  const unsigned long eventInterval_cw = 1000; //5 minute interval 
  const unsigned long eventInterval_ccw = 2000; //5 minute interval 
  unsigned long previousTime = 0;   
  float voltage;
  int sensor_value = analogRead(ROTARY_ANGLE_SENSOR);
  voltage = (float)sensor_value*ADC_REF/1023;
  // float degrees = ((voltage*FULL_ANGLE)/GROVE_VCC)+44;
  float degrees = randomNumber;

    double rads = degrees*(3.142/180);

    double v_bo = cos(rads)*(1.076) -(1.2);

    double angular_speed = v_bo/-0.25;

    unsigned long currentTime = millis();
    if (currentTime - previousTime >= eventInterval_cw) //publish every 5 minute 
    {
    motor.setSpeed(10);
    motor.step(500);// pusing -360 
    previousTime = currentTime;
    motor.setSpeed(10);
    motor.step(-500);// pusing -360   
    }

    // if (currentTime - previousTime >= eventInterval_ccw) //publish every 5 minute 
    // {
    // motor.setSpeed(20);
    // motor.step(-500);// pusing -360 
    // previousTime = currentTime;  
    // }

    Serial.print("The angle between the mark and the starting position: ");
    Serial.println(degrees);
    Serial.print(" Angular Speed :");
    Serial.println(angular_speed);
    Serial.print(" Rads :");
    Serial.println(rads);

    lcd.setCursor(0, 0);
    lcd.print("ANGLE : ");
    lcd.print(degrees);
    lcd.print(" degree");
    lcd.setCursor(0, 2);
    lcd.print("ANGULAR :");
    lcd.print(angular_speed);
    lcd.print(" rad/s");

    // delay(1000);
    // lcd.clear();
    // delay(1000);
    // int brightness;
    // brightness = map(degrees, 0, FULL_ANGLE, 0, 255);
    // analogWrite(LED,brightness);
    // delay(500);
}
