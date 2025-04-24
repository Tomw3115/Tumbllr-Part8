#include "Motor.h"
#include "Balanced.h"
#include <Adafruit_NeoPixel.h>
#include "sensor.h"

Timer2 Timer2;
extern Mpu6050 Mpu6050;
extern Motor Motor;
extern Balanced Balanced;

IR IR;
Ultrasonic Ultrasonic;

// initialize variables
char Ultrasonic::measure_flag=0;
unsigned long Ultrasonic::measure_prev_time=0;
static float Ultrasonic::distance_value;
unsigned long previous_time = 0;
float stop_dist;
float avoid_dist;

void check_obstacle()
{
  //IR.Sense();
  Ultrasonic.Get_Distance();
  //delay(50);
  //IR.Check();
  //IR.Lights(IR::ir_sense);
  stop_dist = Ultrasonic.Check() - avoid_dist;
  Serial.println(stop_dist);
  //Ultrasonic.Lights(stop_dist);

}

void setup() 
{
  Motor.Pin_init();
  Motor.Encoder_init();
  Timer2.init(TIMER);
  Mpu6050.init();
  Balanced.initLED();
  Serial.begin(9600);
  //IR.Pin_init();
  Ultrasonic.Pin_init();
  Balanced.Stop(); 
  delay(5000); // pause to add payload to Tumbllr after balance is acheived
  check_obstacle();
  // path to calibrate WHEEL_DIA
  //Balanced.ForwardDist(20);
  // if distance moved is less than 20 inches, decrease WHEEL_DIA
  // if distance moved is greater than 20 inches, increase WHEEL_DIA

  // path to calibrate TRACK_WIDTH
  //Balanced.LeftAngle(360);
  // if turn is less than 360 degrees, make TRACK_WIDTH larger
  // if turn is more than 360 degrees, make TRACK_WIDTH smaller

  // add variable to allow distance measurements in floor tiles
  float tile = 17.0; //floor tile length
  
  Balanced.ForwardAvoid(6);
  //check_obstacle();
  Balanced.LeftAngle(180);
  //check_obstacle();
  Balanced.ForwardAvoid(6);
  //check_obstacle();
  // Balanced.RightAngle(90);
  // Balanced.ForwardDist(6*tile);
  // Balanced.RightAngle(90);
  // Balanced.ForwardDist(9*tile);
  // Balanced.LeftAngle(90);
  // Balanced.ForwardDist(3*tile);
  Balanced.Stop();

}
void loop() 
{
  
}

  

