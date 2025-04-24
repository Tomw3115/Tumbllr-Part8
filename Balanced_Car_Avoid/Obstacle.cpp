#include "Obstacle.h"
#include <Adafruit_NeoPixel.h>


IR IR;
Ultrasonic Ultrasonic;

// define lights object type
Adafruit_NeoPixel lights = Adafruit_NeoPixel(4, 3, NEO_GRB + NEO_KHZ800);

extern Balanced Balanced;
extern Mpu6050 Mpu6050;

void Ultrasonic::Pin_init()
{
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  //initilaize lights
  lights.begin();
  lights.clear();
  lights.show();

}

void Ultrasonic::Lights(bool obstacle)
  {
    if (obstacle)
    {
        lights.setPixelColor(0, 100, 0, 0);
        lights.setPixelColor(1, 100, 0, 0);
        lights.show();
        delay(10);
    }
    else
    {
        lights.setPixelColor(0, 0, 100, 0);
        lights.setPixelColor(1, 0, 100, 0);
        lights.show();
        delay(10);
    }

  }

  void IR::Lights()
  {
    int ir_sense = left_is_obstacle + right_is_obstacle;
    Serial.println(ir_sense);
    // lights.clear();
    // lights.show();
    switch (ir_sense)   
    {
      case 0:
        {
          lights.setPixelColor(2, 0, 0, 0);
          lights.setPixelColor(3, 0, 0, 0);
          lights.show();
          delay(10);
          break;
        }
      case 1:
        { 
          lights.setPixelColor(2, 100, 0, 0);
          lights.setPixelColor(3, 0, 0, 0);
          lights.show();
          delay(10);
          break;
        }
      case 2:
        {
          lights.setPixelColor(2, 0, 0, 0);
          lights.setPixelColor(3, 100, 0, 0);
          lights.show();
          delay(10);
          break;
        }
      case 3:
        {
          lights.setPixelColor(2, 100, 0, 0);
          lights.setPixelColor(3, 100, 0, 0);
          lights.show();
          delay(10);
          break;
        }
    }
  }

char Ultrasonic ::measure_flag = 0;
unsigned long Ultrasonic ::measure_prev_time = 0;
double Ultrasonic ::distance_value;

void Function::Obstacle_Mode()
{
  IR.Send();
  Ultrasonic.Get_Distance();
  Ultrasonic.Lights(OBSTACLE_JUDAGEMENT);
  IR.Lights();


  if (millis() - follow_prev_time >= 100)
  {
    follow_prev_time = millis();
    Balanced.Motion_Control(FORWARD);
    IR.Lights();
    while (OBSTACLE_JUDAGEMENT)
    {
      Balanced.Motion_Control(STOP);

      if (millis() - Obstacle_time > 5000)
      { 
        Obstacle_time = millis();
        Back_time = millis();
        while (millis() - Back_time < 500)
        {
          Balanced.Motion_Control(BACK);
        }
      }

      Turning_Time = millis();
      while (millis() - Turning_Time < 750)
      {
        if (turn_flag)
        { turn_flag = 0;
          IR.Check();
        }
      }
      turn_flag = 1;


      Ultrasonic.Get_Distance();
    }

  }
    Obstacle_time = millis();
}

void IR::Check()
{
  Lights();
  int motion = left_is_obstacle + right_is_obstacle;
  switch (motion)
  {
    case Meet_OBSTACLE: Balanced.Motion_Control(LEFT);
      left_is_obstacle = 0; break;

    case FOLLOW_RIGHT:
      Balanced.Motion_Control(RIGHT);
      left_is_obstacle = 0; break;

    case FOLLOW_LEFT:
      Balanced.Motion_Control(LEFT);
      right_is_obstacle = 0; break;

    case FOLLOW_BACK:
      Balanced.Motion_Control(LEFT);
      right_is_obstacle = 0;
      left_is_obstacle = 0; break;
  }

}

void IR::Send()
{
  static unsigned long ir_send_time;

  if (millis() - ir_send_time > 15)
  {
    for (int i = 0; i < 39; i++)
    {
      digitalWrite(IR_SEND_PIN, LOW);
      delayMicroseconds(9);
      digitalWrite(IR_SEND_PIN, HIGH);
      delayMicroseconds(9);
    }
    ir_send_time = millis();
  }

}

