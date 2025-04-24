// Contents of file sensor.h

#ifndef sensor_h
#define sensor_h
// code goes here

class IR
{
  public:
    void Pin_init();
    void Check();
    void Sense();
    void Lights(int ir_sense);
  public:
    static int IR::left_is_obstacle;
    static int IR::right_is_obstacle;
    static int IR::ir_sense;
  private:
    #define RECV_PIN 9
    #define IR_SEND_PIN 9
    #define LEFT_RECEIVE_PIN A0
    #define RIGHT_RECEIVE_PIN A1
};

class Ultrasonic
  {
  public:
    void Pin_init();
    void Get_Distance();
    float Check();
    void Lights(float dist);
    static void Distance_Measure();
  public:
    static char measure_flag;
    static unsigned long measure_prev_time;
    unsigned long get_distance_prev_time;
    static float distance_value;
    unsigned long ir_send_time;
  private:
    #define ECHO_PIN 17
    #define TRIG_PIN 11
};
#endif