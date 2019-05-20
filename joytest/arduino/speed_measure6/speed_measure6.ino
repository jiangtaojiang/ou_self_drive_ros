/*Need convert from Hz to mph
  speed measurement
  D0,1,2,3,4,5,6,7,8,9,10,11,12,13,A0,1,2,3,4,5
   x,x,2,3,4,5,6,7,8,9,10,11, x,13,A0,1,2,3,4,5
  --
  Pin assignment
  D12 speed in
  D3  speed out for acc & brake
  ---
  Wire assignment
  --speed measurement-------------------------------
                      --------
  5V------------------|4     |
  5V----|14        |  |LM324 |
  D12---|2 74LS14 1|--|1,2  3|---C--------SPEED SENSOR
  GND---|7         |  |      |   |0.1uF
  GND-----------------|11    |  GND
                      --------
  --------------------------------------------------
  -----Speed measurement ----
  D3---|LOWPASS|----|A0 acc & brake arduino|
       R=100k,C=0.33uF
  ---------------------------
TOPIC in-------
  nothing
  TOPIC out---------
  c_speed<- Float32
*/
#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
//int SPDcount = 12;
int SPDcount = A0;
std_msgs::Float32 c_speed;
ros::Publisher pub("c_speed", &c_speed);
float speed = 0;
float out = 0.0;
int c_val = 0;
int p_val = 0;
void setup()
{
  nh.initNode();
  nh.advertise(pub);
  ADCSRA = ADCSRA & 0xf8;
  ADCSRA = ADCSRA | 0x04;
  out = 5037;
}
unsigned long c_millis = 0;
unsigned long count = 0;
unsigned long acount = 0;
void loop() {
  c_millis = millis() + 100;
  p_val   = analogRead(SPDcount);
  count = 0;
  acount = 0;//5037
  while(1) {
    c_val = analogRead(SPDcount);
    if(c_val - p_val > 200) count++;
    if(p_val - c_val > 200) count++;
    p_val = c_val;
    acount++;
    if(c_millis < millis()) break;
  }
  float val =  float(acount) / float(count + 1.0);
  out = 0.9 * out + 0.1 * (float) val;
  c_speed.data = out;//spd[count];
  pub.publish(&c_speed);
  nh.spinOnce();
}
