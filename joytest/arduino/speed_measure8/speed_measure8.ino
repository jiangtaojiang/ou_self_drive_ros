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

close all
t= 0:1/5000:5;                   % 10 seconds @ 5kHz sample rate
fo=10;f1=300;                    % Start at 10Hz, go up to 400Hz
y=(chirp(t,fo,2,f1,'logarithmic')>0)*1024;
figure;plot(t,y);
y=y+100*rand(size(y));
figure
spectrogram(y,256,200,256,1000);
figure
out = [];
i = 1;
while(1)
  c_micros = t(i)*1000 + 100;
  p_val = y(i);
  i = i + 1;
  count = 0;
  acount = 0;
  while(1)
    c_val = y(i);
    if(c_val - p_val > 200) count = count +1;end
    if(p_val - c_val > 200) count = count +1;end
    p_val = c_val;
    acount = acount + 1;
    if(c_micros < t(i)*1000) break;end
    i = i + 1;
    if(i >= length(t)) break;end
  end
  out = [out, (count+1)/acount];
  i = i + 1;
  if(i >= length(t)) break;end
end
plot(out)



*/
#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
int SPDcount = A0;
std_msgs::Float32 c_speed;
ros::Publisher pub("c_speed", &c_speed);
int i = 0;
int value[3];
void setup()
{
  nh.initNode();
  nh.advertise(pub);
  ADCSRA = ADCSRA & 0xf8;
  ADCSRA = ADCSRA | 0x04;
  value[0] = analogRead(SPDcount);
  value[1] = value[0];
  value[2] = value[1];
  i = 0;
}
int c_val;
int p_val;
unsigned long c_micros;
unsigned long n_micros;
unsigned long c_time;
unsigned long c_micros1[3];
unsigned long ud_duration[2];
unsigned int count;
unsigned int udcount;
int ud;
float duration;
void loop() {
  p_val = 0;
  count = 0;
  udcount = 0;
  ud = 0;
  duration = 0.0;
  n_micros = micros() + 50000;
  while(1) {
    c_micros = micros();
    value[i] = analogRead(SPDcount);
    c_micros1[i] = c_micros;
    i++;
    if(i > 2) i = 0;
    c_time = (c_micros1[0] + c_micros1[1] + c_micros1[2]) / 3.0;
    if((value[0] >= value[1]) && (value[1] > value[2])) {c_val = value[1];}
    if((value[0] >= value[2]) && (value[2] > value[1])) {c_val = value[2];}
    if((value[1] >= value[0]) && (value[0] > value[2])) {c_val = value[0];}
    if((value[1] >= value[2]) && (value[2] > value[0])) {c_val = value[2];}
    if((value[2] >= value[0]) && (value[0] > value[1])) {c_val = value[0];}
    if((value[2] >= value[1]) && (value[1] > value[0])) {c_val = value[1];}
    if(c_val - p_val > 500) {ud_duration[ud++] = c_time;count++;}
    if(p_val - c_val > 500) {ud_duration[ud++] = c_time;count++;}
    if(ud > 1) {
      duration += (float)(ud_duration[1] - ud_duration[0]);
      ud_duration[0] = ud_duration[1];
      ud = 1;
      udcount++;
    }
    p_val = c_val;
    if(n_micros < c_micros) break;
  }
  float total = (float)(c_micros - n_micros + 50000);
  float fcount = (float)count;
  if(udcount > 0) fcount = (float)udcount +  (total - duration) / (duration / (float)udcount);
  c_speed.data = fcount;
  pub.publish(&c_speed);
  nh.spinOnce();
}
