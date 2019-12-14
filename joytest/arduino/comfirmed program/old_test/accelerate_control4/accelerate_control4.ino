/* PULLDOWN REQUIRED
  acc control & brake control & brake lamp & e-stop
  D0,1,2,3,4,5,6,7,8,9,10,11,12,13,A0,1,2,3,4,5
   x,x,2,x,x,x,x,x,x,x, x, x, ?,13, x,1,2,3,4,5
  --
  Pin assignment
  D3  acc signal PWM
  D4  relay4 estop
  D5  relay3 no connection
  D6  relay2 acc signal
  D7  relay1 acc switch
  D8  brake lamp
  D9  front brake PWM
  D10 rear brake PWM
  D11 acc swtich PWM
  A0  speed measurement    Not implemented
  ---
  Wire assignment
  --E-stop-----
  D4 ----PULLDOWN Normal->LOW Estop->HIGH
  --acc control-------------------------------
  D7---------------|
             --------------
  --- BLACK -|NC relay1   |
  D11--BLUE--|NO       COM|--WHITE-- acc switch
             --------------  (255->STOP, 0->MOVE)

  D6--------------------------------|
                               --------------
  --- GREEN -------------------|NC relay2   |
  D3-|LOW |--|OPAMP |--YELLOW--|NO       COM|--RED-- acc signal
     |PASS|  |buffer|          --------------
     R=100k,C=0.33uF
  --front brake control-------------------------------
        -------------------
  D9----|In  H-Bridge  Out|--BLUE--brake signal
  12V---|12V              | (D9 PWM from 0 to 100)
  GND---|GND              |
        -------------------
  --rear brake control-------------------------------
  D10-------|Rear brake|---Linear actuator
                           myservo.writeMicroseconds (1950);//released
                           myservo.writeMicroseconds (1600);//pulled
  --brake lamp--------------------------------------
                   ---------------
  D8----YELLOW-----|Ctl          |
  G-----BLACK------|GND relay    |
  Vcc---RED--------|+5V       COM|------|Brake lamp|
  12V--------------|NO           |
  12V--|Pedal |----|NC           |
       |Switch|    ---------------
  -----Speed measurement ----
  A0---|LOWPASS|----|D3 Speed measurement arduino|   Not implemented
       R=100k,C=0.33uF
  TOPIC in-------
  acc-> Int16 from 0 to 255
  brake-> Int16 from 0 to 100
  TOPIC out---------
  c_acc<- Int16
  c_brake<- Int16
  c_blamp<- Bool
  c_estop<- Bool
*/
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
ros::NodeHandle nh;
std_msgs::Int16 c_acc; // for debugging
std_msgs::Int16 c_brake; // for debugging
std_msgs::Bool c_blamp; // for debugging
std_msgs::Bool c_estop; // for debugging
ros::Publisher pubacc("c_acc", &c_acc);
ros::Publisher pubbrake("c_brake", &c_brake);
ros::Publisher pubblamp("c_blamp", &c_blamp);
ros::Publisher pubestop("c_estop", &c_estop);

int acc_switch_relay = 7;//  pin07 PD7 on/off relay 1
int acc_signal_relay = 6;//  pin06 PD6 on/off relay 2
int acc_switch_PWM = 11;//  pin11 PD11 PWM 255->STOP,0->MOVE
int acc_signal_PWM = 3;//  pin03 PD3 PWM  SpeedControl
int brake_signal_PWM = 9;// pin09
int estop = 4; ////estop
int brake_lamp = 8;
int rear_brake = 10;

int acc = 0;
int brake = 0;
unsigned long duration_time = 0;

bool blamp = false;

unsigned long publishtime = 0;

bool estop_flag = false;
bool estop_flag_pre = false;
bool acc_topic = false;
bool brake_topic = false;


// acc_on(1-255) acc and relay is on
// acc_on(0) relay is off
// blamp = accbrake_off();acc = 0;brake = 0;duration_time = 0  completely off
// blamp = brake_on(1-100) front brake
// blamp = brake_on(255) rear brake
// blamp = brake_on(0) off
void accCb(const std_msgs::Int16 &Acc)
{
  acc = Acc.data;
  if (acc > 255) acc = 255; if (acc < 0) acc = 0;
  brake_topic = false;
  acc_topic = true;
  brake = 0;
}
void brakeCb(const std_msgs::Int16 &Brake)
{
  brake = Brake.data;
  if (brake > 100) brake = 100; else if (brake < 0) brake = 0;
  acc = 0;
  acc_topic = false;
  brake_topic = true;
}
// acc_on(1-255) acc and relay is on
// acc_on(0) relay is off
// blamp = brake_on(1-100) front brake
// blamp = brake_on(255) rear brake
// blamp = brake_on(0) off
void acc_on(int acc1) {
  if (acc1 == 0) {
    digitalWrite(acc_signal_relay, LOW); // trun on relay2
    digitalWrite(acc_switch_relay, LOW); // trun on relay1
    analogWrite(acc_switch_PWM, 255); // stop
    analogWrite(acc_signal_PWM, 0);
  } else {
    digitalWrite(acc_signal_relay, HIGH); // trun on relay2
    digitalWrite(acc_switch_relay, HIGH); // trun on relay1
    analogWrite(acc_switch_PWM, 0); // move
    acc1--;
    analogWrite(acc_signal_PWM, acc1);
  }
}
bool brake_on(int brake1) {
  if (brake1 == 255) {
    analogWrite(brake_signal_PWM, 0);
    //    myservo.writeMicroseconds (1600);//pulled
  } else {
    analogWrite(brake_signal_PWM, brake1);
    //    myservo.writeMicroseconds (1950);//released
    if (brake1 == 0) {
      digitalWrite(brake_lamp, false);
      //      duration_time = 0;
      return false;
    }
  }
  digitalWrite(brake_lamp, true);
  return true;
}
unsigned long c_millis;
bool estop_on = false;
ros::Subscriber<std_msgs::Int16> subacc("acc", &accCb);
ros::Subscriber<std_msgs::Int16> subbrake("brake", &brakeCb);
void setup()
{
  acc = 0;
  brake = 0;
  duration_time = 0;
  pinMode(estop, INPUT);
  pinMode(acc_switch_relay, OUTPUT); //  pin07 PD7 on/off relay 1
  pinMode(acc_signal_relay, OUTPUT); //  pin06 PD6 on/off relay 2
  pinMode(acc_switch_PWM, OUTPUT); //  pin11 PD11 PWM  HIGH->STOP,LOW->move
  pinMode(acc_signal_PWM, OUTPUT); //  pin03 PD3 PWM  SpeedControl
  analogWrite(acc_switch_PWM, 255); // stop
  analogWrite(acc_signal_PWM, 0); //
  pinMode(brake_signal_PWM, OUTPUT);
  pinMode(brake_lamp, OUTPUT);
  analogWrite(brake_signal_PWM, 0);
  digitalWrite(brake_lamp, LOW);
  //  myservo.attach(10);
  //  myservo.writeMicroseconds (1950);//released
  nh.initNode();
  nh.subscribe(subacc);
  nh.advertise(pubacc);
  nh.subscribe(subbrake);
  nh.advertise(pubbrake);
  nh.advertise(pubblamp);
  nh.advertise(pubestop);
  publishtime = millis() + 50;
  estop_on = digitalRead(estop);
  estop_flag_pre = estop_on;
  if (estop_on == true) {
    duration_time = c_millis + 1000;
    brake = 100;
  }
}
// acc_on(1-255) acc and relay is on
// acc_on(0) relay is off
// blamp = accbrake_off();acc = 0;brake = 0;duration_time = 0  completely off
// blamp = brake_on(1-100) front brake
// blamp = brake_on(255) rear brake
// blamp = brake_on(0) off
void loop() {
  c_millis = millis();
  estop_flag = digitalRead(estop);
  if (estop_flag_pre == false && estop_flag == true) {  // rising edge
    estop_on = true;
    duration_time = c_millis + 1000;
    brake = 100;
  }
  if (estop_flag_pre == true && estop_flag == false) { // falling edge
    estop_on = false;
    brake = 0; blamp = brake_on(brake);
    acc = 0; acc_on(acc);
    duration_time = 0;
  }
  if (estop_on == true) {
    brake_topic = false;
    acc_topic = false;
    brake = 100; blamp = brake_on(brake);
    acc = 0; acc_on(acc);
  }
  if (brake_topic == true) {
    duration_time = c_millis + 1000;
    acc = 0;
    brake_topic = false;
  }
  if (acc_topic == true) {
    duration_time = c_millis + 1000;
    brake = 0;
    acc_topic = false;
  }
  if (duration_time > 0) {
    if (brake > 0) {
      acc = 0; acc_on(acc);
      if (c_millis > duration_time) brake = 255;
      blamp = brake_on(brake);
    }
    if (acc > 0) {
      brake = 0; blamp = brake_on(brake);
      if (c_millis > duration_time) {
        acc = 1; duration_time = 0;
      }
      acc_on(acc);
    }
  }
  estop_flag_pre = estop_flag;
  if (c_millis > publishtime) {
    c_estop.data = estop_flag;
    pubestop.publish(&c_estop);
    c_brake.data = brake;
    pubbrake.publish(&c_brake);
    c_blamp.data = blamp;
    pubblamp.publish(&c_blamp);
    c_acc.data = acc;
    pubacc.publish(&c_acc);
    publishtime = c_millis + 50;
  }
  nh.spinOnce();
}
