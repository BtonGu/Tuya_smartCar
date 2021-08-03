#include <TuyaWifi.h>
#include <SoftwareSerial.h>
#include <Servo.h>

TuyaWifi my_device;
Servo servo1;
Servo servo2;
//SoftwareSerial DebugSerial(8,9);


/* Current LED status */
unsigned char led_state = 0;
long dp_value_value;
void Motor(unsigned char sp1,unsigned char sp2);
void Re(void);
/* Connect network button pin */
int key_pin = 7;
int ledpin  = A5;
int E1 = 10;  
int M1 =8;
int E2 =11;                        
int M2 = 13;    
int Buzz =4;

/* Data point define */
#define DPID_BOOL  101
#define DPID_Servo1 102
#define DPID_Servo2 103
#define DPID_Speed 104
#define DPID_LR    105
#define DPID_led   106
#define DPID_Re 107
#define DPID_Auto 108
/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type. 
 *                                     dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
*/
unsigned char dp_array[][2] =
{
  {101, DP_TYPE_BOOL},
  {102, DP_TYPE_VALUE},
  {103, DP_TYPE_VALUE},
  {104, DP_TYPE_VALUE},
  {105, DP_TYPE_VALUE},
  {106, DP_TYPE_BOOL},
  {107, DP_TYPE_BOOL},
  {108, DP_TYPE_BOOL},
};

unsigned char pid[] = {"efv07xavzji9ir6j"};
unsigned char mcu_ver[] = {"1.0.0"};

/* last time */
unsigned long last_time = 0;
unsigned char angle1 = 90;
unsigned char angle2 = 20;
unsigned char SPEED = 0;
 char LR = 0;
unsigned char car_led = 0;
unsigned char re_status = 0;

void setup() 
{
  Serial.begin(9600);//  DebugSerial.begin(9600);
  servo1.attach(9);
  servo2.attach(A1);
  servo1.write(angle1);
  servo2.write(angle2);

  //Initialize led port, turn off led.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(ledpin, OUTPUT);
  digitalWrite(ledpin, HIGH);
 

  //Initialize networking keys.
  pinMode(key_pin, INPUT_PULLUP);
  pinMode(M1,OUTPUT);  
  pinMode(M2,OUTPUT);
  digitalWrite(12, 0);
  for(last_time=0;last_time<2;last_time++){
    analogWrite(Buzz,80);   
    delay(1000);
    analogWrite(Buzz,0);   
    delay(1000);
  }
  last_time=0;


  //Enter the PID and MCU software version
  my_device.init(pid, mcu_ver);
  //incoming all DPs and their types array, DP numbers
  my_device.set_dp_cmd_total(dp_array, 8);
  //register DP download processing callback function
  my_device.dp_process_func_register(dp_process);
  //register upload all DP callback function
  my_device.dp_update_all_func_register(dp_update_all);

  last_time = millis();
}

void loop() 
{
  my_device.uart_service();

  //Enter the connection network mode when Pin7 is pressed.
  if (digitalRead(key_pin) == LOW) {
    delay(80);
    if (digitalRead(key_pin) == LOW) {
      my_device.mcu_set_wifi_mode(SMART_CONFIG);
    }
  }
  /* LED blinks when network is being connected */
  if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW)) {
    if (millis()- last_time >= 500) {
      last_time = millis();

      if (led_state == LOW) {
        led_state = HIGH;
      } else {
        led_state = LOW;
      }
      digitalWrite(LED_BUILTIN, led_state);
    }
  }

  delay(10);
}

/**
 * @description: DP download callback function.
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length)
{
  unsigned char i;
  switch(dpid) {
    case 101:
      led_state = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
      if (led_state) {
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        digitalWrite(LED_BUILTIN, LOW);
        Motor(0,0);
        re_status=0;
        LR=0;
      }
      //Serial.print(led_state);
      dp_update_all();
    break;
    
    case DPID_Servo1:
      angle1 = my_device.mcu_get_dp_download_data(dpid, value, length);
      servo1.write(angle1);
      my_device.mcu_dp_update(dpid, angle1, 1);
    break;
    
    case DPID_Servo2:
      angle2 = my_device.mcu_get_dp_download_data(dpid, value, length);
      if(angle2<20)angle2=20;
      servo2.write(angle2);
      my_device.mcu_dp_update(dpid, angle2, 1);
    break;
    
    case DPID_Speed:
      SPEED = my_device.mcu_get_dp_download_data(dpid, value, length);
      if(SPEED<20)SPEED=0;
      if(led_state==1){Motor(SPEED,SPEED);}
      else SPEED=0;
      my_device.mcu_dp_update(dpid, SPEED, 1);
    break;
    
    case DPID_LR:
      LR = my_device.mcu_get_dp_download_data(dpid, value, length);
      switch(LR){
        case -2:  Motor(SPEED,SPEED+SPEED/2);      break;
        case -1:  Motor(SPEED/3 * 2,SPEED);  break;
        case 0:  Motor(SPEED,SPEED);         break;
        case 1:  Motor(SPEED,2* SPEED/3 );   break;
        case 2:  Motor(SPEED/2+SPEED,SPEED);       break;
      }
      my_device.mcu_dp_update(dpid, LR, 1);
    break;
    
    case DPID_led:
      car_led = my_device.mcu_get_dp_download_data(dpid, value, length);
      if(car_led)        digitalWrite(ledpin, LOW);
      else               digitalWrite(ledpin, HIGH);
      my_device.mcu_dp_update(dpid, car_led, 1);
    break;
    
    case DPID_Re:
      re_status = my_device.mcu_get_dp_download_data(dpid, value, length);
      if(re_status) Re();
      else Motor(0,0);
      my_device.mcu_dp_update(dpid, re_status, 1);
    break;
    
    case DPID_Auto:
      my_device.mcu_dp_update(dpid, value, 1);
    break;

    default:break;
  }
  return SUCCESS;
}

/**
 * @description: Upload all DP status of the current device.
 * @param {*}
 * @return {*}
 */
void dp_update_all(void)
{
  my_device.mcu_dp_update(101, led_state, 1);
  my_device.mcu_dp_update(102, angle1, 1);
  my_device.mcu_dp_update(103, angle2, 1);
  my_device.mcu_dp_update(104, SPEED, 1);
  my_device.mcu_dp_update(105, LR, 1);
  my_device.mcu_dp_update(106, car_led, 1);
  my_device.mcu_dp_update(107, re_status, 1);
}


/********************************CAR************************************/
void Motor(unsigned char sp1,unsigned char sp2){
    sp1=(unsigned char)(0.8*sp1);
    if(sp1>200){sp1=200;}
    if(sp2>200){sp2=200;}

    digitalWrite(M2, LOW);  
    digitalWrite(M1, LOW);       
    analogWrite(E1, sp1);   //PWM调速
    analogWrite(E2, sp2);   //PWM调速
}
void Re(void){
    digitalWrite(M1,HIGH);  
    digitalWrite(M2,HIGH);      
    analogWrite(E1,80);   //PWM调速
    analogWrite(E2,96);   //PWM调速
    
}

