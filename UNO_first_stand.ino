//ms1   ms2    ms3
//L       L        L       整步(没有细分)
//H      L         L      1/2(2细分)
//L      H        L       1/4(4细分)
//H      H        L       1/8(8细分)
//H      H        H      1/16(16细分)
#include "PinChangeInt.h"
#include "OLED12864.h"
#include "AS5600_IIC.h"
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>

// 新增角度控制相关定义和变量
#define MICRO 16 // 微步分辨率
#define REDUCE 1 // 减速比
#define STEP_ANGLE 200 // 步进角度，单位是步数每转（steps per revolution）。
volatile double ANGLE = 0;
volatile unsigned long STEP = 0;
volatile unsigned long TARGET_STEP = 0;
volatile int flag_dir = 0; // 0=正转 1=反转
volatile bool angle_mode_active = false;
volatile uint8_t overflow_count = 0;   // 溢出周期计数器
volatile uint8_t cycles_per_step = 1; // 每个步进需要的溢出周期数

int mode = 1;
unsigned char tcnt2 = 178; // 定时器寄存器变量?
float deg_per_sec = 360; // 每秒度数 ：转速
OLED12864 oled12864;

int EN_PIN = 8;    //使能引脚
int DIR_PIN = 5;   //方向引脚
int STEP_PIN = 2;  //脉冲引脚

int ADD_PIN = A0; // 增加
int SUB_PIN = A1; // 减小
int SET_PIN = A2; // 设置

int speed_tmp = 1000;
String turn_tmp = "POS"; // 位置模式
String bin = "OFF"; // 状态

int power = 105; // 亮度
bool LED_switch = false;
#define LED 9

float angle = 0.0;           // 使用浮点保持精度
uint16_t last_raw_value = 0; // 存储上一次原始值
int calculate_tcnt2(float deg_per_sec);
void update_target() {
  TARGET_STEP = MICRO * REDUCE * STEP_ANGLE * ANGLE / 360;
}

// 修改后的按键处理函数
void set_key_deal(){
  static int flag_set = 0; // 用于方向控制的标志位
  switch(mode){

      //转换
    case 1:
        if(LED_switch == false)
      {
        LED_switch = true;
        analogWrite(LED, power);
        Serial.println("SET OPEN KEY");
        bin = "ON";
      }
      else
      {
        LED_switch = false;
        analogWrite(LED, 0);
        Serial.println("SET CLOSE KEY");
        bin = "OFF";
      }
      break;

      // 用按键控制旋转状态
      // flag_set只在case内部定义用来控制
      // 定义为static
    case 2:
      Serial.println("SET KEY");
        if(++flag_set >= 3) flag_set = 0;
        if(flag_set == 0)
      {  
        digitalWrite(EN_PIN, LOW);   
        digitalWrite(DIR_PIN, LOW);    
        Serial.println("正转");
        turn_tmp = "POS";
      }
        else if(flag_set == 1)
      {
        digitalWrite(EN_PIN, LOW);   
        digitalWrite(DIR_PIN, HIGH);    
        Serial.println("反转");  
        turn_tmp = "INV";
      }
        else if(flag_set == 2)
      {
        digitalWrite(EN_PIN, HIGH);   
        digitalWrite(DIR_PIN, HIGH); 
        turn_tmp = "STOP";   
        Serial.println("停转");
      }
      break;
      

      // 判断当前的STEP与设置值的大小关系
      // 然后设置正反转
      // 引入了flag_dir 表示正转反转状态
      // 每次判断完后开启定时器 溢出中断？
    case 3:
      digitalWrite(EN_PIN, LOW);
      update_target();
      if(STEP < TARGET_STEP){
        digitalWrite(DIR_PIN, LOW);
        TIMSK2 |= (1<<TOIE2);
        flag_dir = 0;
      }else if(STEP > TARGET_STEP){
        digitalWrite(DIR_PIN, HIGH);
        TIMSK2 |= (1<<TOIE2);
        flag_dir = 1;
      }else{
        TIMSK2 &= ~(1<<TOIE2);
      }
      break;
  }
}

// 用于处理“增加”按键的操作
void add_key_deal(){
  switch(mode){
    case 1:
      power += 25;
      power = min(power,255);
      if(LED_switch == true)  analogWrite(LED, power);
      if(power == 255 ) mode = 2;
      Serial.println("ADD KEY");
      break;
    case 2:
      deg_per_sec +=10;
      deg_per_sec = min(deg_per_sec,700);
      tcnt2 = calculate_tcnt2(deg_per_sec);
      overflow_count = 0; // 重置计数器
      if(deg_per_sec == 700 ) mode = 3;
      break;
    case 3:
      if(ANGLE < 360) ANGLE += 10;
      update_target();
      break;
  }
}

// 用于处理“减少”按键的操作
void sub_key_deal(){
  switch(mode){
    case 1:
    power -= 25;
      power = max(power,0);
      if(LED_switch == true)  analogWrite(LED, power); 
      Serial.println("SUB KEY");
      break;
    case 2:
      deg_per_sec -=10;
      deg_per_sec = max(deg_per_sec,10);
      tcnt2 = calculate_tcnt2(deg_per_sec);
      overflow_count = 0;
      break;
    case 3:
      if(ANGLE > 0) ANGLE -= 10;
      update_target();
      break;
  }
}

ISR(TIMER2_OVF_vect) {
  if (mode == 3) {
    TCNT2 = 120; // Mode3 固定速度
    digitalWrite(STEP_PIN, HIGH);
    _delay_us(10);
    digitalWrite(STEP_PIN, LOW);
 
    if (flag_dir == 0) {
      if (++STEP >= TARGET_STEP) {
        STEP = TARGET_STEP;
        TIMSK2 &= ~(1<<TOIE2);
      }
    } else if (flag_dir == 1) {
      if (--STEP <= TARGET_STEP) {
        STEP = TARGET_STEP;
        TIMSK2 &= ~(1<<TOIE2);
      }
    }
  } else {
    TCNT2 = tcnt2;
    if(++overflow_count >= cycles_per_step) {
      overflow_count = 0;
      digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
    }
  }
}

void AS5600_get() {
  // 读取当前原始值
  uint16_t current_raw = AS5600_IIC_Read_OneByte((0x36<<1), 0x0E) << 8;
  current_raw |= AS5600_IIC_Read_OneByte((0x36<<1), 0x0F);
  // 计算差值并处理溢出
  int16_t delta = current_raw - last_raw_value;
  if (delta > 2048) {       // 逆向溢出（例如 4095 -> 0）
    delta -= 4096;
  } else if (delta < -2048) { // 正向溢出（例如 0 -> 4095）
    delta += 4096;
  }
  // 更新角度（反向关系：RAW增加 → ANGLE减少）
  angle -= delta * 360.0 / 4096.0; // 注意负号
  // 角度归一化到 0~359.99
  if (angle >= 180) angle -= 360;
  else if (angle < -180) angle += 360;
  // 更新上一次原始值
  last_raw_value = current_raw;
}
//


//const int buttonpin = 2;
//int currentMode =0;
//unsigned long previousMillis = 0;
//const long debounceDelay = 50;

void setup() {
  sei();

//
  Serial.begin(115200);
  AS5600_IIC_Init();
  // 初始化时读取第一次原始值
  last_raw_value = AS5600_IIC_Read_OneByte((0x36<<1), 0x0E) << 8;
  last_raw_value |= AS5600_IIC_Read_OneByte((0x36<<1), 0x0F);
  
  pinMode(LED, OUTPUT);
  analogWrite(LED, 0);
  
  pinMode( EN_PIN,  OUTPUT ); 
  pinMode( DIR_PIN,  OUTPUT );
  pinMode( STEP_PIN,  OUTPUT );
  digitalWrite(EN_PIN, LOW);    //使能步进电机
  digitalWrite(DIR_PIN, LOW);   //方向初始化
  digitalWrite(STEP_PIN, LOW);
  
  pinMode( SET_PIN,  INPUT );
  pinMode( ADD_PIN,  INPUT );
  pinMode( SUB_PIN,  INPUT );

  attachPinChangeInterrupt( SET_PIN , set_key_deal, RISING );
  attachPinChangeInterrupt( ADD_PIN , add_key_deal, RISING );
  attachPinChangeInterrupt( SUB_PIN , sub_key_deal, RISING );

  oled12864.init();
  oled12864.clear();

  oled12864.show(0,0,"POWER:");
  oled12864.show(0,6,0);
  oled12864.show(1,0,"SWITCH:");
  oled12864.show(1,7,bin);
  oled12864.display(); 

  TIMSK2 &= ~(1<<TOIE2);
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));   //模式选择，正常模式
  TCCR2B &= ~(1<<WGM22);
  ASSR &= ~(1<<AS2);     //禁止异步中断触发
  TIMSK2 &= ~(1<<OCIE2A);     //中断允许标志位
  TCCR2B &= ~(1<<CS22);       // 清除CS22位
  TCCR2B |= (1<<CS21) | (1<<CS20); // 设置CS21和CS20
  TCNT2 = tcnt2;     //初值
  TIMSK2 |= (1<<TOIE2);  //溢出中断允许标志位

  STEP = 0;
  ANGLE = 0;
  update_target();

  Serial.println("Starting......");
}

void loop() {
//      int buttonState = digitalRead(buttonpin);
//      unsigned long currentMillis = millis();
//      if (buttonState == LOW){
//        if (currentMillis - previousMillis >= debounceDelay){
//          previousMillis = currentMillis;
//          currentMode = (currentMode == 0)?1:0;
//          if (currentMode ==0){
//            mode =1;
//            Serial.println("A");
//          }else{Serial.println("B");}
//          }
//        }
//        else{previousMillis = currentMillis;}
//      }
//      
//  
//    if (Serial.available() > 0) {
//  String command = Serial.readStringUntil('\n');
//    command.trim();
//    if (command.equalsIgnoreCase("mode1")) {
//     mode = 1;
//      Serial.println("Switched to LED Control Mode");
//   } else if (command.equalsIgnoreCase("mode2")) {
//      mode = 2;
//      Serial.println("Switched to Speed Control Mode");
//    } else if (command.equalsIgnoreCase("mode3")) {
//      mode = 3;
//      Serial.println("Switched to Angle Control Mode");
//    }
//  }
  static unsigned long lastRefreshTime = 0;
  if(mode == 3){
    AS5600_get(); 
  }
  if (millis() - lastRefreshTime >= 100) {
    lastRefreshTime = millis();
    
    switch(mode){
      case 1:
        oled12864.clear();
        oled12864.show(0,0,"POWER:");
        oled12864.show(0,6,power);
        oled12864.show(1,0,"SWITCH:");
        oled12864.show(1,7,bin);
        oled12864.display();
        break;
      case 2:
        oled12864.clear();
        oled12864.show(0,0,"SPEED:");
        oled12864.show(0,6,(int)deg_per_sec);
        oled12864.show(1,0,"TURN :");
        oled12864.show(1,6,turn_tmp);
        oled12864.show(2,0,"CYCLES:");
        oled12864.show(2,7,cycles_per_step);
        oled12864.display();
        break;
      case 3:
        oled12864.clear();
        oled12864.show(0,0,"ANGLE:");
        oled12864.show(0,6,(int)ANGLE);
        oled12864.show(1,0,"TARGET:");
        oled12864.show(1,7,(int)TARGET_STEP);
        oled12864.show(2,2,"CURRENT:");
        oled12864.show(3,0,(int)angle);
        oled12864.display();
        break;
    }
  }
  
  // 模式切换处理
  // 帅！！
    static int last_mode = 0;
    if(mode != last_mode){ 
    last_mode = mode;
    
    // 重置所有硬件状态
    digitalWrite(EN_PIN, HIGH);
    TIMSK2 &= ~(1<<TOIE2);
 
    // 初始化新模式
    switch(mode) {
      case 1:
        // 无特殊操作
        break;
      case 2:
        digitalWrite(EN_PIN, LOW);
        TIMSK2 |= (1<<TOIE2);
        break;
      case 3:
        digitalWrite(EN_PIN, LOW);
        STEP = 0;
        ANGLE = 0;
        update_target();
        break;
    }
  }
}
int calculate_tcnt2(float deg_per_sec) {
    // 每分钟转的圈数
    float rpm = deg_per_sec / 6.0;              // 正确：将角速度转换为RPM (60秒/10倍)   公式：RPM = (deg_per_sec * 60) / 360 = deg_per_sec / 6
    // 每秒转需要的脉冲数->脉冲频率
    float pulse_freq = rpm * (200.0 * 16) / 60.0; // 正确：计算脉冲频率（200步/转，16细分）
    // 根据当前分频系数计算中断频率  
    // 中断的频率还要乘上每个步进需要的中断数量
    float interrupt_freq = pulse_freq * cycles_per_step;
    
    // 计算TCNT2初值（64分频）
    // 256是ARR
    float tcnt2_val = 256.0 - (16000000.0 / (interrupt_freq * 64.0));
    tcnt2_val = constrain(tcnt2_val, 0.0, 255.0);
    
    // 动态调整分频系数（带递归）
    if(tcnt2_val < 10.0 && cycles_per_step < 256) {
        cycles_per_step *= 2;
        return calculate_tcnt2(deg_per_sec); // 递归计算新值
    } 
    else if(tcnt2_val > 200.0 && cycles_per_step > 1) {
        cycles_per_step /= 2;
        return calculate_tcnt2(deg_per_sec);
    }
    
    return (int)tcnt2_val;
}
