
// *************************** НАСТРОЙКИ ***************************
#define amount 8160 // количество шагов

// ************ ПИНЫ ************ 
#define STEP 6 // пин STEP на драйвере, шаги 
#define DIR 5 // пин DIR на драйвере, направление
#define EN 12 // пин EN на драйвере, enable драйвера
#define EN_2 11 // пин EN на драйвере, enable драйвера(второй так как нужно больше тока)
#define POTENT A7 //потенциометр управления жалюзами
#define POTENT_2 A6 // потенциометр управления яркостю светодиодной ленты
#define BUT 8 // кнопка управления настольными лампами
#define LED 9 // светодиодная лента
#define RELAY 7 // реле
#include <avr/eeprom.h>
// ***************** ПЕРЕМЕННЫЕ *****************
boolean flag, flag_but, led_flag, butt;
int val, steps, pwm;
unsigned long last_press;

void setup() {
  pinMode(STEP, OUTPUT); 
  pinMode(DIR, OUTPUT); 
  pinMode(EN, OUTPUT); 
  pinMode(BUT, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(RELAY, OUTPUT);  
  
  steps = eeprom_read_word(10); 
  digitalWrite(RELAY,1); 
  Serial.begin(9600);
}

 void yield(){
  
  val = analogRead(POTENT);  
  val = map(val,0,1023,0,3);
  val = constrain(val,0,2);
  
  //Serial.println(val);
  
    if(val==0){
    flag=0;
  }
  
  if(val==2){
    flag=1;
  }

  butt = !digitalRead(BUT);
 
  if(butt==1&&flag_but==0&&millis()-last_press>400){
    //Serial.println(1);
    flag_but=1;
    led_flag=!led_flag;
    digitalWrite(RELAY,led_flag);
    last_press = millis(); 
  }
  if(butt==0&&flag_but==1){
    //Serial.println(1);
    flag_but=0;
  }
 
  pwm = analogRead(POTENT_2);  
  pwm = map(pwm,15,1023,0,255);
  pwm = constrain(pwm,0,255);
  analogWrite(LED, pwm);  
  
 }

void loop() {

  //Serial.println(steps);
  
  if(steps==0||steps==amount||val==1){
     eeprom_update_word(10,steps);
  }

  
  if(steps==0||val==1||steps==amount){
  digitalWrite(EN,1);
  digitalWrite(EN_2,1);  
  }
  
  digitalWrite(STEP,0);
  delay(1);
  
  if(val==0&&steps<amount&&flag==0){
    digitalWrite(EN,0);
    digitalWrite(EN_2,0);
    steps++;
    digitalWrite(DIR,flag);
   digitalWrite(STEP,1);
   delay(1);
  }
     
   if(val==2&&steps>0&&flag==1){
    digitalWrite(EN,0);
    digitalWrite(EN_2,0);
    steps--;
    digitalWrite(DIR,flag);
   digitalWrite(STEP,1);
   delay(1);
   } 
         
}
