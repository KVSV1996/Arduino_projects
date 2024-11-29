/*
  Скетч создан на основе FASTSPI2 EFFECTS EXAMPLES автора teldredge (www.funkboxing.com)
  А также вот этой статьи https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/#cylon
  Доработан, переведён и разбит на файлы 2017 AlexGyver
  Ещё раз переработан для работы с енкодером 2022.11 KVSV1996
  Виброшено всё лишнее (или многое, то что я понимал)
  Немного переписан под esp32 2023.12 KVSV1996
  
*/

#include "FastLED.h"          // библиотека для работы с лентой

      
#include "EncButton.h"        //библиотека для работы с енкодером

#define CLK 22  //пины енкодера
#define DT 21
#define SW 5

EncButton enc1(CLK, DT, SW);   //настройка енкодера

#define POTENT 15  // переменный резистор на яркость

#define TR 18  // пин транистора для отключения ленты

#define LED_COUNT 239          // число светодиодов в кольце/ленте
#define LED_DT 4             // пин, куда подключен DIN ленты

int max_bright = 255;          // максимальная яркость (0 - 255)
boolean adapt_light = 1;       // адаптивная подсветка (1 - включить, 0 - выключить)

byte fav_modes[] = {2, 3, 7, 11, 14, 23, 25, 27, 30};  // список "любимых" режимов
byte num_modes = sizeof(fav_modes);         // получить количество "любимых" режимов (они все по 1 байту..)
int new_bright;

//----------------Настройка RGB--------------------

int color, shift, _r = 255, _g = 255, _b = 255;
int val, potent_data, min_val_potent = 200, pot_sk = 4095;

volatile byte ledMode = 1;
/*
  Стартовый режим
  0 - все выключены
  1 - все включены
  3 - кольцевая радуга
  888 - демо-режим
*/

// цвета мячиков для режима
byte ballColors[3][3] = {
  {0xff, 0, 0},
  {0xff, 0xff, 0xff},
  {0   , 0   , 0xff}
};

// ---------------СЛУЖЕБНЫЕ ПЕРЕМЕННЫЕ-----------------
int BOTTOM_INDEX = 0;        // светодиод начала отсчёта
int TOP_INDEX = int(LED_COUNT / 2);
int EVENODD = LED_COUNT % 2;
struct CRGB leds[LED_COUNT];
int ledsX[LED_COUNT][3];     //-ARRAY FOR COPYING WHATS IN THE LED STRIP CURRENTLY (FOR CELL-AUTOMATA, MARCH, ETC)

int thisdelay = 20;          //-FX LOOPS DELAY VAR
int thisstep = 10;           //-FX LOOPS DELAY VAR
int thishue = 0;             //-FX LOOPS DELAY VAR
int thissat = 255;           //-FX LOOPS DELAY VAR

int thisindex = 0;
int thisRED = 0;
int thisGRN = 0;
int thisBLU = 0;

int idex = 0;                //-LED INDEX (0 to LED_COUNT-1
int ihue = 0;                //-HUE (0-255)
int ibright = 0;             //-BRIGHTNESS (0-255)
int isat = 0;                //-SATURATION (0-255)
int bouncedirection = 0;     //-SWITCH FOR COLOR BOUNCE (0-1)
float tcount = 0.0;          //-INC VAR FOR SIN LOOPS
int lcount = 0;              //-ANOTHER COUNTING VAR

volatile uint32_t btnTimer;
volatile byte modeCounter;
volatile boolean changeFlag;
// ---------------СЛУЖЕБНЫЕ ПЕРЕМЕННЫЕ-----------------


void setup()
{
   //enc1.setType(TYPE2);              // выбераем тип енкодера
   
  //Serial.begin(115200);              // открыть порт для связи
    
  LEDS.setBrightness(max_bright);  // ограничить максимальную яркость

  LEDS.addLeds<WS2811, LED_DT, GRB>(leds, LED_COUNT);  // настрйоки для нашей ленты (ленты на WS2811, WS2812, WS2812B)
  one_color_all(0, 0, 0);          // погасить все светодиоды
  LEDS.show();   

  pinMode(TR, OUTPUT);  
  digitalWrite(TR, 0);  
}

void one_color_all(int cred, int cgrn, int cblu) {       //-SET ALL LEDS TO ONE COLOR
  for (int i = 0 ; i < LED_COUNT; i++ ) {
    leds[i].setRGB( cred, cgrn, cblu);
  }
}


void iRGB(int color){
  if (color > 170){
    shift = (color - 170) *3;
    _r = shift;
    _g = 0;
    _b = 255- shift;
  }
  else if(color > 85){
    shift = (color - 85) *3;
    _r = 0;
    _g = 255- shift;
    _b = shift;
  }
  else{
    shift = color * 3;
    _r = 255- shift;
    _g = shift;
    _b = 0;
  }
}

void loop() {
  enc1.tick();

  if (adapt_light && !enc1.step()) {                         // если включена адаптивная яркость    
      potent_data = analogRead(POTENT);                    
      new_bright = map(potent_data, min_val_potent, pot_sk, 0, max_bright);   // считать показания с резистора, перевести диапазон
      new_bright = constrain(new_bright,0,max_bright);
      LEDS.setBrightness(new_bright);        // установить новую яркость  
      //Serial.println(new_bright);    
      if (potent_data < min_val_potent){                   //если значение потенциометра ниже порога отключить ленту. 
       digitalWrite(TR, 0);     
      }
      else{
        digitalWrite(TR, 1);
      }
   }    
 
 
 if (enc1.left()){
    //modeCounter--;
    if (modeCounter <= 0){
      modeCounter = num_modes-1;
    }
    else{
      modeCounter--;
    }
    ledMode = fav_modes[modeCounter];    
    changeFlag = true;  
    //Serial.println("left");
    //Serial.println(ledMode);  
    //Serial.println(modeCounter);
  }
  
  if (enc1.right()){
    //modeCounter++;
    if (modeCounter >= num_modes-1){
      modeCounter = 0;
    }
    else{
      modeCounter++;
    }
    ledMode = fav_modes[modeCounter];    
    changeFlag = true;   
    //Serial.println("right");
    //Serial.println(ledMode);
    //Serial.println(modeCounter); 
  }   
  
  if (enc1.click()){   
    ledMode = 1; 
    _r = 255;
    _g = 255;
    _b = 255;
    changeFlag = true;
    //Serial.println(ledMode);    
  }
  
  if(enc1.step()){
    val = analogRead(POTENT);  
    val = map(val,min_val_potent,pot_sk,0,255);
    color = constrain(val,0,255);
    //Serial.println("step");
    iRGB(color);
    }
    
  
  switch (ledMode) {
    //case 999: break;                           // пазуа
    case  1: one_color_all(_r, _g, _b); LEDS.setBrightness(new_bright); LEDS.show(); break; //---ALL ON
    case  2: rainbow_fade(); break;            // плавная смена цветов всей ленты
    case  3: rainbow_loop(); break;            // крутящаяся радуга
    case  4: random_burst(); break;            // случайная смена цветов
    case  5: color_bounce(); break;            // бегающий светодиод
    case  6: color_bounceFADE(); break;        // бегающий паровозик светодиодов
    case  7: ems_lightsONE(); break;           // вращаются красный и синий
    case  8: ems_lightsALL(); break;           // вращается половина красных и половина синих
    case  9: flicker(); break;                 // случайный стробоскоп
    case 10: pulse_one_color_all(); break;     // пульсация одним цветом
    case 11: pulse_one_color_all_rev(); break; // пульсация со сменой цветов
    case 12: fade_vertical(); break;           // плавная смена яркости по вертикали (для кольца)
    case 13: rule30(); break;                  // безумие красных светодиодов
    case 14: random_march(); break;            // безумие случайных цветов
    case 15: rwb_march(); break;               // белый синий красный бегут по кругу (ПАТРИОТИЗМ!)
    case 16: radiation(); break;               // пульсирует значок радиации
    case 17: color_loop_vardelay(); break;     // красный светодиод бегает по кругу
    case 18: white_temps(); break;             // бело синий градиент (?)
    case 19: sin_bright_wave(); break;         // тоже хрень какая то
    case 20: pop_horizontal(); break;          // красные вспышки спускаются вниз
    case 21: quad_bright_curve(); break;       // полумесяц
    case 22: flame(); break;                   // эффект пламени
    case 23: rainbow_vertical(); break;        // радуга в вертикаьной плоскости (кольцо)
    case 24: pacman(); break;                  // пакман
    case 25: random_color_pop(); break;        // безумие случайных вспышек
    case 26: ems_lightsSTROBE(); break;        // полицейская мигалка
    case 27: rgb_propeller(); break;           // RGB пропеллер
    case 28: kitt(); break;                    // случайные вспышки красного в вертикаьной плоскости
    case 29: matrix(); break;                  // зелёненькие бегают по кругу случайно
    case 30: new_rainbow_loop(); break;        // крутая плавная вращающаяся радуга
    case 31: strip_march_ccw(); break;         // чёт сломалось
    case 32: strip_march_cw(); break;          // чёт сломалось
    case 33: colorWipe(0x00, 0xff, 0x00, thisdelay);
      colorWipe(0x00, 0x00, 0x00, thisdelay); break;                                // плавное заполнение цветом
    case 34: CylonBounce(0xff, 0, 0, 4, 10, thisdelay); break;                      // бегающие светодиоды
    case 35: Fire(55, 120, thisdelay); break;                                       // линейный огонь
    case 36: NewKITT(0xff, 0, 0, 8, 10, thisdelay); break;                          // беготня секторов круга (не работает)
    case 37: rainbowCycle(thisdelay); break;                                        // очень плавная вращающаяся радуга
    case 38: TwinkleRandom(20, thisdelay, 1); break;                                // случайные разноцветные включения (1 - танцуют все, 0 - случайный 1 диод)
    case 39: RunningLights(0xff, 0xff, 0x00, thisdelay); break;                     // бегущие огни
    case 40: Sparkle(0xff, 0xff, 0xff, thisdelay); break;                           // случайные вспышки белого цвета
    case 41: SnowSparkle(0x10, 0x10, 0x10, thisdelay, random(100, 1000)); break;    // случайные вспышки белого цвета на белом фоне
    case 42: theaterChase(0xff, 0, 0, thisdelay); break;                            // бегущие каждые 3 (ЧИСЛО СВЕТОДИОДОВ ДОЛЖНО БЫТЬ КРАТНО 3)
    case 43: theaterChaseRainbow(thisdelay); break;                                 // бегущие каждые 3 радуга (ЧИСЛО СВЕТОДИОДОВ ДОЛЖНО БЫТЬ КРАТНО 3)
    case 44: Strobe(0xff, 0xff, 0xff, 10, thisdelay, 1000); break;                  // стробоскоп

    case 45: BouncingBalls(0xff, 0, 0, 3); break;                                   // прыгающие мячики
    case 46: BouncingColoredBalls(3, ballColors); break;                            // прыгающие мячики цветные
    //case 47: one_color_all(255, 0, 0); LEDS.setBrightness(new_bright); LEDS.show(); break; //---ALL ON

    //case 888: demo_modeA(); break;             // длинное демо
    //case 889: demo_modeB(); break;             // короткое демо
  }
  
}
