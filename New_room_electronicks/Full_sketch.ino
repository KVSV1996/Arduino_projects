#include <WiFi.h>
#include <WebServer.h>
#include <FastLED.h>
#include <Arduino.h>
#include <Preferences.h>

// Параметры светодиодной ленты
#define LED_PIN     25          // Пин данных
#define NUM_LEDS    300          // Количество светодиодов
#define LED_TYPE    WS2815      // Тип светодиодов
#define COLOR_ORDER GRB         // Порядок цветов

CRGB leds[NUM_LEDS];

// Пины управления
#define POT_PIN          34    // Пин потенциометра
#define TRANSISTOR_PIN   33   // Пин транзистора
#define CLK_PIN          19   // Пин CLK энкодера
#define DT_PIN           18   // Пин DT энкодера
#define SW_PIN           5    // Пин кнопки энкодера

// Пины шаговика и света
#define ENABLE_PIN 26       // Управление ENABLE драйвера
#define STEP_PIN 27         // Управление STEP драйвера
#define DIR_PIN 14          // Управление DIR драйвера
#define POT_PIN_2 35          // Аналоговый вход потенциометра
#define BUTTON_PIN 36       // Пин кнопки
#define TOGGLE_PIN 32       // Пин, который будет переключаться

// Порог для выключения ленты (настройте по необходимости)
#define THRESHOLD        200  // Значение ADC ниже которого лента выключается

//************************ ШАГОВИК **************************
// Константы
const long MAX_POSITION = 8000;    // Максимальная позиция двигателя
const int POT_MAX = 4095;          // Максимальное значение потенциометра (12 бит)
const unsigned long STEP_INTERVAL_US = 300; // Интервал между шагами в микросекундах
const unsigned long DEBOUNCE_DELAY = 30;     // Задержка антидребезга для кнопки в миллисекундах

// Переменные состояния
long currentPosition = 0;      // Текущая позиция двигателя
bool direction = true;         // true - вперед, false - назад
bool motorEnabled = false;     // Флаг состояния двигателя
unsigned long lastStepTime = 0; // Время последнего шага в микросекундах

// Переменные для кнопки
bool lastButtonLightState = HIGH;       // Предыдущее состояние кнопки (для INPUT_PULLUP)
bool buttonLightState = HIGH;           // Текущее состояние кнопки
bool toggleState = LOW;            // Состояние пина TOGGLE_PIN
unsigned long lastDebounceTime = 0; // Время последнего изменения состояния кнопки

// EEPROM Preferences
Preferences preferences;
const char* PREF_NAMESPACE = "motor";

//**************************** ЛЕНТА ****************************

// Переменные для эффектов
uint8_t currentEffect = 2;
const uint8_t numEffects = 4; // Количество доступных эффектов

// Переменные для энкодера
int currentStateCLK;
int lastStateCLK;
int currentDir = 0; // Переменная для хранения направления вращения
unsigned long lastEncoderTurnTime = 0;

// Переменные для кнопки энкодера
unsigned long lastButtonPress = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;

// Для антидребезга кнопки
unsigned long debounceDelay = 50; // Задержка для антидребезга кнопки (мс)
unsigned long buttonDebounceTime = 0;

// Переменные для обработки нажатия кнопки
unsigned long buttonPressTime = 0;
bool isButtonPressed = false;
const unsigned long longPressTime = 1000; // Время для определения длинного нажатия (мс)
bool isColorChangeMode = false; // Флаг режима изменения цвета

// Переменная для хранения пользовательского цвета
uint8_t userSelectedHue = 0; // Оттенок, выбранный пользователем

// Переменные для эффектов
// Эффект дыхания с плавной сменой цвета
unsigned long lastBreatheUpdate = 0;
const unsigned long breatheInterval = 50; // Интервал обновления дыхания (мс)
uint8_t breatheBrightness = 0;
int8_t breatheStep = 1;
uint8_t breatheHue = 0;

// Эффект радуги
uint8_t hue = 0;
unsigned long lastRainbowUpdate = 0;
unsigned long rainbowInterval = 50; // Интервал обновления радуги (мс)

// Переменная для хранения максимальной яркости
uint8_t maxBrightness = 255;

// Параметры Wi-Fi
const char* ssid = "Crystal V3";         // Замените на ваш SSID
const char* password = "1020304050"; // Замените на ваш пароль

//*********************** Методы шаговика **************************
// Функции для управления драйвером
void enableMotor(bool enable) {
  digitalWrite(ENABLE_PIN, enable ? HIGH : LOW); // НЕ инвертированное управление
  motorEnabled = enable;
}

void setDirection(bool dir) {
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
  direction = dir;
}

void stepMotor() {
  // Генерация шага
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(10); // Минимальная задержка для сигнала HIGH
  digitalWrite(STEP_PIN, LOW);

  // Обновление позиции
  if (direction) {
    currentPosition++;
  } else {
    currentPosition--;
  }

  // Проверка достижения крайних положений
  if (currentPosition >= MAX_POSITION) {
    currentPosition = MAX_POSITION;
    enableMotor(false); // Отключить двигатель
    preferences.putLong("position", currentPosition); // Сохранить позицию
    Serial.println("Достигнута максимальная позиция (8000)");
  } else if (currentPosition <= 0) {
    currentPosition = 0;
    enableMotor(false); // Отключить двигатель
    preferences.putLong("position", currentPosition); // Сохранить позицию
    Serial.println("Достигнута минимальная позиция (0)");
  }
}

// Создание экземпляра веб-сервера
WebServer server(80);

void setup() {
  Serial.begin(115200);

  // Инициализация Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Подключение к Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi подключен.");
  Serial.print("IP адрес: ");
  Serial.println(WiFi.localIP());

  // Настройка маршрутов веб-сервера
  server.on("/", handleRoot);
  server.on("/setEffect", handleSetEffect);
  server.on("/setHue", handleSetHue);
  server.begin();
  Serial.println("Веб-сервер запущен.");

  // Инициализация пинов
  pinMode(TRANSISTOR_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP); // Если используете кнопку
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Кнопка подключена к GND и пину с подтяжкой
  pinMode(TOGGLE_PIN, OUTPUT);

  // Изначально двигатель и TOGGLE_PIN выключены
  enableMotor(false);
  digitalWrite(TOGGLE_PIN, LOW);
  toggleState = LOW;


  // Инициализация EEPROM Preferences
  preferences.begin(PREF_NAMESPACE, false);
  currentPosition = preferences.getLong("position", 0); // Чтение сохранённой позиции
  Serial.print("Текущая позиция двигателя: ");
  Serial.println(currentPosition);

  // Инициализация светодиодной ленты
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(maxBrightness);

  // Изначально выключаем ленту
  digitalWrite(TRANSISTOR_PIN, LOW);

  // Чтение начального состояния CLK энкодера
  lastStateCLK = digitalRead(CLK_PIN);

   xTaskCreatePinnedToCore(
    stepperTask,   // Функция задачи
    "StepperTask", // Имя задачи
    2000,         // Размер стека задачи в байтах (проверьте, достаточно ли)
    NULL,          // Параметр для функции задачи (если не требуется, передаем NULL)
    1,             // Приоритет задачи (1 - низкий, выше - выше приоритет)
    NULL,          // Указатель на дескриптор задачи (если не требуется, передаем NULL)
    0);            // Ядро, на котором выполняется задача (0 или 1)


  Serial.println("Программа запущена.");
}

void stepperTask(void *pvParameters) {
  const int threshold1 = POT_MAX / 3;
  const int threshold2 = 2 * POT_MAX / 3;
  const int hysteresis = 50; // Настройте значение по необходимости

  int lastZone = -1; // Хранит предыдущую зону

  while (true) {
    int potValue = analogRead(POT_PIN_2);

    // Добавляем фильтрацию показаний потенциометра (скользящее среднее)
    const int numReadings = 10;
    static int readings[numReadings] = {0};
    static int readIndex = 0;
    static long total = 0;
    static int average = 0;

    total -= readings[readIndex];
    readings[readIndex] = potValue;
    total += readings[readIndex];
    readIndex = (readIndex + 1) % numReadings;
    average = total / numReadings;

    potValue = average;

    int zone;

    // Определяем зону с учетом гистерезиса
    if (lastZone == 0) {
      if (potValue < threshold1 + hysteresis) {
        zone = 0;
      } else if (potValue >= threshold1 + hysteresis && potValue < threshold2 - hysteresis) {
        zone = 1;
      } else {
        zone = 2;
      }
    } else if (lastZone == 1) {
      if (potValue < threshold1 - hysteresis) {
        zone = 0;
      } else if (potValue >= threshold1 - hysteresis && potValue < threshold2 + hysteresis) {
        zone = 1;
      } else {
        zone = 2;
      }
    } else if (lastZone == 2) {
      if (potValue < threshold1 - hysteresis) {
        zone = 0;
      } else if (potValue >= threshold1 - hysteresis && potValue < threshold2 - hysteresis) {
        zone = 1;
      } else {
        zone = 2;
      }
    } else {
      // Если lastZone не определена, определяем зону без гистерезиса
      if (potValue < threshold1) {
        zone = 0;
      } else if (potValue < threshold2) {
        zone = 1;
      } else {
        zone = 2;
      }
    }

    // Если зона изменилась, обновляем lastZone и выполняем действия
    if (zone != lastZone) {
      lastZone = zone;

      // Отладочная информация
      Serial.print("potValue: ");
      Serial.print(potValue);
      Serial.print(" | Zone: ");
      Serial.println(zone);

      // Обработка положения потенциометра для управления двигателем
      if (zone == 0) {
        // Движение вперед
        if (!motorEnabled && currentPosition < MAX_POSITION) {
          enableMotor(true); // Включить двигатель
          setDirection(true); // Установить направление вперед
          lastStepTime = micros(); // Инициализируем таймер для шагов
          Serial.println("Движение вперед");
        } else if (motorEnabled && !direction && currentPosition < MAX_POSITION) {
          // Двигатель включен и направление не вперед, сменим направление на вперед
          setDirection(true);
          lastStepTime = micros();
          Serial.println("Смена направления на вперед");
        }
      }
      else if (zone == 1) {
        // Удержание позиции
        if (motorEnabled) {
          enableMotor(false); // Отключить двигатель
          preferences.putLong("position", currentPosition); // Сохранить позицию
          Serial.println("Удержание позиции");
        }
      }
      else if (zone == 2) {
        // Движение назад
        if (!motorEnabled && currentPosition > 0) {
          enableMotor(true); // Включить двигатель
          setDirection(false); // Установить направление назад
          lastStepTime = micros(); // Инициализируем таймер для шагов
          Serial.println("Движение назад");
        } else if (motorEnabled && direction && currentPosition > 0) {
          // Двигатель включен и направление не назад, сменим направление на назад
          setDirection(false);
          lastStepTime = micros();
          Serial.println("Смена направления на назад");
        }
      }
    }

    // Управление двигателем (остается без изменений)
    if (motorEnabled) {
      unsigned long currentTime = micros();
      if (currentTime - lastStepTime >= STEP_INTERVAL_US) {
        lastStepTime = currentTime;

        // Проверяем, не достигли ли крайней позиции
        if ((direction && currentPosition < MAX_POSITION) ||
            (!direction && currentPosition > 0)) {
          stepMotor();
        } else {
          enableMotor(false); // Отключить двигатель
          preferences.putLong("position", currentPosition); // Сохранить позицию
          Serial.println("Достигнута крайняя позиция, двигатель остановлен");
        }
      }
    }

    // Обработка кнопки с антидребезгом (остается без изменений)
    int reading = digitalRead(BUTTON_PIN);

    if (reading != lastButtonLightState) {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
      if (reading != buttonLightState) {
        buttonLightState = reading;

        if (buttonLightState == LOW) {
          Serial.println("Кнопка нажата");
          toggleState = !toggleState;
          digitalWrite(TOGGLE_PIN, toggleState);
          Serial.print("TOGGLE_PIN установлен в: ");
          Serial.println(toggleState == HIGH ? "HIGH" : "LOW");
        } else if (buttonLightState == HIGH) {
          Serial.println("Кнопка отпущена");
        }
      }
    }

    lastButtonLightState = reading;

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}


void loop() {
  // Обработка клиентов веб-сервера
  server.handleClient();

  // === Обработка энкодера ===
  // Чтение текущего состояния CLK
  currentStateCLK = digitalRead(CLK_PIN);

  // Если состояние CLK изменилось с HIGH на LOW
  if (lastStateCLK == HIGH && currentStateCLK == LOW) {
    // Проверяем направление вращения
    if (digitalRead(DT_PIN) == HIGH) {
      // Вращение по часовой стрелке
      currentEffect++;
      if (currentEffect >= numEffects) { // Не включаем режим пользовательского цвета
        currentEffect = 0;
      }
      Serial.print("Переключен на эффект №");
      Serial.println(currentEffect);
    } else {
      // Вращение против часовой стрелки
      if (currentEffect == 0) {
        currentEffect = numEffects - 1; // Не включаем режим пользовательского цвета
      } else {
        currentEffect--;
      }
      Serial.print("Переключен на эффект №");
      Serial.println(currentEffect);
    }
    lastEncoderTurnTime = millis();
  }

  // Обновляем lastStateCLK
  lastStateCLK = currentStateCLK;

  // === Обработка кнопки энкодера ===
  // Чтение текущего состояния кнопки
  int reading = digitalRead(SW_PIN);

  // Проверка на изменение состояния кнопки для антидребезга
  if (reading != lastButtonState) {
    buttonDebounceTime = millis();
  }

  if ((millis() - buttonDebounceTime) > debounceDelay) {
    // Если состояние кнопки изменилось после антидребезга
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        // Кнопка была нажата
        buttonPressTime = millis();
        isButtonPressed = true;
      } else {
        // Кнопка была отпущена
        isButtonPressed = false;
        unsigned long pressDuration = millis() - buttonPressTime;

        if (pressDuration < longPressTime) {
          // Короткое нажатие
          Serial.println("Кнопка энкодера нажата (короткое нажатие)");
          currentEffect = 2; // Переключаемся на режим №2 (белый свет)
        } else {
          // Длинное нажатие закончилось
          Serial.println("Кнопка энкодера отпущена после удержания");
          isColorChangeMode = false; // Выходим из режима изменения цвета
          currentEffect = 3; // Устанавливаем текущий эффект на пользовательский цвет
          //userSelectedHue = lastUserSelectedHue; // Сохраняем последний выбранный цвет
        }
      }
    }
  }

  // Обработка удержания кнопки
  if (isButtonPressed && (millis() - buttonPressTime > longPressTime) && !isColorChangeMode) {
    Serial.println("Кнопка энкодера удерживается более 1 секунды");
    isColorChangeMode = true; // Входим в режим изменения цвета
  }

  // Обновляем lastButtonState
  lastButtonState = reading;

  // === Обработка режима изменения цвета ===
  if (isColorChangeMode) {
    // Чтение значения потенциометра (0 - 4095 для ESP32)
    int potValue = analogRead(POT_PIN);
    uint8_t hueValue = map(potValue, 0, 4095, 0, 255); // Маппинг значения потенциометра на оттенок
    //Serial.println("Обработка режима изменения цвета");

    // Устанавливаем цвет всей ленты
    fill_solid(leds, NUM_LEDS, CHSV(hueValue, 255, maxBrightness));
    FastLED.show();

    // Сохраняем последний выбранный цвет
    userSelectedHue = hueValue;
  } else {
    // === Обработка потенциометра и яркости ===
    // Чтение значения потенциометра (0 - 4095 для ESP32)
    int potValue = analogRead(POT_PIN);
    //Serial.print("Значение потенциометра: ");
    //Serial.println(potValue);

    // Калибровка потенциометра: минимальное положение
    if (potValue < THRESHOLD) {
      // Если потенциометр в минимальном положении, выключаем ленту
      digitalWrite(TRANSISTOR_PIN, LOW);
      //Serial.println("Лента выключена (потенциометр в минимальном положении)");
      FastLED.clear();
      FastLED.show();
      return; // Выходим из цикла, чтобы не выполнять дальнейшую обработку
    }

    // Иначе включаем ленту
    digitalWrite(TRANSISTOR_PIN, HIGH);
    //Serial.println("Лента включена");

    // Карта значения потенциометра на уровень яркости (0 - 255)
    uint8_t brightness = map(potValue, THRESHOLD, 4095, 0, 255);
    brightness = constrain(brightness, 0, 255);

    // Обновляем максимальную яркость
    maxBrightness = brightness;

    // === Выполнение выбранного эффекта ===
    switch (currentEffect) {
      case 0:
        rainbowEffect();
        break;
      case 1:
        breatheEffect();
        break;
      case 2:
        whiteEffect();
        break;
      case 3:
        userColorEffect(); // Добавляем обработку пользовательского цвета
        break;
      default:
        fill_solid(leds, NUM_LEDS, CRGB::White);
        FastLED.show();
        break;
    }
  }

  // Короткая задержка для стабильности
  delay(5);
}

// Эффект радуги
void rainbowEffect() {
  unsigned long currentTime = millis();
  if (currentTime - lastRainbowUpdate >= rainbowInterval) {
    lastRainbowUpdate = currentTime;
    hue++; // Инкрементируем оттенок
  }
  fill_rainbow(leds, NUM_LEDS, hue, 7);
  FastLED.setBrightness(maxBrightness); // Устанавливаем максимальную яркость
  FastLED.show();
}

// Эффект дыхания с плавной сменой цвета
void breatheEffect() {
  unsigned long currentTime = millis();
  if (currentTime - lastBreatheUpdate > breatheInterval) {
    lastBreatheUpdate = currentTime;

    // Обновляем яркость, не превышая maxBrightness
    breatheBrightness += breatheStep;
    if (breatheBrightness >= maxBrightness) {
      breatheBrightness = maxBrightness;
      breatheStep = -1;
    }
    if (breatheBrightness <= 0) {
      breatheBrightness = 0;
      breatheStep = 1;
    }

    // Устанавливаем текущую яркость
    FastLED.setBrightness(breatheBrightness);

    // Обновляем цвет для плавной смены
    breatheHue += 1; // Увеличиваем оттенок для плавного перехода
    fill_solid(leds, NUM_LEDS, CHSV(breatheHue, 255, 255)); // Используем полную насыщенность и яркость
    FastLED.show();
  }
}

// Эффект чисто белого света
void whiteEffect() {
  FastLED.setBrightness(maxBrightness); // Устанавливаем максимальную яркость
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();
}

// Эффект пользовательского цвета
void userColorEffect() {
  FastLED.setBrightness(maxBrightness);
  fill_solid(leds, NUM_LEDS, CHSV(userSelectedHue, 255, 255));
  FastLED.show();
}

// Обработка главной страницы
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Управление лентой</title>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; text-align: center; background-color: #f0f0f0; }";
  html += "h1 { color: #333; }";
  html += ".button { padding: 10px 20px; margin: 5px; font-size: 16px; }";
  html += ".slider { width: 300px; }";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>Управление режимами светодиодной ленты</h1>";
  
  // Отображаем текущий режим
  String effectName;
  switch (currentEffect) {
    case 0:
      effectName = "Радуга";
      break;
    case 1:
      effectName = "Дыхание";
      break;
    case 2:
      effectName = "Белый свет";
      break;
    case 3:
      effectName = "Пользовательский цвет";
      break;
    default:
      effectName = "Неизвестный режим";
      break;
  }
  
  html += "<p>Текущий режим: " + effectName + "</p>";
  
  // Кнопки для переключения режимов
  html += "<button class='button' onclick=\"location.href='/setEffect?effect=0'\">Радуга</button>";
  html += "<button class='button' onclick=\"location.href='/setEffect?effect=1'\">Дыхание</button>";
  html += "<button class='button' onclick=\"location.href='/setEffect?effect=2'\">Белый свет</button>";
  html += "<button class='button' onclick=\"location.href='/setEffect?effect=3'\">Пользовательский цвет</button>";
  
  // Добавляем ползунок для изменения userSelectedHue
  html += "<h2>Изменение оттенка цвета</h2>";
  html += "<form action='/setHue' method='GET'>";
  html += "<input type='range' min='0' max='255' value='" + String(userSelectedHue) + "' class='slider' name='hue' onchange='this.form.submit()'>";
  html += "<p>Текущий оттенок: " + String(userSelectedHue) + "</p>";
  html += "</form>";
  
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

// Обработка установки режима
void handleSetEffect() {
  if (server.hasArg("effect")) {
    currentEffect = server.arg("effect").toInt();
    // Проверяем диапазон режимов
    if (currentEffect < 0 || currentEffect >= numEffects) {
      currentEffect = 0;
    }
    Serial.print("Режим установлен через веб на: ");
    Serial.println(currentEffect);
  }
  server.sendHeader("Location", "/"); // Перенаправляем на главную страницу
  server.send(303);
}

void handleSetHue() {
  if (server.hasArg("hue")) {
    userSelectedHue = server.arg("hue").toInt();
    // Ограничиваем значение от 0 до 255
    if (userSelectedHue < 0) userSelectedHue = 0;
    if (userSelectedHue > 255) userSelectedHue = 255;
    Serial.print("Оттенок установлен через веб на: ");
    Serial.println(userSelectedHue);
    
    // Если текущий режим не пользовательский, переключаемся на него
    if (currentEffect != 3) {
      currentEffect = 3;
    }
  }
  
  // Перенаправляем обратно на главную страницу
  server.sendHeader("Location", "/");
  server.send(303);
}
