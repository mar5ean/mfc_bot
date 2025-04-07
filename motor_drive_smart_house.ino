#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>
#include <MQ135.h>

// Пины для моторов
#define MOTOR_LEFT_PWM_PIN 12     // ШИМ пин для левого мотора
#define MOTOR_RIGHT_PWM_PIN 2     // ШИМ пин для правого мотора
#define MOTOR_LEFT_DIR_PIN 11     // Пин направления для левого мотора
#define MOTOR_RIGHT_DIR_PIN 3     // Пин направления для правого мотора

#define DEFAULT_SPEED 255         // Стандартная скорость (0-255)
#define INVERT_RIGHT_MOTOR true   // Инвертировать правый мотор

// Пины для датчиков
#define DHTPIN 5                  // Пин для DHT11
#define DHTTYPE DHT11
#define MQ135PIN A0               // Аналоговый пин для MQ135

// Интервал чтения сенсоров
#define SENSOR_READ_INTERVAL 50000 // Интервал считывания датчиков (мс)

// ================= Классы для моторов ===================

class Motor {
private:
  uint8_t pwmPin;
  uint8_t dirPin;
  uint8_t currentSpeed;
  bool currentDirection;  // true - вперед, false - назад
  bool inverted;          // Инвертировать ли направление мотора

public:
  Motor(uint8_t pwm, uint8_t dir, bool invert = false) {
    pwmPin = pwm;
    dirPin = dir;
    currentSpeed = 0;
    currentDirection = true;
    inverted = invert;
    
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    
    // Начальное состояние - мотор остановлен
    setSpeed(0);
    setDirection(true);
  }
  
  // Установка скорости (0-255)
  void setSpeed(uint8_t speed) {
    currentSpeed = constrain(speed, 0, 255);
    // Инвертируем скорость для XY-BLDC (0 - полная скорость, 255 - остановка)
    analogWrite(pwmPin, 255 - currentSpeed);
  }
  
  // Установка направления (true - вперед, false - назад)
  void setDirection(bool forward) {
    currentDirection = forward;
    // Если мотор инвертирован, меняем направление на противоположное
    bool actualDirection = inverted ? !forward : forward;
    digitalWrite(dirPin, actualDirection ? HIGH : LOW);
  }
  
  // Остановка мотора
  void stop() {
    setSpeed(0);
  }
  
  // Получение текущей скорости
  uint8_t getSpeed() {
    return currentSpeed;
  }
  
  // Получение текущего направления
  bool getDirection() {
    return currentDirection;
  }
};

class TankDrive {
private:
  Motor leftMotor;
  Motor rightMotor;
  
public:
  TankDrive(uint8_t leftPwm, uint8_t leftDir, uint8_t rightPwm, uint8_t rightDir, bool invertRight) 
    : leftMotor(leftPwm, leftDir, false), rightMotor(rightPwm, rightDir, invertRight) {
  }
  
  // Установка скорости и направления для левой гусеницы
  void setLeft(int speed) {
    // Положительная скорость - вперед, отрицательная - назад
    bool direction = speed >= 0;
    leftMotor.setDirection(direction);
    leftMotor.setSpeed(abs(speed));
  }
  
  // Установка скорости и направления для правой гусеницы
  void setRight(int speed) {
    // Положительная скорость - вперед, отрицательная - назад
    bool direction = speed >= 0;
    rightMotor.setDirection(direction);
    rightMotor.setSpeed(abs(speed));
  }
  
  // Остановка обеих гусениц
  void stop() {
    leftMotor.stop();
    rightMotor.stop();
  }
  
  // Движение вперед с заданной скоростью
  void forward(uint8_t speed = DEFAULT_SPEED) {
    setLeft(speed);
    setRight(speed);
  }
  
  // Движение назад с заданной скоростью
  void backward(uint8_t speed = DEFAULT_SPEED) {
    setLeft(-speed);
    setRight(-speed);
  }
  
  // Поворот на месте влево (левая гусеница назад, правая вперед)
  void rotateLeft(uint8_t speed = DEFAULT_SPEED) {
    setLeft(-speed);
    setRight(speed);
  }
  
  // Поворот на месте вправо (левая гусеница вперед, правая назад)
  void rotateRight(uint8_t speed = DEFAULT_SPEED) {
    setLeft(speed);
    setRight(-speed);
  }
  
  // Плавный поворот влево (правая гусеница быстрее)
  void turnLeft(uint8_t speed = DEFAULT_SPEED, uint8_t turnRate = 50) {
    uint8_t leftSpeed = map(turnRate, 0, 100, speed, 0);
    setLeft(leftSpeed);
    setRight(speed);
  }
  
  // Плавный поворот вправо (левая гусеница быстрее)
  void turnRight(uint8_t speed = DEFAULT_SPEED, uint8_t turnRate = 50) {
    uint8_t rightSpeed = map(turnRate, 0, 100, speed, 0);
    setLeft(speed);
    setRight(rightSpeed);
  }
};

// ================= Классы для датчиков ===================

// Класс для датчика DHT11
class DHT11Sensor {
private:
    DHT dht;
    
public:
    DHT11Sensor(uint8_t pin, uint8_t type) : dht(pin, type) {}
    
    void begin() {
        dht.begin();
    }
    
    float readTemperature() {
        return dht.readTemperature();
    }
    
    float readHumidity() {
        return dht.readHumidity();
    }
};

// Класс для датчика BMP180
class BMP180Sensor {
private:
    Adafruit_BMP085 bmp;
    
public:
    bool begin() {
        return bmp.begin();
    }
    
    float readPressure() {
        return bmp.readPressure() / 100.0; // Перевод в hPa
    }
};

// Класс для датчика MQ135
class MQ135Sensor {
private:
    MQ135 mq135;
    int pin;
    
public:
    MQ135Sensor(uint8_t analogPin) : mq135(analogPin), pin(analogPin) {}
    
    float readPPM() {
        return mq135.getPPM();
    }
    
    int readRawValue() {
        return analogRead(pin);
    }
    
    // Приблизительные расчеты концентраций различных газов
    float readCO() {
        // Приблизительное преобразование для CO (угарный газ)
        return map(readRawValue(), 0, 1023, 0, 1000) / 10.0;
    }
    
    float readNH3() {
        // Приблизительное преобразование для NH3 (аммиак)
        return map(readRawValue(), 0, 1023, 0, 500) / 10.0;
    }
    
    float readNOx() {
        // Приблизительное преобразование для NOx (оксиды азота)
        return map(readRawValue(), 0, 1023, 0, 300) / 10.0;
    }
    
    float readCH4() {
        // Приблизительное преобразование для CH4 (метан)
        return map(readRawValue(), 0, 1023, 0, 2000) / 10.0;
    }
    
    String getAirQuality() {
        float ppm = readPPM();
        if (ppm < 700) return "Отличное";
        else if (ppm < 1000) return "Хорошее";
        else if (ppm < 1500) return "Среднее";
        else if (ppm < 2000) return "Плохое";
        else return "Опасное";
    }
};

// Класс для вывода данных
class DataPrinter {
public:
    static void printAirQuality(MQ135Sensor &sensor) {
        float co2 = sensor.readPPM();
        float co = sensor.readCO();
        float nh3 = sensor.readNH3();
        float nox = sensor.readNOx();
        float ch4 = sensor.readCH4();
        int raw = sensor.readRawValue();
        String quality = sensor.getAirQuality();
        
        Serial.println("Данные датчика MQ135:");
        Serial.print("  CO2: ");
        Serial.print(co2);
        Serial.println(" PPM");
        
        Serial.print("  CO (угарный газ): ");
        Serial.print(co);
        Serial.println(" PPM");
        
        Serial.print("  NH3 (аммиак): ");
        Serial.print(nh3);
        Serial.println(" PPM");
        
        Serial.print("  NOx (оксиды азота): ");
        Serial.print(nox);
        Serial.println(" PPM");
        
        Serial.print("  CH4 (метан): ");
        Serial.print(ch4);
        Serial.println(" PPM");
        
        Serial.print("  Сырое значение ADC: ");
        Serial.println(raw);
        
        Serial.print("  Качество воздуха: ");
        Serial.println(quality);
    }
    
    static void printBMPPressure(float value) {
        Serial.print("BMP180 Pressure: ");
        Serial.print(value);
        Serial.println(" hPa");
    }
    
    static void printDHTTemperature(float value) {
        Serial.print("DHT11 Temp: ");
        Serial.print(value);
        Serial.println(" C");
    }
    
    static void printDHTHumidity(float value) {
        Serial.print("DHT11 Humidity: ");
        Serial.print(value);
        Serial.println(" %");
    }
    
    static void printSeparator() {
        Serial.println("----------------------------");
    }
    
    static void printMotorStatus(int leftSpeed, int rightSpeed) {
        Serial.print("Левый мотор: ");
        Serial.print(leftSpeed);
        Serial.print(", Правый мотор: ");
        Serial.println(rightSpeed);
    }
    
    static void printHelp() {
        Serial.println("Формат команд:");
        Serial.println("  <левый мотор>,<правый мотор> - управление моторами");
        Serial.println("  sensors - показать данные с датчиков");
        Serial.println("  help - показать эту справку");
    }
};

// ================= Глобальные объекты =================

TankDrive tank(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN, 
               MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN, INVERT_RIGHT_MOTOR);

DHT11Sensor dhtSensor(DHTPIN, DHTTYPE);
BMP180Sensor bmpSensor;
MQ135Sensor mq135Sensor(MQ135PIN);

unsigned long lastSensorRead = 0;

// ================= Функции =================

// Функция для считывания и вывода данных с датчиков
void readAndPrintSensorData() {
    // Вывод данных с MQ135
    DataPrinter::printAirQuality(mq135Sensor);
    
    // Чтение BMP180 (давление)
    float pressure = bmpSensor.readPressure();
    
    // Чтение DHT11 (температура и влажность)
    float temperature_dht = dhtSensor.readTemperature();
    float humidity = dhtSensor.readHumidity();

    // Вывод данных в Serial Monitor
    DataPrinter::printBMPPressure(pressure);
    DataPrinter::printDHTTemperature(temperature_dht);
    DataPrinter::printDHTHumidity(humidity);
    DataPrinter::printSeparator();
    
    // Сразу запрашиваем новую команду
    Serial.println("Введите команду:");
}

// ================= SETUP =================

void setup() {
    // Инициализация Serial
    Serial.begin(9600);
    
    // Инициализация датчиков
    dhtSensor.begin();
    
    if (!bmpSensor.begin()) {
        Serial.println("Ошибка инициализации BMP180!");
        // Не блокируем выполнение, продолжаем работу без BMP180
        Serial.println("Продолжение работы без BMP180.");
    } else {
        Serial.println("BMP180 успешно инициализирован.");
    }
    
    Serial.println("Система управления танковым шасси с датчиками готова.");
    DataPrinter::printHelp();
    Serial.println("Введите команду:");
}

// ================= LOOP =================

void loop() {
    // Периодическое считывание данных с датчиков без блокировки
    unsigned long currentMillis = millis();
    if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
        lastSensorRead = currentMillis;
        Serial.println("Автоматическое считывание данных с датчиков:");
        readAndPrintSensorData();
    }

    // Обработка команд из Serial
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command.length() < 1) {
            Serial.println("Введите команду:");
            return;
        }
        
        // Проверка на специальные команды
        if (command.equalsIgnoreCase("sensors")) {
            Serial.println("Считывание данных с датчиков по запросу:");
            readAndPrintSensorData();
            return;
        } 
        else if (command.equalsIgnoreCase("help")) {
            DataPrinter::printHelp();
            Serial.println("Введите команду:");
            return;
        }
        
        // Обработка команды управления моторами
        int commaIndex = command.indexOf(',');
        
        if (commaIndex > 0 && commaIndex < command.length() - 1) {
            // Разбираем команду формата <левый мотор>,<правый мотор>
            int leftSpeed = command.substring(0, commaIndex).toInt();
            int rightSpeed = command.substring(commaIndex + 1).toInt();
            
            // Ограничиваем значения в диапазоне от -255 до 255
            leftSpeed = constrain(leftSpeed, -255, 255);
            rightSpeed = constrain(rightSpeed, -255, 255);
            
            // Устанавливаем скорости для моторов
            tank.setLeft(leftSpeed);
            tank.setRight(rightSpeed);
            
            // Выводим информацию о текущем состоянии
            DataPrinter::printMotorStatus(leftSpeed, rightSpeed);
            Serial.println("Введите команду:");
        } else {
            // Неверный формат команды
            Serial.println("Ошибка! Неверный формат команды.");
            DataPrinter::printHelp();
            Serial.println("Введите команду:");
        }
    }
}