#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>
#include <MQ135.h>

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
    
    float readTemperature() {
        return bmp.readTemperature();
    }
    
    float readPressure() {
        return bmp.readPressure() / 100.0; // Перевод в hPa
    }
    
    float readAltitude() {
        return bmp.readAltitude();
    }
};

// Класс для датчика MQ135
class MQ135Sensor {
private:
    MQ135 mq135;
    
public:
    MQ135Sensor(uint8_t pin) : mq135(pin) {}
    
    float readPPM() {
        return mq135.getPPM();
    }
};

// Класс для вывода данных
class DataPrinter {
public:
    static void printAirQuality(float value) {
        Serial.print("MQ135 (CO2 PPM): ");
        Serial.println(value);
    }
    
    static void printBMPTemperature(float value) {
        Serial.print("BMP180 Temp: ");
        Serial.print(value);
        Serial.println(" C");
    }
    
    static void printBMPPressure(float value) {
        Serial.print("BMP180 Pressure: ");
        Serial.print(value);
        Serial.println(" hPa");
    }
    
    static void printBMPAltitude(float value) {
        Serial.print("BMP180 Altitude: ");
        Serial.print(value);
        Serial.println(" m");
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
};

// Объявление объектов
#define DHTPIN 2
#define DHTTYPE DHT11

DHT11Sensor dhtSensor(DHTPIN, DHTTYPE);
BMP180Sensor bmpSensor;
MQ135Sensor mq135Sensor(A0);

void setup() {
    Serial.begin(9600);
    
    dhtSensor.begin();
    
    if (!bmpSensor.begin()) {
        Serial.println("Ошибка BMP180!");
        while (1);
    }
    
    Serial.println("Инициализация завершена.");
}

void loop() {
    // Чтение MQ135 (качество воздуха, CO2)
    float air_quality = mq135Sensor.readPPM();
    
    // Чтение BMP180 (температура, давление, высота)
    float temperature_bmp = bmpSensor.readTemperature();
    float pressure = bmpSensor.readPressure();
    float altitude = bmpSensor.readAltitude();
    
    // Чтение DHT11 (температура и влажность)
    float temperature_dht = dhtSensor.readTemperature();
    float humidity = dhtSensor.readHumidity();

    // Вывод данных в Serial Monitor
    DataPrinter::printAirQuality(air_quality);
    DataPrinter::printBMPTemperature(temperature_bmp);
    DataPrinter::printBMPPressure(pressure);
    DataPrinter::printBMPAltitude(altitude);
    DataPrinter::printDHTTemperature(temperature_dht);
    DataPrinter::printDHTHumidity(humidity);
    DataPrinter::printSeparator();
    
    delay(2000); // Интервал между измерениями
}