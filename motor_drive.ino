#define MOTOR_LEFT_PWM_PIN 12     // ШИМ пин для левого мотора
#define MOTOR_RIGHT_PWM_PIN 2     // ШИМ пин для правого мотора
#define MOTOR_LEFT_DIR_PIN 11     // Пин направления для левого мотора
#define MOTOR_RIGHT_DIR_PIN 3     // Пин направления для правого мотора

#define DEFAULT_SPEED 255         // Стандартная скорость (0-255)
#define INVERT_RIGHT_MOTOR true   // Инвертировать правый мотор

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

TankDrive tank(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN, 
               MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN, INVERT_RIGHT_MOTOR);

void setup() {
  // Инициализация Serial
  Serial.begin(9600);
  Serial.println("Формат команд: <левый мотор>,<правый мотор>");
  Serial.println("Скорость от -255 до 255 для каждого мотора");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() < 1) return;
    
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
      Serial.print("Левый мотор: ");
      Serial.print(leftSpeed);
      Serial.print(", Правый мотор: ");
      Serial.println(rightSpeed);
    } else {
      // Неверный формат команды
      Serial.println("Ошибка! Используйте формат: <левый мотор>,<правый мотор>");
      Serial.println("Например: 200,150");
    }
  }
}