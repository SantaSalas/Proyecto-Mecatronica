
#include <Wire.h>

//////////////////// CONFIGURACIÓN HARDWARE ////////////////////
const int MPU6050_ADDR = 0x68;
const int ENA = 5;         // PWM motor izquierdo
const int IN1 = 7;         // Motor izquierdo dirección
const int IN2 = 6;         // Motor izquierdo dirección  
const int ENB = 11;        // PWM motor derecho
const int IN3 = 4;         // Motor derecho dirección
const int IN4 = 3;         // Motor derecho dirección

//////////////////// PARÁMETROS PID ////////////////////////////
float Kp = 23;              // 
float Ki = 1;              // 
float Kd = 9;              // 
float setpoint = 0.0;      // Ángulo objetivo

//////////////////// PARÁMETROS SISTEMA ////////////////////////
const float ALPHA = 0.98;           // Filtro complementario
const int ZONA_MUERTA = 40;        // PWM donde motores no responden
const int PWM_MIN = 50;             // PWM mínimo efectivo
const int PWM_MAX = 255;           // PWM máximo
const float ANGULO_LIMITE = 45.0;  // Ángulo de seguridad

//////////////////// VARIABLES GLOBALES ////////////////////////
float angulo = 0.0;
float error = 0.0;
float error_prev = 0.0;
float integral = 0.0;
float derivada = 0.0;
float salida = 0.0;

unsigned long lastTime = 0;
const unsigned long sampleTime = 10;  // Control a 100Hz

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
float gyroY_offset = 0;

bool calibrado = false;
unsigned long tiempoInicio = 0;

//////////////////// SETUP //////////////////////
void setup() {
  // Configurar pines motores
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Asegurar motores detenidos
  detenerMotores();
  
  // Configurar MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0);     // Despertar sensor
  Wire.endTransmission(true);
  
  // Esperar estabilización del sensor
  delay(500);
  
  // Calibración automática (3 segundos)
  calibrarSensor();
  
  // Inicializar tiempo
  lastTime = millis();
  tiempoInicio = millis();
}

//////////////////// LOOP PRINCIPAL //////////////////////
void loop() {
  unsigned long now = millis();
  
  // Control PID a 100Hz
  if(now - lastTime >= sampleTime) {
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    
    // 1) Leer sensor y calcular ángulo
    leerSensor();
    calcularAngulo(dt);
    
    // 2) Verificar límite de seguridad
    if(abs(angulo) > ANGULO_LIMITE) {
      detenerMotores();
      delay(100);
      return;
    }
    
    // 3) Calcular error
    error = setpoint - angulo;
    
    // 4) Cálculo PID
    float P = Kp * error;
    
    integral += error * dt;
    float I = Ki * integral;
    
    derivada = (error - error_prev) / dt;
    float D = Kd * derivada;
    
    salida = P + I + D;
    
    // 5) Anti-windup
    if(salida > PWM_MAX) {
      salida = PWM_MAX;
      integral -= error * dt;
    } else if(salida < -PWM_MAX) {
      salida = -PWM_MAX;
      integral -= error * dt;
    }
    
    // Limitar integral
    integral = constrain(integral, -100, 100);
    
    // 6) Aplicar a motores
    int pwm = (int)salida;
    controlarMotores(pwm);
    
    // 7) Guardar error anterior
    error_prev = error;
  }
}

//////////////////// FUNCIONES AUXILIARES //////////////////////

void calibrarSensor() {
  // Calibración silenciosa (sin Serial)
  gyroY_offset = 0;
  long sumaGyroY = 0;
  long sumaAccelX = 0;
  const int numMuestras = 100;
  
  for(int i = 0; i < numMuestras; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
    
    accelX = Wire.read() << 8 | Wire.read();
    accelY = Wire.read() << 8 | Wire.read();
    accelZ = Wire.read() << 8 | Wire.read();
    int16_t temp = Wire.read() << 8 | Wire.read();
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
    
    sumaGyroY += gyroY;
    sumaAccelX += accelX;
    
    delay(30);  // Total ~3 segundos de calibración
  }
  
  gyroY_offset = (sumaGyroY / numMuestras) / 131.0;
  
  // Calcular ángulo inicial (por si no está perfectamente horizontal)
  float accelX_prom = sumaAccelX / numMuestras;
  angulo = atan2(accelX_prom, accelZ) * 180.0 / PI;
  
  calibrado = true;
}

void leerSensor() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void calcularAngulo(float dt) {
  // Ángulo desde acelerómetro
  float accelAngulo = -atan2(accelX, accelZ) * 180.0 / PI;
  
  // Velocidad angular desde giroscopio
  float gyroRate = (gyroY / 131.0) - gyroY_offset;
  
  // Filtro complementario
  angulo = ALPHA * (angulo + gyroRate * dt) + (1 - ALPHA) * accelAngulo;
}

void controlarMotores(int velocidad) {
  // Zona muerta
  if(abs(velocidad) < ZONA_MUERTA) {
    detenerMotores();
    return;
  }
  
  // Mapear saltando zona muerta
  int vel = abs(velocidad);
  if(vel > ZONA_MUERTA) {
    vel = map(vel, ZONA_MUERTA, PWM_MAX, PWM_MIN, PWM_MAX);
  }
  vel = constrain(vel, 0, PWM_MAX);
  
  if(velocidad > 0) {
    // Inclinado adelante -> mover adelante
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, vel);
    
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, vel);
  } else {
    // Inclinado atrás -> mover atrás
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, vel);
    
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, vel);
  }
}

void detenerMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

/*
  ============================================
         INSTRUCCIONES DE USO FINAL
  ============================================
     - Colocar robot HORIZONTAL
     - Encender con switch
     - Esperar 3 segundos (calibración)
     - Robot se balancea automáticamente
  ============================================
*/