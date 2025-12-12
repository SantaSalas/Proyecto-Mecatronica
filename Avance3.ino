/*
  ROBOT AUTO-BALANCÍN - CONTROL PID COMPLETO
  ME4250 - Mecatrónica - Universidad de Chile
  Basado en estructura PID de muestra del curso
  
  Hardware: Arduino UNO + MPU6050 + L298N + 2x Motor DC
*/

#include <Wire.h>

//////////////////// CONFIGURACIÓN HARDWARE ////////////////////
// Pines MPU6050 (I2C)
const int MPU6050_ADDR = 0x68;  // Dirección I2C
// SDA -> A4, SCL -> A5 (hardware I2C)

// Pines Motor Izquierdo
const int ENA_IZQ = 5;     // PWM motor izquierdo
const int IN1_IZQ = 7;      // Control dirección
const int IN2_IZQ = 6;      // Control dirección

// Pines Motor Derecho  
const int ENB_DER = 11;     // PWM motor derecho
const int IN3_DER = 4;      // Control dirección
const int IN4_DER = 3;      // Control dirección

// Límites de seguridad
const float ANGULO_MAX = 35.0;      // [AJUSTAR] Ángulo límite seguridad
const int PWM_MIN = 50;             // [AJUSTAR] PWM mínimo efectivo
const int PWM_MAX = 255;            // PWM máximo
const int ZONA_MUERTA = 40;         // [AJUSTAR] Zona donde motores no responden

//////////////////// PARÁMETROS PID ////////////////////////////
// [AJUSTAR] Valores iniciales conservadores
float Kp = 20.0;   // Ganancia proporcional
float Ki = 0.5;    // Ganancia integral  
float Kd = 10.0;   // Ganancia derivativa
float setpoint = 0.0;  // [AJUSTAR] Ángulo objetivo (0 = vertical)

//////////////////// VARIABLES DEL CONTROL /////////////////////
float angulo = 0.0;           // Ángulo actual del robot
float error = 0.0;            // Error actual
float error_prev = 0.0;       // Error anterior
float integral = 0.0;         // Acumulador integral
float derivada = 0.0;         // Componente derivativa
float salida = 0.0;           // Salida del PID
int pwm_izq = 0;              // PWM motor izquierdo
int pwm_der = 0;              // PWM motor derecho

// Variables componentes PID (para análisis)
float P_term = 0.0;
float I_term = 0.0;
float D_term = 0.0;

// Control de tiempo
unsigned long lastTime = 0;
const unsigned long sampleTime = 10;  // [AJUSTAR] ms (100 Hz)

//////////////////// VARIABLES SENSOR MPU6050 //////////////////
// Datos crudos
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
int16_t temperatura;

// Offsets calibración
float accelX_offset = 0;
float accelY_offset = 0;
float gyroY_offset = 0;

// Filtro complementario
const float ALPHA = 0.98;  // [AJUSTAR] 98% gyro, 2% accel

// Estado del sistema
bool sistemaCalibrado = false;
bool controlActivo = false;
bool emergencyStop = false;

//////////////////// VARIABLES ANÁLISIS MÉTRICAS ///////////////
float anguloMaximo = 0.0;     // Para calcular overshoot
float tiempoEstablecido = 0.0;  // Tiempo establecimiento
unsigned long tiempoInicio = 0;
bool primeraVez = true;
float sumaError = 0.0;         // Para error estacionario
int contadorError = 0;

//////////////////// FUNCIONES AUXILIARES - MOTORES ////////////
void configurarMotores() {
  pinMode(ENA_IZQ, OUTPUT);
  pinMode(IN1_IZQ, OUTPUT);
  pinMode(IN2_IZQ, OUTPUT);
  
  pinMode(ENB_DER, OUTPUT);
  pinMode(IN3_DER, OUTPUT);
  pinMode(IN4_DER, OUTPUT);
  
  detenerMotores();
  Serial.println("[MOTORES] Configurados OK");
}

void moverMotores(int pwmValue) {
  // Aplicar zona muerta
  if (abs(pwmValue) < ZONA_MUERTA) {
    detenerMotores();
    return;
  }
  
  // Mapear considerando zona muerta
  int pwmAjustado = abs(pwmValue);
  if (pwmAjustado > ZONA_MUERTA) {
    pwmAjustado = map(pwmAjustado, ZONA_MUERTA, PWM_MAX, PWM_MIN, PWM_MAX);
  }
  
  pwmAjustado = constrain(pwmAjustado, 0, PWM_MAX);
  
  if (pwmValue > 0) {
    // Mover adelante
    digitalWrite(IN1_IZQ, HIGH);
    digitalWrite(IN2_IZQ, LOW);
    analogWrite(ENA_IZQ, pwmAjustado);
    
    digitalWrite(IN3_DER, HIGH);
    digitalWrite(IN4_DER, LOW);
    analogWrite(ENB_DER, pwmAjustado);
    
  } else {
    // Mover atrás
    digitalWrite(IN1_IZQ, LOW);
    digitalWrite(IN2_IZQ, HIGH);
    analogWrite(ENA_IZQ, pwmAjustado);
    
    digitalWrite(IN3_DER, LOW);
    digitalWrite(IN4_DER, HIGH);
    analogWrite(ENB_DER, pwmAjustado);
  }
}

void detenerMotores() {
  digitalWrite(IN1_IZQ, LOW);
  digitalWrite(IN2_IZQ, LOW);
  analogWrite(ENA_IZQ, 0);
  
  digitalWrite(IN3_DER, LOW);
  digitalWrite(IN4_DER, LOW);
  analogWrite(ENB_DER, 0);
}

//////////////////// FUNCIONES AUXILIARES - MPU6050 ////////////
void configurarMPU6050() {
  Wire.begin();
  
  // Despertar MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0);     // Despertar
  Wire.endTransmission(true);
  
  delay(100);
  
  // Verificar comunicación
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("[MPU6050] Detectado OK");
  } else {
    Serial.println("[ERROR] MPU6050 no detectado!");
    emergencyStop = true;
  }
}

void leerMPU6050Raw() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  temperatura = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

float calcularAngulo(float dt) {
  // Ángulo desde acelerómetro
  float accelAngulo = atan2(accelX - accelX_offset, accelZ) * 180 / PI;
  
  // Velocidad angular desde giroscopio
  float gyroRate = (gyroY / 131.0) - gyroY_offset;
  
  // Filtro complementario
  angulo = ALPHA * (angulo + gyroRate * dt) + (1 - ALPHA) * accelAngulo;
  
  return angulo;
}

void calibrarSensor() {
  Serial.println("\n[CALIBRACIÓN] Mantener robot VERTICAL y QUIETO");
  Serial.print("Calibrando");
  
  // Reset valores
  accelX_offset = 0;
  accelY_offset = 0;
  gyroY_offset = 0;
  angulo = 0;
  
  // Tomar muestras
  const int numMuestras = 200;
  long sumaAX = 0, sumaAY = 0, sumaGX = 0;
  
  for (int i = 0; i < numMuestras; i++) {
    leerMPU6050Raw();
    sumaAX += accelX;
    sumaAY += accelY;
    sumaGX += gyroX;
    
    if (i % 40 == 0) Serial.print(".");
    delay(5);
  }
  
  // Calcular offsets
  accelX_offset = sumaAX / numMuestras;
  accelY_offset = sumaAY / numMuestras;
  gyroX_offset = (sumaGX / numMuestras) / 131.0;
  
  Serial.println("\n[CALIBRACIÓN] Completada!");
  Serial.print("  Offset Gyro X: ");
  Serial.println(gyroX_offset);
  
  sistemaCalibrado = true;
  tiempoInicio = millis();
  primeraVez = true;
}

//////////////////// FUNCIÓN ANÁLISIS MÉTRICAS /////////////////
void analizarMetricas() {
  // Detectar overshoot
  if (abs(angulo) > abs(anguloMaximo)) {
    anguloMaximo = angulo;
  }
  
  // Acumular error para estacionario
  if (!primeraVez && abs(error) < 2.0) {  // Banda 2%
    sumaError += abs(error);
    contadorError++;
  }
  
  // Detectar establecimiento (criterio 2%)
  if (primeraVez && abs(error) < 1.0) {
    tiempoEstablecido = (millis() - tiempoInicio) / 1000.0;
    primeraVez = false;
    
    // Mostrar métricas
    float overshoot = abs(anguloMaximo) > abs(setpoint) ? 
                      ((anguloMaximo - setpoint) / max(abs(setpoint), 1.0)) * 100 : 0;
    
    Serial.println("\n========== MÉTRICAS PID ==========");
    Serial.print("Ts (establecimiento): ");
    Serial.print(tiempoEstablecido, 2);
    Serial.println(" seg");
    Serial.print("%OS (overshoot): ");
    Serial.print(overshoot, 1);
    Serial.println("%");
    Serial.println("==================================");
  }
}

//////////////////// SETUP //////////////////////
void setup() {
  Serial.begin(115200);
  
  Serial.println("\n  ROBOT AUTO-BALANCÍN PID V3   ");
  
  configurarMotores();
  configurarMPU6050();
  
  Serial.println("\nCOMANDOS DISPONIBLES:");
  Serial.println("  C = Calibrar sensor");
  Serial.println("  P = Iniciar control PID");
  Serial.println("  S = STOP emergencia");
  Serial.println("  + = Aumentar setpoint");
  Serial.println("  - = Reducir setpoint");
  Serial.println("  R = Reporte estado");
  Serial.println("  T = Tuning (ajuste Kp,Ki,Kd)");
  Serial.println("\nEsperando comando...\n");
  
  lastTime = millis();
}

//////////////////// LOOP PRINCIPAL //////////////////////
void loop() {
  // Procesar comandos serial
  if (Serial.available()) {
    procesarComandos();
  }
  
  // Control de emergencia
  if (emergencyStop) {
    detenerMotores();
    return;
  }
  
  // Control PID con tiempo de muestreo
  unsigned long now = millis();
  if ((now - lastTime >= sampleTime) && controlActivo && sistemaCalibrado) {
    float dt = (now - lastTime) / 1000.0;  // Convertir a segundos
    lastTime = now;
    
    // 1) Leer sensor y calcular ángulo
    leerMPU6050Raw();
    angulo = calcularAngulo(dt);
    
    // 2) Verificación de seguridad
    if (abs(angulo) > ANGULO_MAX) {
      emergencyStop = true;
      detenerMotores();
      Serial.println("[EMERGENCIA] Ángulo límite excedido!");
      return;
    }
    
    // 3) Calcular error
    error = setpoint - angulo;
    
    // 4) PID (cálculo manual como el ejemplo)
    P_term = Kp * error;
    
    integral += error * dt;
    I_term = Ki * integral;
    
    derivada = (error - error_prev) / dt;
    D_term = Kd * derivada;
    
    salida = P_term + I_term + D_term;
    
    // 5) Anti-windup (como el ejemplo)
    if (salida > PWM_MAX) {
      salida = PWM_MAX;
      integral -= error * dt;  // Evita acumulación excesiva
    } else if (salida < -PWM_MAX) {
      salida = -PWM_MAX;
      integral -= error * dt;
    }
    
    // 6) Limitar integral independientemente
    integral = constrain(integral, -100, 100);
    
    // 7) Convertir a PWM
    int pwm = (int)salida;
    
    // 8) Aplicar a motores
    moverMotores(pwm);
    
    // 9) Mostrar datos para Serial Plotter
    // Formato: angulo,setpoint,pwm,P,I,D
    Serial.print(angulo);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(pwm/10.0);  // Escalar PWM para visualización
    Serial.print(",");
    Serial.print(P_term);
    Serial.print(",");
    Serial.print(I_term);
    Serial.print(",");
    Serial.println(D_term);
    
    // 10) Analizar métricas
    analizarMetricas();
    
    // 11) Guardar error anterior
    error_prev = error;
    
    // 12) Reporte error estacionario cada 2 segundos
    static unsigned long lastReport = 0;
    if (now - lastReport > 2000 && contadorError > 0) {
      float errorProm = sumaError / contadorError;
      if (errorProm < 2.0 && !primeraVez) {
        Serial.print("[ESTACIONARIO] Error promedio: ");
        Serial.print(errorProm, 2);
        Serial.println("°");
      }
      lastReport = now;
    }
  }
}

//////////////////// PROCESAMIENTO DE COMANDOS /////////////////
void procesarComandos() {
  char cmd = Serial.read();
  
  switch(cmd) {
    case 'C':
    case 'c':
      calibrarSensor();
      break;
      
    case 'P':
    case 'p':
      if (sistemaCalibrado) {
        controlActivo = true;
        emergencyStop = false;
        integral = 0;  // Reset integral
        error_prev = 0;
        Serial.println("[CONTROL] PID activado");
      } else {
        Serial.println("[ERROR] Calibrar primero con 'C'");
      }
      break;
      
    case 'S':
    case 's':
      emergencyStop = !emergencyStop;
      controlActivo = false;
      detenerMotores();
      if (emergencyStop) {
        Serial.println("[STOP] Emergencia activada");
      } else {
        Serial.println("[STOP] Sistema listo");
      }
      break;
      
    case '+':
      setpoint += 1.0;
      Serial.print("[SETPOINT] Nuevo: ");
      Serial.print(setpoint);
      Serial.println("°");
      break;
      
    case '-':
      setpoint -= 1.0;
      Serial.print("[SETPOINT] Nuevo: ");
      Serial.print(setpoint);
      Serial.println("°");
      break;
      
    case 'R':
    case 'r':
      Serial.println("\n===== ESTADO ACTUAL =====");
      Serial.print("Ángulo: ");
      Serial.print(angulo, 2);
      Serial.println("°");
      Serial.print("Setpoint: ");
      Serial.print(setpoint, 2);
      Serial.println("°");
      Serial.print("Error: ");
      Serial.print(error, 2);
      Serial.println("°");
      Serial.print("Kp: ");
      Serial.print(Kp);
      Serial.print(" | Ki: ");
      Serial.print(Ki);
      Serial.print(" | Kd: ");
      Serial.println(Kd);
      Serial.print("Calibrado: ");
      Serial.println(sistemaCalibrado ? "SI" : "NO");
      Serial.print("Control activo: ");
      Serial.println(controlActivo ? "SI" : "NO");
      Serial.println("========================");
      break;
      
    case 'T':
    case 't':
      menuTuning();
      break;
      
    case '?':
      Serial.println("\n=== AYUDA COMANDOS ===");
      Serial.println("C = Calibrar (hacer primero)");
      Serial.println("P = Activar PID");
      Serial.println("S = Stop/Resume");
      Serial.println("+/- = Ajustar setpoint");
      Serial.println("R = Reporte estado");
      Serial.println("T = Tuning PID");
      Serial.println("======================");
      break;
  }
}

//////////////////// MENÚ TUNING INTERACTIVO //////////////////
void menuTuning() {
  Serial.println("\n=== TUNING PID ===");
  Serial.println("Valores actuales:");
  Serial.print("  Kp=");
  Serial.print(Kp);
  Serial.print(" Ki=");
  Serial.print(Ki);
  Serial.print(" Kd=");
  Serial.println(Kd);
  
  Serial.println("\nIngrese nuevos valores:");
  Serial.println("Formato: P20.5 (para Kp=20.5)");
  Serial.println("         I0.8  (para Ki=0.8)");
  Serial.println("         D12.0 (para Kd=12.0)");
  Serial.println("         X     (salir)");
  
  while(true) {
    if (Serial.available()) {
      char tipo = Serial.read();
      if (tipo == 'X' || tipo == 'x') {
        Serial.println("Tuning finalizado");
        break;
      }
      
      float valor = Serial.parseFloat();
      
      switch(tipo) {
        case 'P':
        case 'p':
          Kp = valor;
          Serial.print("Nuevo Kp = ");
          Serial.println(Kp);
          break;
          
        case 'I':
        case 'i':
          Ki = valor;
          integral = 0;  // Reset integral al cambiar Ki
          Serial.print("Nuevo Ki = ");
          Serial.println(Ki);
          break;
          
        case 'D':
        case 'd':
          Kd = valor;
          Serial.print("Nuevo Kd = ");
          Serial.println(Kd);
          break;
      }
    }
  }
  
  Serial.println("\nValores finales:");
  Serial.print("  Kp=");
  Serial.print(Kp);
  Serial.print(" Ki=");
  Serial.print(Ki);
  Serial.print(" Kd=");
  Serial.println(Kd);
}

/*
  =====================================================
  GUÍA RÁPIDA DE AJUSTE PID - MÉTODO ZIEGLER-NICHOLS
  =====================================================
  
  1. AJUSTE INICIAL (Solo P):
     - Poner Ki=0, Kd=0
     - Aumentar Kp hasta oscilación sostenida
     - Anotar: Kp_critico y T_critico (periodo)
  
  2. CÁLCULO PID:
     - Kp = 0.6 * Kp_critico
     - Ki = 2 * Kp / T_critico
     - Kd = Kp * T_critico / 8
  
  3. AJUSTE FINO:
     - Si oscila: reducir Kp, aumentar Kd
     - Si lento: aumentar Kp
     - Si error permanente: aumentar Ki
  
  =====================================================
  VALORES TÍPICOS PARA ROBOT BALANCÍN
  =====================================================
  
  Robot ligero (<500g):
    Kp: 15-25
    Ki: 0.5-1.5
    Kd: 8-12
    
  Robot pesado (>500g):
    Kp: 25-40
    Ki: 1.0-2.5
    Kd: 12-20
  
  =====================================================
  SERIAL PLOTTER
  =====================================================
  
  Abrir Tools -> Serial Plotter (115200 baud)
  Se mostrarán 6 señales:
  1. Ángulo (azul)
  2. Setpoint (rojo)
  3. PWM/10 (verde)
  4. Término P (amarillo)
  5. Término I (cyan)
  6. Término D (magenta)
  
  =====================================================
*/
