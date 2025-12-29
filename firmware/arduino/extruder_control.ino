/* Sketch atualizado
   - Motor OFF até o bloco atingir 210°C
   - Apos 210°C, motor pode operar em AUTO (motor PID) ou MAN (manual PWM)
   - Serial: "AUTO" / "MAN" / "0-255"
   - Mantidas protecoes e log CSV
*/

#include <PID_v1.h>
#include <SimpleThermistor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// === LCD I2C ===
LiquidCrystal_I2C lcd(0x27, 16, 2);

// === PINOS ===
#define THERMISTOR_PIN A3
#define PHOTO_PIN A7
#define HEATER_PIN 11
#define LED_PIN 4
#define MOTOR_PWM 5
#define MOTOR_DIR 6

// === SENSOR DE TEMPERATURA ===
SimpleThermistor sensor = SimpleThermistor(THERMISTOR_PIN, 100000, 3950, 10000);

// === PID do bloco (ja existente) ===
double Setpoint = 25.0;
double Input, Output;
double Kp = 5.0, Ki = 0.2, Kd = 10.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Temporizacao
unsigned long lastUpdate = 0;
const unsigned long interval = 500;

// === Rampa ===
double TargetTemp = 220.0;
double rampSetpoint = 25.0;
unsigned long lastRamp = 0;
const unsigned long rampInterval = 2000;
const double rampStep = 5.0;

// Ultimos valores LCD
float lastTemp = -100.0;
float lastDiameter = -1.0;

// === Calibracao termistor ===
float calibTable[][2] = {
  {25.3, 24.8}, {60.8, 50.8}, {123.8, 100.9},
  {174.7, 122.2}, {200.8, 153.6}, {227.5, 175.6},
  {252.6, 191.2}, {262.0, 200.0}, {290.0, 225.0},
  {318.0, 250.0}
};
int nCalib = sizeof(calibTable) / sizeof(calibTable[0]);

const double maxBlockTemp = 249.0;
const double pidOffset = 5.0;

// Filtro exponencial
float filtTemp = 25.0;
const float alpha = 0.20;

// Motor state (hardware/manual)
bool motorLigado = false;
int motorPWMValue = 80;   // valor manual por serial (0..255)

// === Tabela de calibração fototransistor (ADC -> mm) ===
float lightCalib[][2] = {
  {367.32, 0.00},
  {256.02, 1.00},
  {206.48, 1.40},
  {202.92, 1.75},
  {170.03, 1.90}
};
int nLightCalib = 5;

// ================= Motor PID (malha fechada para diametro) =================
double DiameterSetpoint = 1.75;   // objetivo (mm)
double DiameterInput = 0.0;       // input (mm) para motor PID
double MotorOutput = 0.0;         // PID output -> 0..255

// Ganhos iniciais (ajustar em bancada)
double mKp = 100.0;
double mKi = 0.5;
double mKd = 10.0;
PID motorPID(&DiameterInput, &MotorOutput, &DiameterSetpoint, mKp, mKi, mKd, DIRECT);

// Motor control params
const int motorPWMMin = 40;   // pwm minimo que efetivamente gira motor
const int motorPWMMax = 255;
bool motorAutoMode = true;    // true = auto (PID), false = manual
unsigned long lastMotorCmd = 0;

// Temperaturas de acionamento (requisito seu)
const double motorOnTemp  = 200.0; // ligar/permitir motor quando bloco >= 200C
const double motorOffTemp = 199.0; // desligar quando bloco < 199C (histerese)

// Media movel para diâmetro
const int DIAM_SAMPLES = 8;
float diamBuffer[DIAM_SAMPLES];
int diamBufIdx = 0;
int diamCount = 0;

// Stall detection / timeouts
unsigned long stallStart = 0;
int lastADCread = -1;
unsigned long lastValidDiameterMs = 0;

// ================= Funcoes utilitarias =================
float interpTable(float table[][2], int n, float x) {
  if (n <= 0) return x;
  float xs[20], ys[20];
  for (int i = 0; i < n; i++) { xs[i] = table[i][0]; ys[i] = table[i][1]; }
  // sort ascending xs
  for (int i = 0; i < n-1; i++) {
    for (int j = 0; j < n-1-i; j++) {
      if (xs[j] > xs[j+1]) {
        float tx = xs[j]; xs[j] = xs[j+1]; xs[j+1] = tx;
        float ty = ys[j]; ys[j] = ys[j+1]; ys[j+1] = ty;
      }
    }
  }
  if (x <= xs[0]) return ys[0];
  if (x >= xs[n-1]) return ys[n-1];
  for (int i = 0; i < n-1; i++) {
    if (x >= xs[i] && x <= xs[i+1]) {
      float t = (x - xs[i]) / (xs[i+1] - xs[i]);
      return ys[i] + t * (ys[i+1] - ys[i]);
    }
  }
  return ys[n-1];
}

float adcToDiameter(float adc) {
  return interpTable(lightCalib, nLightCalib, adc);
}

float corrigirTemperatura(float tSensor) {
  for (int i = 0; i < nCalib - 1; i++) {
    float x0 = calibTable[i][0];
    float y0 = calibTable[i][1];
    float x1 = calibTable[i+1][0];
    float y1 = calibTable[i+1][1];
    if (tSensor >= x0 && tSensor <= x1) {
      return y0 + (tSensor - x0) * (y1 - y0) / (x1 - x0);
    }
  }
  if (tSensor < calibTable[0][0]) return calibTable[0][1];
  if (tSensor > calibTable[nCalib-1][0]) return calibTable[nCalib-1][1];
  return tSensor;
}

void applyMotorOutput(int out) {
  int pwm = constrain(out, 0, motorPWMMax);
  if (pwm < motorPWMMin) pwm = 0; // evita aplicar pwm que nao movera motor
  analogWrite(MOTOR_PWM, pwm);
  if (pwm > 0) digitalWrite(MOTOR_DIR, LOW);
  motorPWMValue = pwm;
  motorLigado = (pwm > 0);
  lastMotorCmd = millis();
}

// ================= Setup =================
void setup() {
  Serial.begin(9600);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PHOTO_PIN, INPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetTunings(Kp, Ki, Kd);

  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(0, 255);
  motorPID.SetTunings(mKp, mKi, mKd);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.setCursor(0, 1);
  lcd.print("Diam:");

  analogWrite(HEATER_PIN, 0);
  applyMotorOutput(0); // garante motor parado

  // inicializa rampSetpoint a partir da leitura real do bloco (se valida)
  float startTemp = sensor.getTempC();
  if (!isnan(startTemp) && startTemp > -50 && startTemp < 400) rampSetpoint = startTemp;
  else rampSetpoint = 25.0;

  filtTemp = rampSetpoint;
  Input = filtTemp;

  for (int i = 0; i < DIAM_SAMPLES; i++) diamBuffer[i] = 0.0;

  Serial.println("reported;corrected;Setpoint;PWM;Light;Diameter;Motor");
  Serial.print("Rampa iniciada em: ");
  Serial.println(rampSetpoint);
}

// ================= Loop =================
void loop() {
  unsigned long now = millis();

  // rampa
  if (now - lastRamp >= rampInterval && rampSetpoint < TargetTemp) {
    lastRamp = now;
    rampSetpoint += rampStep;
    if (rampSetpoint > TargetTemp) rampSetpoint = TargetTemp;
  }

  if (now - lastUpdate < interval) return;
  lastUpdate = now;

  // leitura termistor
  float reported = sensor.getTempC();
  if (isnan(reported) || reported <= -200 || reported > 1000) {
    analogWrite(HEATER_PIN, 0);
    digitalWrite(LED_PIN, LOW);
    applyMotorOutput(0);
    Serial.println("WARN: leitura do sensor invalida, desligando sistemas.");
    return;
  }

  double corrected = corrigirTemperatura(reported);
  filtTemp = alpha * corrected + (1.0 - alpha) * filtTemp;
  Input = filtTemp;

  // respeita limite de seguranca do bloco
  double targetSetpoint = rampSetpoint;
  if (Input + pidOffset >= maxBlockTemp) targetSetpoint = min(targetSetpoint, maxBlockTemp - pidOffset);
  Setpoint = targetSetpoint;

  myPID.Compute();

  int pwmOutput = (int)Output;
  if (Input >= maxBlockTemp - 10 && pwmOutput > 120) pwmOutput = 120;
  if (Input >= maxBlockTemp) pwmOutput = 0;

  analogWrite(HEATER_PIN, pwmOutput);
  digitalWrite(LED_PIN, pwmOutput > 5);

  // leitura do fototransistor e conversao
  int lightValue = analogRead(PHOTO_PIN);
  float estimatedDiameterRaw = adcToDiameter((float)lightValue);
  estimatedDiameterRaw = constrain(estimatedDiameterRaw, 0.0, 5.0);

  // media movel
  diamBuffer[diamBufIdx] = estimatedDiameterRaw;
  diamBufIdx = (diamBufIdx + 1) % DIAM_SAMPLES;
  if (diamCount < DIAM_SAMPLES) diamCount++;
  float diamSum = 0.0;
  for (int i = 0; i < diamCount; i++) diamSum += diamBuffer[i];
  float estimatedDiameter = diamSum / diamCount;

  // atualiza entrada do motor PID
  DiameterInput = estimatedDiameter;

  // === Stall detection ===
// Apenas ativa se estiver no modo AUTO.
// No modo MANUAL, o usuário está no controle, então não desligamos o motor.
if (motorAutoMode) {
    if (lastADCread == lightValue) {
        if (stallStart == 0) stallStart = now;
        else if (now - stallStart > 3000) {
            applyMotorOutput(0);
            Serial.println("WARN: possivel bloqueio do filamento - motor desligado (AUTO).");
        }
    } else {
        stallStart = 0;
    }
} else {
    // MANUAL → sem stall detection, motor continua ligado
    stallStart = 0;  // opcional: apenas reseta contagem
}

  lastADCread = lightValue;
  lastValidDiameterMs = now;

  // ---- CONTROLE DO MOTOR: motor OFF se temp < motorOnTemp
  if (Input >= motorOnTemp) {
    // temperatura atingida: permitir operacao
    if (motorAutoMode) {
      // AUTO: PID controla o motor
      if (!isnan(DiameterInput) && DiameterInput > 0.0) {
        motorPID.Compute();
        applyMotorOutput((int)MotorOutput);
      } else {
        applyMotorOutput(0);
      }
    } else {
      // MANUAL: aplicar PWM manual configurado
      applyMotorOutput(motorPWMValue);
    }
  } else {
    // abaixo da temperatura minima: motor forcado OFF
    if (motorLigado) {
      Serial.println("INFO: abaixo de 210C - motor desligado por seguranca.");
    }
    applyMotorOutput(0);
  }

  // LCD
  if (abs(Input - lastTemp) >= 0.1) {
    lcd.setCursor(6, 0);
    lcd.print(Input, 1);
    lcd.print((char)223);
    lcd.print("C ");
    lastTemp = Input;
  }

  if (abs(estimatedDiameter - lastDiameter) >= 0.01) {
    lcd.setCursor(6, 1);
    lcd.print("    ");
    lcd.setCursor(6, 1);
    lcd.print(estimatedDiameter, 2);
    lcd.print("mm ");
    lastDiameter = estimatedDiameter;
  }

  // CSV Serial
  Serial.print(reported, 2); Serial.print(";");
  Serial.print(Input, 2); Serial.print(";");
  Serial.print(Setpoint, 2); Serial.print(";");
  Serial.print(pwmOutput); Serial.print(";");
  Serial.print(lightValue); Serial.print(";");
  Serial.print(estimatedDiameter, 2); Serial.print(";");
  Serial.println(motorLigado ? 1 : 0);

  // Serial commands: AUTO / MAN / numeric PWM
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() == 0) return;
    if (s.equalsIgnoreCase("AUTO")) {
      motorAutoMode = true;
      Serial.println("Motor: AUTO mode");
    } else if (s.equalsIgnoreCase("MAN")) {
      motorAutoMode = false;
      Serial.println("Motor: MANUAL mode");
    } else {
      int v = s.toInt();
      if (v >= 0 && v <= 255) {
        motorPWMValue = v;
        if (!motorAutoMode) {
          applyMotorOutput(motorPWMValue);
        }
        Serial.print("Manual PWM set: ");
        Serial.println(motorPWMValue);
      } else {
        Serial.println("WARN: comando invalido. Use AUTO, MAN ou 0-255.");
      }
    }
  }
}
