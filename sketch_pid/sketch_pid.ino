#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---- LCD I2C ----
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---- APDS-9930 ----
#define APDS_ADDR     0x39
#define APDS_ENABLE   0x80
#define APDS_ATIME    0x81
#define APDS_PTIME    0x82
#define APDS_WTIME    0x83
#define APDS_PPULSE   0x8E
#define APDS_CONTROL  0x8F
#define APDS_ID       0x92
#define APDS_CH0DATAL 0x94
#define APDS_CH1DATAL 0x96
#define APDS_PDATAL   0x9C

uint8_t currentATIME = 0xED;
uint8_t currentAGAIN = 0;
const float againValues[] = {1.0, 8.0, 16.0, 120.0};

// ---- X9C103S ----
const int CS_PIN  = 5;
const int UD_PIN  = 4;
const int INC_PIN = 3;

// ---- LED & Wiper ----
const int LED_PIN   = 9;
const int WIPER_PIN = A0;

// ---- Timer ----
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 800; // ms
int currentPosition = 0;
bool autoPaused = false;

// ==========================================
// ====== REGULATOR PID =====================
// ==========================================
const float referinta_lux = 200.0;

// Coeficientii PID
const float Kp = 0.045;
const float Ki = 0.005;
const float Kd = 0.45;

float suma_eroare = 0;
float eroare_trecuta = 0;

// Anti-Windup
const float limita_int = 50.0; 
const int MAX_STEP = 4; 

float pozitie = 50.0;
float lux_filtrat = 0;
const float alpha = 0.2; // Filtru trece-jos pentru lux
// ==========================================

byte barChar[8] = {
  0b11111, 0b11111, 0b11111, 0b11111,
  0b11111, 0b11111, 0b11111, 0b11111
};

// --- APDS-9930 ---
void apdsWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(APDS_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

uint8_t apdsRead(uint8_t reg) {
  Wire.beginTransmission(APDS_ADDR);
  Wire.write(reg); Wire.endTransmission();
  Wire.requestFrom((uint8_t)APDS_ADDR, (uint8_t)1);
  return Wire.read();
}

uint16_t apdsRead16(uint8_t reg) {
  uint8_t lo = apdsRead(reg);
  uint8_t hi = apdsRead(reg + 1);
  return (hi << 8) | lo;
}

float getALSIT() { return 2.73f * (256 - currentATIME); }

void setAGAIN(uint8_t gain) {
  currentAGAIN = constrain(gain, 0, 3);
  uint8_t ctrl = apdsRead(APDS_CONTROL);
  ctrl = (ctrl & 0xFC) | currentAGAIN;
  apdsWrite(APDS_CONTROL, ctrl);
}

void setATIME(uint8_t atime) {
  currentATIME = atime;
  apdsWrite(APDS_ATIME, atime);
}

void autoRange(uint16_t ch0) {
  if (ch0 >= 65000) {
    if (currentAGAIN > 0) setAGAIN(currentAGAIN - 1);
    else if (currentATIME < 0xFE) setATIME(currentATIME + 10);
  } else if (ch0 < 100 && ch0 > 0) {
    if (currentAGAIN < 3) setAGAIN(currentAGAIN + 1);
    else if (currentATIME > 0xC0) setATIME(currentATIME - 10);
  }
}

bool apdsInit() {
  uint8_t id = apdsRead(APDS_ID);
  if (id != 0x39 && id != 0x12 && id != 0x30) return false;
  
  apdsWrite(APDS_ENABLE, 0x00); delay(10);
  apdsWrite(APDS_ATIME, currentATIME);
  apdsWrite(APDS_PTIME, 0xFF);
  apdsWrite(APDS_WTIME, 0xFF);
  apdsWrite(APDS_PPULSE, 0x08);
  apdsWrite(APDS_CONTROL, 0x20 | currentAGAIN);
  apdsWrite(APDS_ENABLE, 0x0F); delay(12);
  return true;
}

uint16_t getAmbientLight() { return apdsRead16(APDS_CH0DATAL); }
uint16_t getIRLight()      { return apdsRead16(APDS_CH1DATAL); }
uint16_t getProximity()    { return apdsRead16(APDS_PDATAL); }

float calculateLux(uint16_t ch0, uint16_t ch1) {
  if (ch0 >= 65000) return -1;
  float iac1 = (float)ch0 - 1.862f * (float)ch1;
  float iac2 = 0.746f * (float)ch0 - 1.291f * (float)ch1;
  float iac  = max(max(iac1, iac2), 0.0f);
  float lpc  = (0.49f * 52.0f) / (getALSIT() * againValues[currentAGAIN]);
  return iac * lpc;
}

// --- X9C103S ---
void potStep(bool up) {
  digitalWrite(CS_PIN, LOW);
  digitalWrite(UD_PIN, up ? HIGH : LOW); delayMicroseconds(1);
  digitalWrite(INC_PIN, LOW); delayMicroseconds(1);
  digitalWrite(INC_PIN, HIGH); delayMicroseconds(1);
  digitalWrite(CS_PIN, HIGH); delay(1);
}

void potMove(bool up, int steps) {
  for (int i = 0; i < steps; i++) potStep(up);
}

void potReset() {
  potMove(false, 100);
  currentPosition = 0;
}

void potGoTo(int targetPos) {
  targetPos = constrain(targetPos, 0, 99);
  int dif = targetPos - currentPosition;
  if (dif > 0) potMove(true, dif);
  else if (dif < 0) potMove(false, -dif);
  currentPosition = targetPos;
}

// --- Algoritm PID ---
int computePID(float luxCurent) {
  if (luxCurent < 0) luxCurent = 1000;

  float eroare = referinta_lux - luxCurent;

  // Proportional
  float P = Kp * eroare;

  // Integrativ
  suma_eroare += eroare;
  suma_eroare = constrain(suma_eroare, -limita_int, limita_int);
  float I = Ki * suma_eroare;

  // Derivativ
  float derivata_eroare = eroare - eroare_trecuta;
  float D = Kd * derivata_eroare;
  eroare_trecuta = eroare;

  // Aplicam schimbarea pozitiei si limitam cursa
  pozitie += (P + I + D);
  pozitie = constrain(pozitie, 0.0, 99.0);

  return (int)pozitie;
}

// --- Afisare LCD ---
void updateLCD(float lux, int intensitate_LED) {
  // NU mai folosim lcd.clear() aici!
  
  // Rand 1: Lux + descriere
  lcd.setCursor(0, 0);
  if (lux < 0) {
    lcd.print("Lux: SATURAT    "); // Spatii la final pentru a curata textul vechi
  } else {
    lcd.print("Lux: "); 
    lcd.print((int)lux); 
    lcd.print(" lx     "); // Spatii la final
  }

  // Rand 2: LED% (Stanga)
  lcd.setCursor(0, 1);
  lcd.print("LED:"); 
  lcd.print(map(intensitate_LED, 0, 255, 0, 100)); 
  lcd.print("%  "); // Spatii de curatare

  // Rand 2: Pot% (Dreapta)
  lcd.setCursor(8, 1);
  lcd.print("Pot:"); 
  lcd.print(map(currentPosition, 0, 99, 0, 100)); 
  lcd.print("%  "); // Spatii de curatare
}

// --- Citire + reglare ---
void doAutoRead() {
  uint16_t ch0 = getAmbientLight(), ch1 = getIRLight();
  float lux = calculateLux(ch0, ch1);
  
  lux_filtrat = alpha * lux + (1 - alpha) * lux_filtrat;

  // Apelam PID-ul
  int targetPos = computePID(lux_filtrat);
  int dif = targetPos - currentPosition;

  // Limitare rampa
  if (dif > MAX_STEP) dif = MAX_STEP;
  if (dif < -MAX_STEP) dif = -MAX_STEP;

  potGoTo(currentPosition + dif);

  // Citire nivel LED
  int intensitate_LED = map(currentPosition, 0, 99, 0, 255);

  updateLCD(lux, intensitate_LED);

  // Jurnal Serial
  Serial.println("========== PID AUTO ==========");
  Serial.print("  Lux: ");
  if (lux < 0) Serial.println("ERR"); else Serial.println(lux_filtrat, 1);
  Serial.print("  LuxRef: "); Serial.println(referinta_lux);
  Serial.print("  Pot: "); Serial.print(currentPosition); Serial.println("/99");
  Serial.print("  LED: "); Serial.print(map(intensitate_LED, 0, 255, 0, 100)); Serial.println("%");
  Serial.println("==============================");

  autoRange(ch0);
}

// --- Setup & Loop ---
void setup() {
  pinMode(CS_PIN, OUTPUT); pinMode(UD_PIN, OUTPUT);
  pinMode(INC_PIN, OUTPUT); pinMode(LED_PIN, OUTPUT);
  
  analogWrite(LED_PIN, 0);

  digitalWrite(CS_PIN, HIGH); digitalWrite(INC_PIN, HIGH);

  Serial.begin(9600); Wire.begin();
  lcd.init(); lcd.backlight(); lcd.createChar(0, barChar);
  lcd.setCursor(0, 0); lcd.print("Initializare...");

  if (!apdsInit()) {
    lcd.clear(); lcd.print("EROARE APDS!");
    Serial.println("EROARE: APDS-9930!");
  }

  potReset();
  
  // Initializam starile PID-ului cu mediul actual
  uint16_t ch0 = getAmbientLight(), ch1 = getIRLight();
  lux_filtrat = calculateLux(ch0, ch1);
  eroare_trecuta = referinta_lux - lux_filtrat;
  
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Gata! Auto 10s");
  
  lastReadTime = millis();
  delay(2000);
}

void loop() {
  int intensitate_LED = map(currentPosition, 0, 99, 0, 255);
  static float led_filtrat = 0;
  
  led_filtrat = 0.2 * intensitate_LED + 0.8 * led_filtrat;
  analogWrite(LED_PIN, (int)led_filtrat);

  if (!autoPaused && (millis() - lastReadTime >= READ_INTERVAL)) {
    lastReadTime = millis();
    doAutoRead();
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'l') {
      doAutoRead(); lastReadTime = millis();
    } else if (c == 'p') {
      autoPaused = !autoPaused;
      lcd.clear(); lcd.setCursor(0, 0);
      lcd.print(autoPaused ? "AUTO: PAUZA" : "AUTO: ACTIV");
    }
  }
  delay(50);
}