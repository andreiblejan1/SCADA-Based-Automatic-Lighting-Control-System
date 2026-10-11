#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---- LCD I2C ----
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---- APDS-9930 ----
#define APDS_ADDR     0x39 // adresa senzorului pe magistrala
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

uint8_t currentATIME = 0xED; // 51 ms de captura a luminii
uint8_t currentAGAIN = 0; // treapta de sensibilitate (din datasheet)
const float againValues[] = {1.0, 8.0, 16.0, 120.0}; // valorile in functie de treapta

// ---- X9C103S ----
const int CS_PIN  = 5; // pin D5
const int UD_PIN  = 4; // pin D4
const int INC_PIN = 3; // pin D3

// ---- LED & Wiper ----
const int LED_PIN   = 9; // pin D9 PWM
const int WIPER_PIN = A0; // citirea tensiunii de la potentiometru

// ---- Timer ----
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 800; // ms perioada de citire si afisare date
int currentPosition = 0; // treapta initiala a potentiometrului
bool autoPaused = false;

// ==========================================
// ====== REGULATOR PID ADAPTIV =============
// ==========================================
const float referinta_lux = 200.0;

float Kp, Ki, Kd;

// Mod CONSERVATOR
const float Kp_cons = 0.045;
const float Ki_cons = 0.016;
const float Kd_cons = 0.690;

// Mod AGRESIV
const float Kp_ag = 0.095;
const float Ki_ag = 0.003;
const float Kd_ag = 0.470;

const float prag_comutare = 40.0; 

// Variabile PID
float suma_eroare = 0;
float eroare_trecuta = 0;
const float limita_integrare = 20.0; 

const int MAX_STEP = 5; // variatia maxima a treptelor potentiometrului la o iteratie
bool mod = false; // pentru afisare

// variabila_comanda reprezinta cererea ideala a PID-ului, nu pozitia hardware
float variabila_comanda = 50.0; 
float lux_filtrat = 0;
const float alpha = 0.3; // coeficient de filtrare pe iesire

// ==========================================

byte barChar[8] = {
  0b11111, 0b11111, 0b11111, 0b11111,
  0b11111, 0b11111, 0b11111, 0b11111
};

// ==========================================
// --- APDS-9930 ---
// ==========================================

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

// ==========================================
// --- X9C103S ---
// ==========================================

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

// ==========================================

// ==========================================
// --- Algoritm PID Adaptiv  ---
// ==========================================

float legeReglare(float lux_curent) {
  if (lux_curent < 0) lux_curent = 1000; // in caz de saturatie a senzorului

  float eroare = referinta_lux - lux_curent;
  float abs_eroare = abs(eroare);

  // Zona moarta pentru eroare
  // Previne actiunea potentiometrului digital cand eroarea este mai mica decat 1.5 lux
  if (abs_eroare < 1.5) {
    eroare = 0.0;
  }

  // Gain scheduling
  // Calculam un factor de tranzitie intre 0 (conservator) si 1 (agresiv)
  float factor_adaptiv = abs_eroare / prag_comutare;
  if (factor_adaptiv > 1.0) factor_adaptiv = 1.0;

  // Combinam ambele moduri de reglare in functie de marimea erorii
  Kp = Kp_cons + factor_adaptiv * (Kp_ag - Kp_cons);
  Ki = Ki_cons + factor_adaptiv * (Ki_ag - Ki_cons);
  Kd = Kd_cons + factor_adaptiv * (Kd_ag - Kd_cons);

  mod = (abs_eroare > prag_comutare); 

  // Proportional
  float P = Kp * eroare;

  // Integrativ
  suma_eroare += eroare;
  suma_eroare = constrain(suma_eroare, -limita_integrare, limita_integrare); // anti-windup simplu
  float I = Ki * suma_eroare;

  // Derivativ
  float derivata_eroare = eroare - eroare_trecuta;
  float D = Kd * derivata_eroare;
  eroare_trecuta = eroare;

  // -Actualizare comanda
  variabila_comanda += (P + I + D);
  variabila_comanda = constrain(variabila_comanda, 0.0, 99.0);

  return variabila_comanda; // variabila de comanda (0-99 trepte)
}

// ==========================================
// --- COMPENSARE NELINIARITATI ---
// ==========================================

// Aceasta sectiune de cod face legatura intre domeniul matematic liniar al PID-ului
// si domeniul fizic neliniar al LED-ului comandat de potentiometrul digital
// Transformarea prin exponentiala compenseaza faptul ca LED-ul tinde sa fie mult prea luminos la inceputul cursei (x << 1)

int compensareNeliniaritati(float u) {
  if (u < 0.5) return 0; // daca comanda este apropiata de 0, sarim peste compensare si dam comanda nula

  const float prag = 5.0; // treapta hardware minima de la care LED-ul incepe sa lumineze (aleasa empiric)

  const float gamma = 1.5; // coeficient de curbare

  float u_normat = u / 99.0; // normarea variabilei de comanda
  float u_nou = pow(u_normat, gamma); // aplicam curba y = x^gamma
  
  float comanda_noua = prag + (u_nou * (99.0 - prag)); // mapare inapoi pe treptele potentiometrului

  return constrain((int)comanda_noua, 0, 99);
}
// ==========================================

// ==========================================
// --- Afisare LCD ---
// ==========================================

void updateLCD(float lux, int intensitate_LED) {
  lcd.setCursor(0, 0);
  if (lux < 0) {
    lcd.print("Lux: SATURAT    "); 
  } else {
    lcd.print("Lux: "); lcd.print((int)lux); lcd.print(" lx     "); 
  }

  lcd.setCursor(0, 1);
  lcd.print("LED:"); lcd.print(map(intensitate_LED, 0, 255, 0, 100)); lcd.print("%  "); 

  lcd.setCursor(8, 1);
  lcd.print("Pot:"); lcd.print(map(currentPosition, 0, 99, 0, 100)); lcd.print("%  "); 
}

// ==========================================
// --- Citire + reglare ---
// ==========================================

void doAutoRead() {
  uint16_t ch0 = getAmbientLight(), ch1 = getIRLight();
  float lux = calculateLux(ch0, ch1);
   
  lux_filtrat = alpha * lux + (1 - alpha) * lux_filtrat;

  // PID-ul calculeaza variabila de comanda (liniara)
  float u_pid = legeReglare(lux_filtrat);

  // Trecem variabila prin filtrul neliniar pentru a afla comanda hardware
  int targetPos = compensareNeliniaritati(u_pid);

  int dif = targetPos - currentPosition;

  if (dif > MAX_STEP) dif = MAX_STEP;
  if (dif < -MAX_STEP) dif = -MAX_STEP;

  potGoTo(currentPosition + dif);

  int intensitate_LED = map(currentPosition, 0, 99, 0, 255); // calculeaza lumina emisa de LED in functie de treapta potentiometrului (% din maxim)

  updateLCD(lux, intensitate_LED);

  Serial.println("====== PID ADAPTIV CU COMPENSARE NELINIARITATI ======");
  Serial.print("  Lux (Filtrat): ");
  if (lux < 0) Serial.println("SATURAT"); else Serial.println(lux_filtrat, 1);
  Serial.print("  Mod adaptiv: "); Serial.println(mod ? "AGRESIV" : "CONSERVATOR");
  Serial.print("  LED: "); Serial.print(map(intensitate_LED, 0, 255, 0, 100)); Serial.println("%");
  Serial.print("  Potentiometru: "); Serial.print(currentPosition); Serial.println("/99");
  Serial.println("=======================================");

  autoRange(ch0);
}

// ==========================================
// --- Setup + Loop ---
// ==========================================
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
  
  // Variabile initiale
  uint16_t ch0 = getAmbientLight(), ch1 = getIRLight();
  lux_filtrat = calculateLux(ch0, ch1);
  eroare_trecuta = referinta_lux - lux_filtrat;
  
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Gata! Auto");
  
  lastReadTime = millis();
  delay(2000);
}

void loop() {
  // Calculam luminozitatea din pozitia curenta a potentiometrului, eliminand zgomotul senzorului
  int intensitate_LED = map(currentPosition, 0, 99, 0, 255);
  analogWrite(LED_PIN, intensitate_LED);

  // Apelare afisare date
  if (!autoPaused && (millis() - lastReadTime >= READ_INTERVAL)) {
    lastReadTime = millis();
    doAutoRead();
  }

  // Definim doua comenzi input pe serial: fortare citire si pauza afisare
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