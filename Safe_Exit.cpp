#include <Arduino.h>
#include <Servo.h>
#define TRIG_STANG 7
#define ECHO_STANG 6
#define TRIG_DREPT 5
#define ECHO_DREPT 4
#define BUTON_STANG 2
#define BUTON_DREPT 3
#define LED_STANG 8
#define LED_DREPT 9
#define BUZZER 10
#define SERVO_BLOCATOR_STANG 11
#define SERVO_BLOCATOR_DREPT 12
#define BUTON_USA_STANG A0
#define BUTON_USA_DREPT A1
#define LED_INTERIOR_STANG 13
#define LED_INTERIOR_DREPT A2
#define CLANTA_STANG A3
#define CLANTA_DREPT A4
#define LED_ALERTA_STANG A5
#define LED_ALERTA_DREPT A6
const float DISTANTA_PRAG = 50.0;
const unsigned long INTERVAL_SENZOR = 100;
const int MARIME_FILTRU = 5;
const unsigned long INTERVAL_BUZZER = 300;
const unsigned long TIMP_BLOCARE = 2000;
const int POZITIE_BLOCAT = 0;
const int POZITIE_DEBLOCAT = 90;
const unsigned long DEBOUNCE_USA = 200;
const unsigned long DEBOUNCE_CLANTA = 100;
const unsigned long DURATA_ALERTA = 2000;
const unsigned long INTERVAL_FLASH = 200;
float distanteStang[MARIME_FILTRU];
float distanteDrept[MARIME_FILTRU];
int indexStang = 0;
int indexDrept = 0;
unsigned long ultimaCitireStang = 0;
unsigned long ultimaCitireDrept = 50;
float distantaStangFiltrata = 999.0;
float distantaDreptFiltrata = 999.0;
unsigned long ultimaSchimbareBuzzer = 0;
bool buzzerState = false;
Servo servoStang;
Servo servoDrept;
bool usaStangBloata = false;
bool usaDreptBloata = false;
bool usaStangInProces = false;
bool usaDreptInProces = false;
unsigned long startBlocareStang = 0;
unsigned long startBlocareDrept = 0;
unsigned long ultimaApasareStang = 0;
unsigned long ultimaApasareDrept = 0;
unsigned long ultimaApasareClantaStang = 0;
unsigned long ultimaApasareClantaDrept = 0;
unsigned long startAlertaStang = 0;
unsigned long startAlertaDrept = 0;
bool alertaStangActiva = false;
bool alertaDreptActiva = false;
unsigned long ultimFlashStang = 0;
unsigned long ultimFlashDrept = 0;
bool stareFlashStang = false;
bool stareFlashDrept = false;
enum StareLaterala {
  SIGUR,
  OBIECT_DETECTAT,
  AVERTISMENT_ACTIV
};
StareLaterala stareStang = SIGUR;
StareLaterala stareDrept = SIGUR;
float citesteUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long durata = pulseIn(echoPin, HIGH, 30000);
  if (durata == 0) return 999.0;
  return durata * 0.034 / 2;
}
float calculeazaMedie(float* arr) {
  float suma = 0;
  for (int i = 0; i < MARIME_FILTRU; i++) suma += arr[i];
  return suma / MARIME_FILTRU;
}
void actualizeazaStare(StareLaterala &stare, bool obiectDetectat, bool butonApasat) {
  switch (stare) {
    case SIGUR:
      if (obiectDetectat) stare = OBIECT_DETECTAT;
      break;
    case OBIECT_DETECTAT:
      if (!obiectDetectat) stare = SIGUR;
      else if (butonApasat) stare = AVERTISMENT_ACTIV;
      break;
    case AVERTISMENT_ACTIV:
      if (!obiectDetectat) stare = SIGUR;
      else if (!butonApasat) stare = OBIECT_DETECTAT;
      break;
  }
}
void blocheazaUsa(Servo &servo, bool &usaBloata, bool &usaInProces, unsigned long &startBlocare, int pozitieTinta) {
  if (!usaInProces) {
    usaInProces = true;
    startBlocare = millis();
  }
}
void actualizeazaBlocare(Servo &servo, bool &usaBloata, bool &usaInProces, unsigned long &startBlocare, int pozitieTinta) {
  if (!usaInProces) return;
  unsigned long elapsed = millis() - startBlocare; 
  if (elapsed >= TIMP_BLOCARE) {
    servo.write(pozitieTinta);
    usaBloata = (pozitieTinta == POZITIE_BLOCAT);
    usaInProces = false;
  } else {
    int startPos = usaBloata ? POZITIE_BLOCAT : POZITIE_DEBLOCAT;
    int progres = map(elapsed, 0, TIMP_BLOCARE, startPos, pozitieTinta);
    progres = constrain(progres, 0, 180);
    servo.write(progres);
  }
}
void activeazaAlerta(bool &alertaActiva, unsigned long &startAlerta, bool pericol) {
  if (pericol && !alertaActiva) {
    alertaActiva = true;
    startAlerta = millis();
  }
}
void actualizeazaAlerta(unsigned long acum, bool &alertaActiva, unsigned long &startAlerta, 
                        unsigned long &ultimFlash, bool &stareFlash, byte pinLED, 
                        unsigned long durataAlerta, unsigned long intervalFlash) {
  if (!alertaActiva) {
    if (stareFlash) {
      digitalWrite(pinLED, LOW);
      stareFlash = false;
    }
    return;
  }
  if (acum - startAlerta >= durataAlerta) {
    alertaActiva = false;
    digitalWrite(pinLED, LOW);
    stareFlash = false;
    return;
  } 
  if (acum - ultimFlash >= intervalFlash) {
    ultimFlash = acum;
    stareFlash = !stareFlash;
    digitalWrite(pinLED, stareFlash ? HIGH : LOW);
  }
}
void setup() {
  pinMode(TRIG_STANG, OUTPUT);
  pinMode(ECHO_STANG, INPUT);
  pinMode(TRIG_DREPT, OUTPUT);
  pinMode(ECHO_DREPT, INPUT);
  pinMode(BUTON_STANG, INPUT_PULLUP);
  pinMode(BUTON_DREPT, INPUT_PULLUP);
  pinMode(LED_STANG, OUTPUT);
  pinMode(LED_DREPT, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BUTON_USA_STANG, INPUT_PULLUP);
  pinMode(BUTON_USA_DREPT, INPUT_PULLUP);
  pinMode(LED_INTERIOR_STANG, OUTPUT);
  pinMode(LED_INTERIOR_DREPT, OUTPUT);
  pinMode(CLANTA_STANG, INPUT_PULLUP);
  pinMode(CLANTA_DREPT, INPUT_PULLUP);
  pinMode(LED_ALERTA_STANG, OUTPUT);
  pinMode(LED_ALERTA_DREPT, OUTPUT); 
  servoStang.attach(SERVO_BLOCATOR_STANG);
  servoDrept.attach(SERVO_BLOCATOR_DREPT);
  servoStang.write(POZITIE_DEBLOCAT);
  servoDrept.write(POZITIE_DEBLOCAT);  
  for (int i = 0; i < MARIME_FILTRU; i++) {
    distanteStang[i] = 999.0;
    distanteDrept[i] = 999.0;
  }
}
void loop() {
  unsigned long acum = millis();
  if (acum - ultimaCitireStang >= INTERVAL_SENZOR) {
    ultimaCitireStang = acum;
    distanteStang[indexStang] = citesteUltrasonic(TRIG_STANG, ECHO_STANG);
    distantaStangFiltrata = calculeazaMedie(distanteStang);
    indexStang = (indexStang + 1) % MARIME_FILTRU;
  } 
  if (acum - ultimaCitireDrept >= INTERVAL_SENZOR) {
    ultimaCitireDrept = acum;
    distanteDrept[indexDrept] = citesteUltrasonic(TRIG_DREPT, ECHO_DREPT);
    distantaDreptFiltrata = calculeazaMedie(distanteDrept);
    indexDrept = (indexDrept + 1) % MARIME_FILTRU;
  }  
  actualizeazaStare(stareStang, distantaStangFiltrata < DISTANTA_PRAG, digitalRead(BUTON_STANG) == LOW);
  actualizeazaStare(stareDrept, distantaDreptFiltrata < DISTANTA_PRAG, digitalRead(BUTON_DREPT) == LOW);  
  digitalWrite(LED_STANG, (stareStang != SIGUR) ? HIGH : LOW);
  digitalWrite(LED_DREPT, (stareDrept != SIGUR) ? HIGH : LOW); 
  if (stareStang == AVERTISMENT_ACTIV || stareDrept == AVERTISMENT_ACTIV || 
      alertaStangActiva || alertaDreptActiva) {
    if (acum - ultimaSchimbareBuzzer >= INTERVAL_BUZZER) {
      ultimaSchimbareBuzzer = acum;
      buzzerState = !buzzerState;
      digitalWrite(BUZZER, buzzerState);
    }
  } else {
    if (buzzerState) {
      digitalWrite(BUZZER, LOW);
      buzzerState = false;
    }
  } 
  if (digitalRead(BUTON_USA_STANG) == LOW && (acum - ultimaApasareStang) > DEBOUNCE_USA) {
    ultimaApasareStang = acum;  
    if (stareStang == AVERTISMENT_ACTIV || distantaStangFiltrata < DISTANTA_PRAG) {
      if (!usaStangBloata && !usaStangInProces) {
        blocheazaUsa(servoStang, usaStangBloata, usaStangInProces, startBlocareStang, POZITIE_BLOCAT);
        digitalWrite(LED_INTERIOR_STANG, HIGH);
      }
    } else {
      if (usaStangBloata && !usaStangInProces) {
        blocheazaUsa(servoStang, usaStangBloata, usaStangInProces, startBlocareStang, POZITIE_DEBLOCAT);
        digitalWrite(LED_INTERIOR_STANG, LOW);
      }
    }
  }
  
  if (digitalRead(BUTON_USA_DREPT) == LOW && (acum - ultimaApasareDrept) > DEBOUNCE_USA) {
    ultimaApasareDrept = acum;    
    if (stareDrept == AVERTISMENT_ACTIV || distantaDreptFiltrata < DISTANTA_PRAG) {
      if (!usaDreptBloata && !usaDreptInProces) {
        blocheazaUsa(servoDrept, usaDreptBloata, usaDreptInProces, startBlocareDrept, POZITIE_BLOCAT);
        digitalWrite(LED_INTERIOR_DREPT, HIGH);
      }
    } else {
      if (usaDreptBloata && !usaDreptInProces) {
        blocheazaUsa(servoDrept, usaDreptBloata, usaDreptInProces, startBlocareDrept, POZITIE_DEBLOCAT);
        digitalWrite(LED_INTERIOR_DREPT, LOW);
      }
    }
  } 
  if (digitalRead(CLANTA_STANG) == LOW && (acum - ultimaApasareClantaStang) > DEBOUNCE_CLANTA) {
    ultimaApasareClantaStang = acum;
    bool pericol = (stareStang == AVERTISMENT_ACTIV || distantaStangFiltrata < DISTANTA_PRAG);
    activeazaAlerta(alertaStangActiva, startAlertaStang, pericol);
  }  
  if (digitalRead(CLANTA_DREPT) == LOW && (acum - ultimaApasareClantaDrept) > DEBOUNCE_CLANTA) {
    ultimaApasareClantaDrept = acum;
    bool pericol = (stareDrept == AVERTISMENT_ACTIV || distantaDreptFiltrata < DISTANTA_PRAG);
    activeazaAlerta(alertaDreptActiva, startAlertaDrept, pericol);
  }  
  actualizeazaAlerta(acum, alertaStangActiva, startAlertaStang, ultimFlashStang, 
                     stareFlashStang, LED_ALERTA_STANG, DURATA_ALERTA, INTERVAL_FLASH);
  actualizeazaAlerta(acum, alertaDreptActiva, startAlertaDrept, ultimFlashDrept, 
                     stareFlashDrept, LED_ALERTA_DREPT, DURATA_ALERTA, INTERVAL_FLASH);
  actualizeazaBlocare(servoStang, usaStangBloata, usaStangInProces, startBlocareStang, 
                      usaStangBloata ? POZITIE_DEBLOCAT : POZITIE_BLOCAT);
  actualizeazaBlocare(servoDrept, usaDreptBloata, usaDreptInProces, startBlocareDrept, 
                      usaDreptBloata ? POZITIE_DEBLOCAT : POZITIE_BLOCAT);
}