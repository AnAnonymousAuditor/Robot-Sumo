#include <Arduino.h>
// Robot Sumo con Arduino Uno
// Componentes: HC-SR04 (con Timer2), 2x TCRT5000 (con interrupciones), 2x Servo S3003 modificados
// Estrategia equilibrada: defensa y ataque - TOTALMENTE NO BLOQUEANTE

#include <Servo.h>
#define ADD(x,y) ((x) + (y))

// === PINES ===
// Ultrasonido HC-SR04
#define TRIG_PIN 12
#define ECHO_PIN 11

// Sensores TCRT5000 en pines de interrupción
#define SENSOR_DELANTERO_PIN 2  // INT0 - Sensor ADELANTE del robot
#define SENSOR_TRASERO_PIN 4    // INT1 - Sensor ATRÁS del robot

// Servos (usar pines PWM)
#define SERVO_IZQ_PIN 10   // PWM
#define SERVO_DER_PIN 9  // PWM

// === CONSTANTES ===
#define DISTANCIA_ATAQUE 40      // cm - atacar si oponente está cerca
#define DISTANCIA_BUSQUEDA 80    // cm - buscar activamente
#define TIEMPO_RETROCESO 400     // ms - tiempo para alejarse del borde
#define TIEMPO_GIRO_BORDE 350    // ms - giro después de detectar borde

// Velocidades servos (AJUSTAR SEGÚN PRUEBAS)
#define VELOCIDAD_STOP_DER 90
#define VELOCIDAD_STOP_IZQ 93
#define VELOCIDAD_BASE 40       // Velocidad base para pruebas
#define VELOCIDAD_BASE_IZQ ADD(VELOCIDAD_STOP_IZQ,VELOCIDAD_BASE)       // Velocidad base para pruebas
#define VELOCIDAD_BASE_DER ADD(VELOCIDAD_STOP_DER,VELOCIDAD_BASE)       // Velocidad base para pruebas
#define VELOCIDAD_TURBO 25      // Velocidad máxima de ataque

// === VARIABLES GLOBALES ===
// Sensores de línea
volatile bool lineaDelDetectada = false;
volatile bool lineaTraDetectada = false;

// Ultrasonido no bloqueante
volatile int distanciaActual = 0;
volatile unsigned long tiempoEchoInicio = 0;
volatile bool esperandoEcho = false;
volatile bool medicionCompleta = false;

Servo servoIzq;
Servo servoDer;

// Estados del robot
enum Estado {
  BUSCAR,
  ATACAR,
  EVITAR_BORDE
};
Estado estadoActual = BUSCAR;

// Control de tiempo para búsqueda
unsigned long tiempoGiro = 0;
bool girando = false;

void detectarLineaDel();
void detectarLineaTra();
void manejarBorde();
void echoInterrupt();
void detener();
void avanzar();
void retroceder();
void girarIzquierda();
void girarDerecha();
void atacarTurbo();
void buscarOponente();

void setup() {
  Serial.begin(9600);

  // Configurar pines ultrasonido
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Configurar sensores infrarrojos con pull-up interno
  pinMode(SENSOR_DELANTERO_PIN, INPUT_PULLUP);
  pinMode(SENSOR_TRASERO_PIN, INPUT_PULLUP);

  // Configurar interrupciones para sensores de línea
  attachInterrupt(digitalPinToInterrupt(SENSOR_DELANTERO_PIN), detectarLineaDel, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_TRASERO_PIN), detectarLineaTra, FALLING);

  // Configurar interrupción para ECHO del ultrasonido
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoInterrupt, CHANGE);

  // Configurar Timer2 para lectura periódica del ultrasonido (cada 64ms aprox)
  cli();  // Deshabilitar interrupciones globales
  TCCR2A = 0;  // Modo normal
  TCCR2B = 0;
  TCNT2 = 0;   // Inicializar contador

  // Configurar para ~64ms usando prescaler 1024
  // OCR2A = (16MHz / (prescaler * frecuencia_deseada)) - 1
  // Para ~15.6Hz (64ms): OCR2A = (16000000 / (1024 * 15.6)) - 1 ≈ 249
  OCR2A = 249;
  TCCR2A |= (1 << WGM21);   // Modo CTC
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  // Prescaler 1024
  TIMSK2 |= (1 << OCIE2A);  // Habilitar interrupción por comparación
  sei();  // Habilitar interrupciones globales

  // Inicializar servos en pines PWM
  servoIzq.attach(SERVO_IZQ_PIN);
  servoDer.attach(SERVO_DER_PIN);

  // Espera inicial de 5 segundos (reglamento sumo)
  detener();
  Serial.println("Iniciando en 5 segundos...");
  delay(5000);
  Serial.println("¡PELEA!");
}

void loop() {
  // === PRIORIDAD 1: Evitar caída del dohyo ===
  if (lineaDelDetectada || lineaTraDetectada) {
    estadoActual = EVITAR_BORDE;
    manejarBorde();
    return;
  }

  // === PRIORIDAD 2 y 3: Atacar o Buscar ===
  // La distancia se actualiza automáticamente por interrupciones
  int distancia = distanciaActual;

  if (distancia > 0 && distancia <= DISTANCIA_ATAQUE) {
    // Oponente cerca - ATACAR CON FUERZA
    estadoActual = ATACAR;
    atacarTurbo();
  }
  else if (distancia > DISTANCIA_ATAQUE && distancia <= DISTANCIA_BUSQUEDA) {
    // Oponente detectado pero lejos - avanzar
    estadoActual = ATACAR;
    avanzar();
  }
  else {
    // No hay oponente - buscar girando
    estadoActual = BUSCAR;
    buscarOponente();
  }

  // Loop completamente no bloqueante - sin delays
}

// === INTERRUPCIONES DE SENSORES DE LÍNEA ===
void detectarLineaDel() {
  // Verificar que realmente está en LOW (debounce por hardware)
  if (digitalRead(SENSOR_DELANTERO_PIN) == HIGH) {
    lineaDelDetectada = true;
  }
}

void detectarLineaTra() {
  // Verificar que realmente está en LOW (debounce por hardware)
  if (digitalRead(SENSOR_TRASERO_PIN) == HIGH) {
    lineaTraDetectada = true;
  }
}

// === INTERRUPCIÓN TIMER2 - Trigger del ultrasonido cada 64ms ===
ISR(TIMER2_COMPA_vect) {
  if (!esperandoEcho) {
    // Enviar pulso trigger
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    esperandoEcho = true;
  }
}

// === INTERRUPCIÓN ECHO - Medir tiempo del pulso ===
void echoInterrupt() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    // Inicio del pulso ECHO
    tiempoEchoInicio = micros();
  } else {
    // Fin del pulso ECHO
    if (esperandoEcho && tiempoEchoInicio > 0) {
      unsigned long duracion = micros() - tiempoEchoInicio;

      // Calcular distancia (solo enteros)
      // Limitar a rango razonable (2-400cm)
      if (duracion > 116 && duracion < 23200) {  // 2cm a 400cm
        distanciaActual = duracion / 58;
      } else {
        distanciaActual = 0;  // Fuera de rango
      }

      esperandoEcho = false;
      medicionCompleta = true;
      tiempoEchoInicio = 0;
    }
  }
}

// === MANEJO DE BORDES (usando millis() en vez de delay) ===
void manejarBorde() {
  static unsigned long tiempoInicioBorde = 0;
  static int faseBorde = 0;  // 0=retroceso, 1=giro
  static bool enBorde = false;
  static bool girarHaciaIzq = false;  // Dirección del giro

  if (!enBorde) {
    // Iniciar maniobra de borde - determinar dirección del giro
    enBorde = true;
    faseBorde = 0;
    tiempoInicioBorde = millis();

    // Decidir hacia dónde girar según qué sensor detectó
    if (lineaDelDetectada) {
      Serial.println("¡BORDE DELANTERO!");
      girarHaciaIzq = !girarHaciaIzq;  // Girar a la derecha (alejarse del borde izq)
    }
    else if (lineaTraDetectada) {
      Serial.println("¡BORDE TRASERO!");
      girarHaciaIzq = !girarHaciaIzq;  // Girar a la izquierda (alejarse del borde der)
    }
    else if (lineaDelDetectada && lineaTraDetectada) {
      Serial.println("¡BORDE LATERAL!");
      girarHaciaIzq = !girarHaciaIzq;  // Girar a la derecha
    }
  }

  unsigned long tiempoTranscurrido = millis() - tiempoInicioBorde;

  if (faseBorde == 0) {
    if (lineaDelDetectada) {
      retroceder();
    }
    else if (lineaTraDetectada) {
      avanzar();
    }


    if (tiempoTranscurrido >= TIEMPO_RETROCESO) {
      faseBorde = 1;
      tiempoInicioBorde = millis();
    }
  }
  else if (faseBorde == 1) {
    // Fase de giro - girar según la dirección determinada
    if (girarHaciaIzq) {
      girarIzquierda();
    } else {
      girarDerecha();
    }

    // Determinar tiempo de giro
    int tiempoGiroTotal = TIEMPO_GIRO_BORDE;
    if (lineaDelDetectada) {
      tiempoGiroTotal += 100;  // Giro más largo si detectó borde frontal
    }

    if (tiempoTranscurrido >= tiempoGiroTotal) {
      // Finalizar maniobra
      lineaDelDetectada = false;
      lineaTraDetectada = false;
      enBorde = false;
      faseBorde = 0;
    }
  }
}

// === ESTRATEGIAS DE COMBATE ===
void atacarTurbo() {
  // Ataque máxima velocidad
  servoIzq.write(VELOCIDAD_BASE_IZQ + VELOCIDAD_TURBO);
  servoDer.write(180 - VELOCIDAD_BASE_DER + VELOCIDAD_TURBO);  // Invertido
}

void buscarOponente() {
  // Giro continuo para escanear (sin delays)
  if (!girando) {
    tiempoGiro = millis();
    girando = true;
  }

  girarDerecha();

  // Cambiar dirección cada 1.5 segundos
  if (millis() - tiempoGiro > 1500) {
    girando = false;
  }
}

// === FUNCIONES DE MOVIMIENTO ===
void avanzar() {
  servoIzq.write(VELOCIDAD_BASE_IZQ);
  servoDer.write(180 - VELOCIDAD_BASE_DER);
}

void retroceder() {
  servoIzq.write(180 - VELOCIDAD_BASE_IZQ);
  servoDer.write(VELOCIDAD_BASE_DER);
}

void girarDerecha() {
  servoIzq.write(VELOCIDAD_BASE_IZQ);
  servoDer.write(VELOCIDAD_BASE_DER);
}

void girarIzquierda() {
  servoIzq.write(180 - VELOCIDAD_BASE_IZQ);
  servoDer.write(180 - VELOCIDAD_BASE_DER);
}

void detener() {
  servoIzq.write(VELOCIDAD_STOP_IZQ);
  servoDer.write(VELOCIDAD_STOP_DER);
}
