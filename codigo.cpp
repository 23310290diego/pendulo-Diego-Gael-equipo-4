#include <Arduino.h>
#include <math.h>

// ==========================================
// DEFINICIÓN DE PINES
// ==========================================
const int RPWM_PIN = 25; // Giro Derecha (PWM)
const int LPWM_PIN = 26; // Giro Izquierda (PWM)
const int REN_PIN  = 16; // Habilitar Derecha
const int LEN_PIN  = 17; // Habilitar Izquierda

const int HALL_A_PIN = 27; // Encoder canal A
const int HALL_B_PIN = 14; // Encoder canal B
const int BOTON_BOOT = 0;  // Botón físico BOOT del ESP32

// Pin analógico para el sensor de corriente (ACS712 20A)
const int PIN_SENSOR_CORRIENTE = 32;

// Pin digital para el sensor MH Flying Fish (Home / Cero físico)
const int PIN_HOME_SENSOR = 33;   // Conectar salida DO del sensor al GPIO 33
const int ESTADO_DETECTADO = LOW; // LOW cuando detecta obstáculo (cambiar a HIGH si tu módulo invierte la lógica)

// Configuración de PWM por Hardware en ESP32
const int PWM_Freq = 10000;    // 10 kHz de frecuencia PWM
const int PWM_Res = 8;         // Resolución de 8 bits (0 a 255)

// ==========================================
// VARIABLES GLOBALES DE CONTROL Y ENCODER
// ==========================================
volatile long contadorPulsos = 0; 
long pulsosPorRev = 170;          // Pulsos por vuelta calibrados (170 PPR)
bool calibrado = true;

bool invertirEncoder = true;

int estadoMotor = 0;              
int direccion = 0;                
unsigned long ultimoReporte = 0;  

// Variables adicionales solo para visualizar exactitud de PWM en Teleplot
int valorPWMRealBits = 0;
float valorPerfilExacto = 0.0;

// Variables para el sensor de corriente
float corrienteFiltrada = 0.0;
float voltajeCero = 1.65; // Se auto-calibra en el setup() y con el comando 'c'

// ==========================================
// VARIABLES PARA POSICIÓN Y VELOCIDAD REGULADA
// ==========================================
bool modoPosicion = false;        
bool modoSegundero = false;       
long pulsosObjetivo = 0;          
float gradosObjetivo = 0.0;       
unsigned long ultimoSegundoReloj = 0;
float gradosBaseReloj = 0.0;      
long segundosRelojActual = 0;

// Sentido del modo reloj (-1 invierte el giro anterior para ir en sentido horario)
const int SENTIDO_RELOJ = -1;     

bool enMovimientoHaciaMeta = false;
int sentidoDeViaje = 1;           

// Variable para ajustar la velocidad de giro manualmente (0 a 100%)
int velocidadGiroManual = 55;

// ==========================================
// VARIABLES PARA PERFILES Y TELEPLOT
// ==========================================
int perfilActivoMotor = 1;
unsigned long tiempoInicio = 0;
float periodoCiclo = 8.0;
bool modoGrafica = false;
unsigned long ultimoClickBoton = 0;

float p0 = 20.0;
float A = 50.0;
float T = 4.0;

// ==========================================
// INTERRUPCIÓN DEL ENCODER
// ==========================================
void IRAM_ATTR procesarEncoder() {
bool estadoB = digitalRead(HALL_B_PIN);
if (invertirEncoder) {
    if (estadoB == HIGH) contadorPulsos--;
    else contadorPulsos++;
} else {
    if (estadoB == HIGH) contadorPulsos++;
    else contadorPulsos--;
}
}

// ==========================================
// CONTROL DE MOTOR CON PWM REGULADO (0 - 100%)
// ==========================================
void frenarMotor() {
digitalWrite(REN_PIN, HIGH);
digitalWrite(LEN_PIN, HIGH);
  // Freno dinámico cortocircuitando ambos PWM a 0
ledcWrite(RPWM_PIN, 0);
ledcWrite(LPWM_PIN, 0);
direccion = 0;
estadoMotor = 0;
valorPWMRealBits = 0;
valorPerfilExacto = 0.0;
}

// Recibe velocidad en porcentaje (0 a 100)
void moverMotorPWM(int dir, int velocidadPct) {
digitalWrite(REN_PIN, HIGH);
digitalWrite(LEN_PIN, HIGH);

  // Limita el porcentaje de 0 a 100
velocidadPct = constrain(velocidadPct, 0, 100);

  // Mapea el porcentaje (0-100%) al ciclo de trabajo de 8 bits (0-255)
int valorPWM = map(velocidadPct, 0, 100, 0, 255);

if (dir == 1) {
    ledcWrite(LPWM_PIN, 0);
    ledcWrite(RPWM_PIN, valorPWM);
    direccion = 1;
    estadoMotor = velocidadPct;
    valorPWMRealBits = valorPWM;
} 
else if (dir == -1) {
    ledcWrite(RPWM_PIN, 0);
    ledcWrite(LPWM_PIN, valorPWM);
    direccion = -1;
    estadoMotor = -velocidadPct;
    valorPWMRealBits = -valorPWM;
} 
else {
    frenarMotor();
}
}

// ==========================================
// CALIBRACIÓN DEL PUNTO CERO DE CORRIENTE (0 AMPERES)
// ==========================================
void calibrarCorrienteCero() {
frenarMotor();
  delay(400); // Esperar a que no exista corriente residual del motor
long sumaMV = 0;
const int muestrasCal = 200;
for (int i = 0; i < muestrasCal; i++) {
    sumaMV += analogReadMilliVolts(PIN_SENSOR_CORRIENTE);
    delay(2);
}
  voltajeCero = ((float)sumaMV / (float)muestrasCal) / 1000.0; // Convertir mV a Voltios
corrienteFiltrada = 0.0;
Serial.print(">> [CORRIENTE CALIBRADA] Voltaje base (0A) = ");
Serial.print(voltajeCero, 3);
Serial.println(" V");
}

String leerLinea() {
unsigned long tEspera = millis();
while (Serial.available() == 0) {
    if (millis() - tEspera > 15000) return "";
    delay(10);
}
String linea = Serial.readStringUntil('\n');
linea.trim();
return linea;
}

void imprimirConversionReloj(float gradosTotales) {
  // Ajustamos la visualización para que el sentido del reloj marque positivo de 0 a 360°
  float gradosReloj = gradosTotales * SENTIDO_RELOJ;
float anguloNorm = fmod(gradosReloj, 360.0);
if (anguloNorm < 0) anguloNorm += 360.0;

float totalMinutosReloj = anguloNorm / 6.0;
int minReloj = (int)totalMinutosReloj;
  int segReloj = (int)((totalMinutosReloj - minReloj) * 60.0);

float absGrados = abs(gradosTotales);
int g = (int)absGrados;
  float restoMin = (absGrados - g) * 60.0;
int m = (int)restoMin;
  int s = round((restoMin - m) * 60.0);
if (s == 60) { s = 0; m++; }
if (m == 60) { m = 0; g++; }

Serial.print("   -> Angulo respecto al CERO (5): ");
if (gradosTotales < 0) Serial.print("-");
Serial.print(g); Serial.print("° ");
Serial.print(m); Serial.print("' ");
Serial.print(s); Serial.print("'' (Brujula Horaria 0-360°: ");
Serial.print(anguloNorm, 1); Serial.println("°)");
}

void irAGrados(float gradosMeta) {
gradosObjetivo = gradosMeta;
  pulsosObjetivo = round((gradosObjetivo / 360.0) * (float)pulsosPorRev);

long actual = contadorPulsos;
long errorInicial = pulsosObjetivo - actual;

if (modoSegundero) {
    // Verifica si ya alcanzó o pasó la meta en el sentido horario definido
    if ((errorInicial * SENTIDO_RELOJ) <= 0) {
    enMovimientoHaciaMeta = false;
    frenarMotor();
    return;
    }
    sentidoDeViaje = SENTIDO_RELOJ;
} 
else {
    if (abs(errorInicial) <= 1) {
    enMovimientoHaciaMeta = false;
    frenarMotor();
    return;
    }
    sentidoDeViaje = (errorInicial > 0) ? 1 : -1;
}

enMovimientoHaciaMeta = true;
modoPosicion = true;
}

// ==========================================
// CONTROLADOR DE POSICIÓN CON VELOCIDAD REGULADA (PWM INTELIGENTE)
// ==========================================
void actualizarControlPosicion() {
if (!enMovimientoHaciaMeta) {
    frenarMotor();
    return;
}

long actual = contadorPulsos;
  long pulsosFaltantes = (pulsosObjetivo - actual) * sentidoDeViaje;

  // 1. Condición de llegada exacta
if (pulsosFaltantes <= 0) {
    frenarMotor();
    enMovimientoHaciaMeta = false;
    return;
}

  // 2. CONTROL DE VELOCIDAD POR ZONAS (RAMPA DE APROXIMACIÓN)
int velocidadDeseada = 0;

if (pulsosFaltantes > 25) {
    velocidadDeseada = velocidadGiroManual;
} 
else if (pulsosFaltantes > 8) {
    velocidadDeseada = 40;
} 
else {
    velocidadDeseada = 32; 
}

valorPerfilExacto = velocidadDeseada;
moverMotorPWM(sentidoDeViaje, velocidadDeseada);
}

// ==========================================
// CÁLCULO DE PERFILES MATEMÁTICOS (1 a 9)
// ==========================================
float calcularEcuacionPerfil(int numPerfil, float t) {
float p = 0;
switch (numPerfil) {
    case 1: p = p0; break;
    case 2: { float a = 8.0; p = p0 + (a * t); } break;
    case 3: {
        float a = 25.0, ta = 2.0, tb = 5.0, tc = 7.0;
        float p_max = p0 + (a * ta);
        if (t >= 0 && t < ta) p = p0 + (a * t);
        else if (t >= ta && t < tb) p = p_max;
        else if (t >= tb && t < tc) p = p_max - (a * (t - tb));
        else p = 0;
    } break;
    case 4: { 
        // Onda senoidal completa centrada en 50% con amplitud de 45% (oscila de 5% a 95% sin recortarse)
        float fm = 0.5; 
        float offsetSenoidal = 50.0;
        float amplitudSenoidal = 45.0;
        p = offsetSenoidal + amplitudSenoidal * sin(2.0 * M_PI * fm * t); 
    } break;
    case 5: {
        float p1 = 20.0, p2 = 80.0;
        p = (fmod(t, T) < (T / 2.0)) ? p1 : p2;
    } break;
    case 6: { float alpha = 0.3, k = 1.8; p = p0 * pow((1.0 + (alpha * t)), k); } break;
    case 7: { float t0 = 4.0, sigma = 1.2; p = A * exp(-pow((t - t0) / sigma, 2.0)); } break;
    case 8: p = A * fmod(t / T, 1.0); break;
    case 9: { float ruido = ((float)random(-1000, 1000) / 1000.0) * A; p = p0 + ruido; } break;
    default: p = 0; break;
}
return constrain(p, 0.0, 100.0);
}

// ==========================================
// MENÚ INTERACTIVO
// ==========================================
void mostrarMenu() {
modoGrafica = false;
  float gradosActuales = ((float)contadorPulsos / pulsosPorRev) * 360.0;

Serial.println("\n=======================================================");
Serial.println("   PANEL DE CONTROL DE POSICION Y VELOCIDAD (PWM)      ");
Serial.println("=======================================================");
Serial.println(" COMANDOS DISPONIBLES:");
Serial.println("   h       -> Buscar sensor HOME fisicamente y calibrar CERO");
Serial.println("   5       -> Guardar posicion actual como CERO (0° / 12:00)");
Serial.println("   0       -> Regresar al punto CERO guardado (0°)");
Serial.println("   a[grad] -> Ir a angulo exacto (Ej: a90, a180, a360)");
Serial.println("   r[min]  -> Ir a marca de reloj 0-60 en sentido horario (Ej: r15 -> 90°)");
Serial.println("   reloj   -> Modo segundero automatico en sentido horario (6°/seg)");
Serial.println("   7       -> Ver posicion actual detallada");
Serial.println("   inv     -> Invertir sentido del encoder");
Serial.println("   1 / 2   -> Giro continuo: 1 = Derecha | 2 = Izquierda");
Serial.println("   v[0-100]-> Ajustar velocidad de giro % PWM (Ej: v35, v80)");
Serial.println("   p1 a p9 -> Activar Perfil Matematico 1 al 9");
Serial.println("   4       -> PARAR motor");
Serial.println("   6       -> Calibrar pulsos por vuelta");
Serial.println("   c       -> Calibrar CERO del sensor de corriente");
Serial.println("   g / m   -> Iniciar / Pausar graficas en Teleplot");
Serial.println("=======================================================");
Serial.print(" > Pulsos actuales: "); Serial.print(contadorPulsos);
Serial.print(" ("); Serial.print(gradosActuales, 1); Serial.print("°)");
Serial.print(" | Meta: "); Serial.print(gradosObjetivo, 1); Serial.print("°");
Serial.print(" | Vel. Giro: "); Serial.print(velocidadGiroManual); Serial.println("%");
Serial.print(" > Estado Sensor Home (Pin 33): ");
Serial.println(digitalRead(PIN_HOME_SENSOR) == ESTADO_DETECTADO ? "DETECTADO (En Home)" : "LIBRE");
Serial.print(" > Voltaje Base 0A (Pin 32): ");
Serial.print(voltajeCero, 3); Serial.println(" V");
imprimirConversionReloj(gradosActuales);
Serial.println("=======================================================\n");
}

// ==========================================
// RUTINA DE BÚSQUEDA DE HOME CON SENSOR MH FLYING FISH
// ==========================================
void buscarHomeFisico() {
modoGrafica = false;
modoPosicion = false;
modoSegundero = false;
enMovimientoHaciaMeta = false;
frenarMotor();

Serial.println("\n>> [HOMING] Buscando el sensor de origen (Home) en Pin 33...");

  // 1. Si al encender ya está pisando el sensor, retrocede un poco para tomar siempre el mismo flanco
if (digitalRead(PIN_HOME_SENSOR) == ESTADO_DETECTADO) {
    Serial.println("   -> El eje ya esta sobre el sensor. Saliendo de la zona para re-calibrar...");
    moverMotorPWM(-1, 35);
    unsigned long tSalida = millis();
    while (digitalRead(PIN_HOME_SENSOR) == ESTADO_DETECTADO) {
      if (millis() - tSalida > 4000) break; // Timeout de seguridad de 4s
    delay(5);
    }
    delay(150); // Margen adicional fuera del sensor
    frenarMotor();
    delay(250);
}

  // 2. Girar lento hacia adelante hasta que el sensor detecte la marca
  moverMotorPWM(1, 35); // Velocidad baja (35%) para frenar con precisión
unsigned long tBusqueda = millis();
bool encontrado = false;

  while (millis() - tBusqueda < 10000) { // Máximo 10 segundos de búsqueda
    if (digitalRead(PIN_HOME_SENSOR) == ESTADO_DETECTADO) {
    encontrado = true;
    break;
    }
    delay(1);
}

frenarMotor();

if (encontrado) {
    delay(150); // Espera breve para que se disipe cualquier inercia mecánica
    contadorPulsos = 0;
    gradosObjetivo = 0.0;
    pulsosObjetivo = 0;
    segundosRelojActual = 0;
    Serial.println(">> [HOME DETECTADO] Posicion CERO (0° / 12:00) calibrada automaticamente.");
} else {
    Serial.println(">> [ALERTA] No se detecto el sensor tras 10 seg. Verifica la conexion en el Pin 33.");
}
}

void calibrarPulsos() {
modoGrafica = false;
modoPosicion = false;
modoSegundero = false;
enMovimientoHaciaMeta = false;
frenarMotor();
contadorPulsos = 0;

Serial.println("\n--- MODO CALIBRACION ---");
Serial.println("1. Gira el eje A MANO exactamente UNA vuelta completa.");
Serial.println("2. Escribe 'ok' y presiona Enter.");
leerLinea();

long pulsos = abs(contadorPulsos);
if (pulsos == 0) {
    Serial.println("Error: No se detectaron pulsos. Revisa el encoder.");
} else {
    pulsosPorRev = pulsos;
    calibrado = true;
    Serial.print("Exito! Calibrado a: "); Serial.print(pulsosPorRev); Serial.println(" pulsos/vuelta.");
}
contadorPulsos = 0;
mostrarMenu();
}

// ==========================================
// SETUP
// ==========================================
void setup() {
Serial.begin(115200);
delay(1000);

  // Configurar resolución analógica del ESP32 a 12 bits (0 a 4095) para el sensor de corriente
analogReadResolution(12);
pinMode(PIN_SENSOR_CORRIENTE, INPUT);

  // Configurar el sensor MH Flying Fish (Home) en GPIO 33
pinMode(PIN_HOME_SENSOR, INPUT_PULLUP);

  // Inicializar PWM por hardware en el ESP32 para los pines del motor
ledcAttach(RPWM_PIN, PWM_Freq, PWM_Res);
ledcAttach(LPWM_PIN, PWM_Freq, PWM_Res);

pinMode(REN_PIN, OUTPUT);
pinMode(LEN_PIN, OUTPUT);

frenarMotor();

pinMode(HALL_A_PIN, INPUT_PULLUP);
pinMode(HALL_B_PIN, INPUT_PULLUP);

  // El BOTON_BOOT se deja configurado pero ya no se procesa en el loop
pinMode(BOTON_BOOT, INPUT_PULLUP);

attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), procesarEncoder, RISING);

  // 1. Busca automáticamente la posición Home al encender o reiniciar el ESP32
buscarHomeFisico();

  // 2. Con el motor ya detenido en Home, calibra automáticamente el 0 de corriente
calibrarCorrienteCero();

tiempoInicio = millis();
mostrarMenu();
}

// ==========================================
// LOOP PRINCIPAL
// ==========================================
void loop() {
if (modoPosicion) {
    actualizarControlPosicion();
}

  // Comandos por Serial y Teleplot
if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd.length() > 0) {
    if (cmd == "m" || cmd == "8") {
        mostrarMenu();
    } 
    else if (cmd == "h") {
        buscarHomeFisico();
        mostrarMenu();
    }
    else if (cmd == "g") {
        Serial.println("Iniciando transmision de datos para Teleplot...");
        delay(300);
        modoGrafica = true;
    }
    else if (cmd == "c") {
        modoPosicion = false;
        modoSegundero = false;
        enMovimientoHaciaMeta = false;
        calibrarCorrienteCero();
    }
    else if (cmd == "inv") {
        modoPosicion = false;
        modoSegundero = false;
        enMovimientoHaciaMeta = false;
        frenarMotor();
        invertirEncoder = !invertirEncoder;
        contadorPulsos = 0;
        Serial.print(">> Polaridad del encoder invertida. Ahora invertirEncoder = ");
        Serial.println(invertirEncoder ? "true" : "false");
    }
    else if (cmd == "5") {
        modoPosicion = false;
        modoSegundero = false;
        enMovimientoHaciaMeta = false;
        frenarMotor();
        contadorPulsos = 0;
        gradosObjetivo = 0.0;
        pulsosObjetivo = 0;
        segundosRelojActual = 0;
        Serial.println(">> [ORIGEN GUARDADO] Posicion actual establecida como CERO (0° / 12:00).");
    }
    else if (cmd == "0") {
        modoSegundero = false;
        contadorPulsos = contadorPulsos % pulsosPorRev;
        irAGrados(0.0);
        Serial.println(">> Regresando al Punto CERO guardado (0°)...");
    }
    else if (cmd.startsWith("a") && cmd.length() > 1) {
        modoSegundero = false;
        float grad = cmd.substring(1).toFloat();
        if (abs(grad) <= 360.0) {
        contadorPulsos = contadorPulsos % pulsosPorRev;
        if (contadorPulsos < 0 && grad >= 0) contadorPulsos += pulsosPorRev;
        }
        irAGrados(grad);
        Serial.print(">> Moviendo motor a: "); Serial.print(grad, 2); Serial.println("°");
        imprimirConversionReloj(grad);
    }
    else if (cmd.startsWith("r") && cmd != "reloj" && cmd.length() > 1) {
        modoSegundero = false;
        float marcaReloj = cmd.substring(1).toFloat();
        // Multiplica por SENTIDO_RELOJ para ir a la marca en sentido horario
        float grad = marcaReloj * 6.0 * SENTIDO_RELOJ;
        contadorPulsos = contadorPulsos % pulsosPorRev;
        if (SENTIDO_RELOJ == 1 && contadorPulsos < 0) contadorPulsos += pulsosPorRev;
        if (SENTIDO_RELOJ == -1 && contadorPulsos > 0) contadorPulsos -= pulsosPorRev;
        irAGrados(grad);
        Serial.print(">> Reloj movido en sentido horario a la marca: "); Serial.print(marcaReloj, 1);
        imprimirConversionReloj(grad);
    }
    else if (cmd == "reloj") {
        frenarMotor();
        gradosBaseReloj = ((float)contadorPulsos / pulsosPorRev) * 360.0;
        segundosRelojActual = 1;
        modoSegundero = true;
        ultimoSegundoReloj = millis();
        // Avanza 6° en sentido horario (SENTIDO_RELOJ)
        irAGrados(gradosBaseReloj + (6.0 * SENTIDO_RELOJ));
        Serial.println(">> MODO SEGUNDERO ACTIVADO EN SENTIDO HORARIO (Avanza 6° por segundo).");
    }
    else if (cmd == "7") {
        float gradosActuales = ((float)contadorPulsos / pulsosPorRev) * 360.0;
        Serial.println("\n--- LECTURA ACTUAL ---");
        Serial.print("   -> Pulsos actuales: "); Serial.println(contadorPulsos);
        Serial.print("   -> Sensor Home (Pin 33): ");
        Serial.println(digitalRead(PIN_HOME_SENSOR) == ESTADO_DETECTADO ? "DETECTADO" : "LIBRE");
        Serial.print("   -> Corriente actual (Pin 32): ");
        Serial.print(corrienteFiltrada, 2); Serial.println(" A");
        imprimirConversionReloj(gradosActuales);
    }
    else if (cmd.startsWith("v") && cmd.length() > 1) {
        int nuevaVel = constrain(cmd.substring(1).toInt(), 0, 100);
        velocidadGiroManual = nuevaVel;
        p0 = (float)nuevaVel; // Actualiza también la base constante para el giro continuo
        if (!modoPosicion && direccion != 0) {
        moverMotorPWM(direccion, velocidadGiroManual);
        }
        Serial.print(">> Velocidad de giro ajustada a: ");
        Serial.print(velocidadGiroManual);
        Serial.println("% PWM.");
    }
    else if (cmd.startsWith("p") && cmd.length() == 2 && isDigit(cmd[1])) {
        int num = cmd.substring(1).toInt();
        if (num >= 1 && num <= 9) {
        modoPosicion = false;
        modoSegundero = false;
        enMovimientoHaciaMeta = false;
        perfilActivoMotor = num;
        direccion = 1;
        tiempoInicio = millis();
          modoGrafica = true; // Para que Teleplot grafique inmediatamente al cambiar perfil
        Serial.print(">> Motor en modo Perfil "); Serial.println(perfilActivoMotor);
        }
    } 
    else if (cmd == "1") {
        modoPosicion = false;
        modoSegundero = false;
        enMovimientoHaciaMeta = false;
        perfilActivoMotor = 1;
        p0 = (float)velocidadGiroManual;
        moverMotorPWM(1, velocidadGiroManual);
        Serial.print(">> Giro continuo a la DERECHA (");
        Serial.print(velocidadGiroManual);
        Serial.println("% PWM).");
    } 
    else if (cmd == "2") {
        modoPosicion = false;
        modoSegundero = false;
        enMovimientoHaciaMeta = false;
        perfilActivoMotor = 1;
        p0 = (float)velocidadGiroManual;
        moverMotorPWM(-1, velocidadGiroManual);
        Serial.print(">> Giro continuo a la IZQUIERDA (");
        Serial.print(velocidadGiroManual);
        Serial.println("% PWM).");
    } 
    else if (cmd == "4") {
        modoPosicion = false;
        modoSegundero = false;
        enMovimientoHaciaMeta = false;
        frenarMotor();
        Serial.println(">> MOTOR DETENIDO.");
    } 
    else if (cmd == "6") {
        calibrarPulsos();
    } 
    else {
        Serial.println("Comando no reconocido.");
    }
    }
}
  // Lógica de modo segundero en sentido horario (SENTIDO_RELOJ)
if (modoSegundero && (millis() - ultimoSegundoReloj >= 1000)) {
    ultimoSegundoReloj += 1000;
    segundosRelojActual++;
    irAGrados(gradosBaseReloj + (segundosRelojActual * 6.0 * SENTIDO_RELOJ));
}

  // Reporte a Teleplot y lectura de corriente (cada 20 ms)
if (millis() - ultimoReporte >= 20) {
    ultimoReporte = millis();
    float tTotal = (millis() - tiempoInicio) / 1000.0;
    float tCiclo = fmod(tTotal, periodoCiclo);

    if (!modoPosicion) {
    if (direccion != 0) {
        valorPerfilExacto = calcularEcuacionPerfil(perfilActivoMotor, tCiclo);
        int valPerfil = (int)valorPerfilExacto;
        moverMotorPWM(direccion, valPerfil);
    } else {
        frenarMotor();
    }
    }

    // --- LECTURA CORREGIDA DEL SENSOR DE CORRIENTE (20A) ---
    // 1. Promediamos 64 lecturas en milivoltios ANTES del abs() para que el ruido se cancele
    long sumaMV = 0;
    for (int i = 0; i < 64; i++) {
    sumaMV += analogReadMilliVolts(PIN_SENSOR_CORRIENTE);
    }
    float voltajePin = ((float)sumaMV / 64.0) / 1000.0; // Convertir mV a Voltios

    // 2. Sensibilidad del ACS712 de 20A = 0.100 V/A (100 mV por Ampere)
    float corrienteInstantanea = abs(voltajePin - voltajeCero) / 0.100;

    // 3. Zona muerta: si el motor está detenido (estadoMotor == 0) o es ruido (< 0.12 A), es 0.0 A
    if (estadoMotor == 0 || corrienteInstantanea < 0.12) {
    corrienteInstantanea = 0.0;
    }

    // 4. Filtro pasa-bajas exponencial
    corrienteFiltrada = (corrienteFiltrada * 0.80) + (corrienteInstantanea * 0.20);
    if (corrienteFiltrada < 0.02) corrienteFiltrada = 0.0;

    if (modoGrafica) {
    long pulsos = contadorPulsos;
    float vueltas = (float)pulsos / pulsosPorRev;
      float gradosAcumulados = vueltas * 360.0;
    float angulo = fmod(gradosAcumulados, 360.0);
    if (angulo < 0) angulo += 360.0;

      // Tus 4 variables originales intactas:
    Serial.print(">Estado_Motor_PWM:"); Serial.println(estadoMotor);
    Serial.print(">Pulsos_Encoder:"); Serial.println(pulsos);
    Serial.print(">Grados_Acumulados:"); Serial.println(gradosAcumulados, 1);
    Serial.print(">Grados_Objetivo:"); Serial.println(gradosObjetivo, 1);
    
      // Variables para ver el perfil activo y la exactitud del PWM:
    Serial.print(">Perfil_Activo:"); Serial.println(perfilActivoMotor);
    Serial.print(">Perfil_Teorico_Exacto:"); Serial.println(valorPerfilExacto, 2);
    Serial.print(">PWM_Real_Bits_0_255:"); Serial.println(valorPWMRealBits);

      // Variable para graficar la corriente en Amperes:
    Serial.print(">Corriente_Amperes:"); Serial.println(corrienteFiltrada, 2);
    }
}
}