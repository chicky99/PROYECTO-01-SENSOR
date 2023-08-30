//JOSE LUIS MONTERROSO
//20142
// PROYECTO #1 SENSOR DE TEMPERATURA

// DEFINIR LIBRERIAS ************************************************************************
#include <Arduino.h>
#include <ESP_LM35.h>
#include <config.h>
// DEFINICION DE PINES **********************************************************************
#define ledr 23
#define leda 22
#define ledv 21
#define sensort 36
#define boton 39
#define servopin 2
#define displayA 13
#define displayB 12
#define displayC 14
#define displayD 26
#define displayE 27
#define displayF 25
#define displayG 32
#define displaydp 33
#define transistorA 19
#define transistorB 18
#define transistorC 5

// DEFINICION DE VARIABLES GLOBALES *********************************************************
float temp;
int decena;
int unidades;
int decimal = 10;
int sensor = 0;
//Servo servomotor;
bool buttonState = HIGH;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

AdafruitIO_Feed *tempcanal = io.feed("PROYECTO1-ED2");

const byte segmentos[8] = {displayA, displayB, displayC, displayD, displayE, displayF, displayG,displaydp};
// DEFINICION DE CANALES PARA PWM ***********************************************************

#define pwmChannelr 0
#define pwmChannela 1
#define pwmChannelv 2
#define pwmChannels 3

// DEFINICION SENALES PWM

#define freqPWM 50

// DEFINICION DE RESOLUCION

#define resolution 8

// DEFINIR PROTOTIPOS DE FUNCIONES **********************************************************
void pwmSetup(void);
void configurarLED(int pwmChannel, int sensor);
void stemp(void);
void leds(float temp);
void servomov(float temp);
void botonsetup(void);
void displaysetup(void);
void displayDigit(int digit);
void displays(float temp);



// DEFINIR FUNCIONES PRINCIPALES ************************************************************
void setup(){
    Serial.begin(115200);
    pwmSetup();
    botonsetup();
    //servomotor.attach(servopin);
    displaysetup();
     // wait for serial monitor to open
    while(! Serial);

    Serial.print("Connecting to Adafruit IO");

    // connect to io.adafruit.com
    io.connect();

     // wait for a connection
    while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
    }

  // we are connected
    Serial.println();
    Serial.println(io.statusText());
}

void loop(){
    displays(temp); // Mostrar temperatura
    // Lectura del estado del botón con debouncing
    int reading = digitalRead(boton);
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != buttonState) {
            buttonState = reading;
            if (buttonState == LOW) {
                stemp(); // Leer temperatura
                leds(temp); // Controlar LEDs
                servomov(temp); // Controlar servo
                }
        }
    }
    lastButtonState = reading;
    
}
// DEFINIR FUNCIONES SECUNDARIAS ************************************************************

void pwmSetup(void){
    ledcSetup(pwmChannelr,freqPWM, resolution);
    ledcSetup(pwmChannela,freqPWM, resolution);
    ledcSetup(pwmChannelv,freqPWM, resolution);
    ledcSetup(pwmChannels, freqPWM, resolution);

    ledcAttachPin(ledr,pwmChannelr);
    ledcAttachPin(leda,pwmChannela);
    ledcAttachPin(ledv,pwmChannelv);
    ledcAttachPin(servopin,pwmChannels);

    
}

void configurarLED(int pwmChannel, int sensor)
{
ledcWrite(pwmChannel, sensor);
}

void stemp() {
    float temperatura = (analogRead(sensort) * 5000.0) / 4096.0;
    temp = temperatura / 10;  // Asignar a la variable global temp
    Serial.println(temp);
    io.run();
    Serial.print("sending -> ");
    Serial.println(temp);
    tempcanal->save(temp);

    //temp++;
    delay(3000);
}

void leds(float temp) {
    if (temp < 25.0) {
        configurarLED(pwmChannelr, 0);
        configurarLED(pwmChannela, 0);
        configurarLED(pwmChannelv, HIGH);
    } else if (temp < 28.0) {
        configurarLED(pwmChannelr, 0);
        configurarLED(pwmChannela, HIGH);
        configurarLED(pwmChannelv, 0);
    } else {
        configurarLED(pwmChannelr, HIGH);
        configurarLED(pwmChannela, 0);
        configurarLED(pwmChannelv, 0);
    }
}

void servomov(float temp){
    if(temp < 25.0){
        int dutyCycle = map(0, 0, 180, 20, 115); // 0 grados
        ledcWrite(pwmChannels, dutyCycle);
        delay(500);
    } else if(temp < 28.0){
        int dutyCycle = map(90, 0, 180, 20, 115); // 90 grados
        ledcWrite(pwmChannels, dutyCycle);
        delay(500);
    } else{
        int dutyCycle = map(180, 0, 180, 20, 115); // 180 grados
        ledcWrite(pwmChannels, dutyCycle);
        delay(500);
    }
}

void displayDigit(int digit){

    const byte digitPatterns[11][7] ={ { 1,1,1,1,1,1,0},
        {0, 1, 1, 0, 0, 0, 0}, // 1
        {1, 1, 0, 1, 1, 0, 1}, // 2
        {1, 1, 1, 1, 0, 0, 1}, // 3
        {0, 1, 1, 0, 0, 1, 1}, // 4
        {1, 0, 1, 1, 0, 1, 1}, // 5
        {1, 0, 1, 1, 1, 1, 1}, // 6
        {1, 1, 1, 0, 0, 0, 0}, // 7
        {1, 1, 1, 1, 1, 1, 1}, // 8
        {1, 1, 1, 1, 0, 1, 1},  // 9
        {0, 0, 0, 0, 0, 0, 1}  // dp
    };

    for (int i = 0; i < 7; i++){
        digitalWrite(segmentos[i],digitPatterns[digit][i]);
    }

    digitalWrite(segmentos[7],0);
}

void displays(float temp){
    int tempint = int(temp);
    int decena = tempint/ 10;
    int unidades = tempint % 10;
    int decimal = int(temp * 10) % 10;

    digitalWrite(transistorA,HIGH);
    digitalWrite(transistorB,LOW);
    digitalWrite(transistorC,LOW);

    displayDigit(decena);
    delay(5);

    digitalWrite(transistorA,LOW);
    digitalWrite(transistorB,HIGH);
    digitalWrite(transistorC,LOW);

    displayDigit(unidades);  
    delay(5);

    digitalWrite(transistorA,LOW);
    digitalWrite(transistorB,LOW);
    digitalWrite(transistorC,HIGH);

    displayDigit(decimal);
    delay(5);
}

void botonsetup(){
    pinMode(boton, INPUT);
}

void displaysetup(void){
    pinMode(displayA, OUTPUT);
    pinMode(displayB, OUTPUT);
    pinMode(displayC, OUTPUT);
    pinMode(displayD, OUTPUT);
    pinMode(displayE, OUTPUT);
    pinMode(displayF, OUTPUT);
    pinMode(displayG, OUTPUT);
    pinMode(displaydp, OUTPUT);
    pinMode(transistorA, OUTPUT);
    pinMode(transistorB, OUTPUT);
    pinMode(transistorC, OUTPUT);
}
