#include <PID_v1.h>
#include <OneWire.h>

// https://skyduino.wordpress.com/2012/04/26/arduino-capteur-de-temperature-ds18b20/
// faut une résistance 4k7 ou moins

/********************************************************
   PID RelayOutput Example
   Same as basic example, except that this time, the output
   is going to a digital pin which (we presume) is controlling
   a relay.  The pid is designed to output an analog value,
   but the relay can only be On/Off.

     To connect them together we use "time proportioning
   control"  Tt's essentially a really slow version of PWM.
   First we decide on a window size (5000mS say.) We then
   set the pid to adjust its output between 0 and that window
   size.  Lastly, we add some logic that translates the PID
   output into "Relay On Time" with the remainder of the
   window being "Relay Off Time"
 ********************************************************/

#define DS18B20 0x28     // Adresse 1-Wire du DS18B20
#define BROCHE_ONEWIRE 7 // Broche utilisée pour le bus 1-Wire
#define RelayPin 6       // broche qui commute une relais on/off
#define TEMP_CIBLE 38    // tempearture cible

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
int WindowSize = 5000;
unsigned long windowStartTime;

OneWire ds(BROCHE_ONEWIRE); // Création de l'objet OneWire ds
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT); //Specify the links and initial tuning parameters


// Fonction récupérant la température depuis le DS18B20
// Retourne true si tout va bien, ou false en cas d'erreur
boolean getTemperature(float *temp) {
  byte data[9], addr[8];
  // data : Données lues depuis le scratchpad
  // addr : adresse du module 1-Wire détecté

  if (!ds.search(addr)) { // Recherche un module 1-Wire
    ds.reset_search();    // Réinitialise la recherche de module
    return false;         // Retourne une erreur
  }

  if (OneWire::crc8(addr, 7) != addr[7]) // Vérifie que l'adresse a été correctement reçue
    return false;                        // Si le message est corrompu on retourne une erreur

  if (addr[0] != DS18B20) // Vérifie qu'il s'agit bien d'un DS18B20
    return false;         // Si ce n'est pas le cas on retourne une erreur

  ds.reset();             // On reset le bus 1-Wire
  ds.select(addr);        // On sélectionne le DS18B20

  ds.write(0x44, 1);      // On lance une prise de mesure de température
  delay(800);             // Et on attend la fin de la mesure

  ds.reset();             // On reset le bus 1-Wire
  ds.select(addr);        // On sélectionne le DS18B20
  ds.write(0xBE);         // On envoie une demande de lecture du scratchpad

  for (byte i = 0; i < 9; i++) // On lit le scratchpad
    data[i] = ds.read();       // Et on stock les octets reçus

  // Calcul de la température en degré Celsius
  *temp = ((data[1] << 8) | data[0]) * 0.0625;

  // Pas d'erreur
  return true;
}

void setup() {
  // put your setup code here, to run once:
  // initialize inputs/outputs

  Serial.begin(9600); ; // Initialisation du port série

  // PID
  windowStartTime = millis();
  Setpoint = TEMP_CIBLE; //initialize the variables we're linked to = temp cible
  myPID.SetOutputLimits(0, WindowSize); //tell the PID to range between 0 and the full window size
  myPID.SetMode(AUTOMATIC);//turn the PID on
}

void loop() {
  // put your main code here, to run repeatedly:
  float temp;

  // Lit la température ambiante à ~1Hz
  if (getTemperature(&temp)) {

    // Affiche la température
    Serial.print("Temperature : ");
    Serial.print(temp);
    Serial.write(176); // caractère °
    Serial.write('C');
    Serial.println();

    /************************************************
       turn the output pin on/off based on pid output
     ************************************************/
    unsigned long now = millis();
    Input = temp ;
    myPID.Compute();
    if (now - windowStartTime > WindowSize) { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if (Output > now - windowStartTime) digitalWrite(RelayPin, HIGH); else digitalWrite(RelayPin, LOW);
  }
}

