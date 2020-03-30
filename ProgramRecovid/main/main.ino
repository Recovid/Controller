#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include "../main.h"
#include "../sensor.h"

#include <Thread.h> //https://github.com/ivanseidel/ArduinoThread
#include <ThreadController.h>


#define DEBUG 1
#define uint8_t unsigned char
#define uint16_t unsigned short


//Revoir l'utilité de chaque variable pour sa definition et son type.
unsigned int RawO2 = 0,
        PAtmo = 0,
        Ti = 0,
        VTi = 0,
        VTe = 0;

float   Paw = 0,
        QPatientSLM = 0;

long    VMi = 0,
        VMe = 0;

bool Flag_moteur = true;
bool Flag_expiration = true;


Thread sensorReadingThread = Thread();
Thread myThread_25Hz = Thread();
Thread myThread_100Hz = Thread();

void configure_threads(){
    sensorReadingThread.enabled = true; // Default enabled value is true
    sensorReadingThread.setInterval(1); // Setts the wanted interval in ms
    //sensorReadingThread.ThreadName = "myThread tag";
    sensorReadingThread.onRun(thread_1KHertz); // callback of the function

    myThread_25Hz.enabled = true; // Default enabled value is true
    myThread_25Hz.setInterval(40); // Setts the wanted interval in ms
    //myThread_25Hz.ThreadName = "myThread tag";
    myThread_25Hz.onRun(thread_1KHertz); // callback of the function

    myThread_100Hz.enabled = true; // Default enabled value is true
    myThread_100Hz.setInterval(10); // Setts the wanted interval in ms
    //sensorReadingThread.ThreadName = "myThread tag";
    myThread_100Hz.onRun(thread_1KHertz); // callback of the function
}

configure_threads();
StaticThreadController<3> controller (&sensorReadingThread, &myThread_25Hz, &myThread_100Hz);

/*
 * Paw = pression voie aerienne (en cmH20) [-2;100] [0.1]
 * QPatientSLM = debit patient a 21°C et 1013hPa [-200;200] [0.1]
 * QPatientBTPS = debit Patient a 37°C, Pression ambiante et 100% Humidite (breath Temperature and pressure saturated) [-200;200] [0.1]
 * RawO2 = tension qui sort de la cellule O2
 * PAtmo Pression atmospherique (exprime en hpa) [0;1200]
 * Ti = Temps d'inspiration = VT(en L)/Debit de pointe(L/Min) [0;2] [0.1]
 * VTi = volume tidal inspiratoire (ml) [0;1500] [1]
 * VTe = Volume tidal expiratoire (ml) [0;1500] [1]
 */
/*
* Inspiration :
*
* on ferme l'electovalve I/E (Valve expi connecte a la sortie du ballon)
* on actionne le ballon pendant le temps Ti a la vitesse determiné (pas de controle durant le cycle)
* apres le Ti on commence a faire revenir le ballon mais on garde l'electrovanne fermé durant le temps de plateau ou le temps
* de la pause inpiratoire (tant que le medecin a le bouton appuyé) puis passage a l'expiration.
*
* Expiration
*
*        On ouvre l'electrovanne (plutot simple)
* si le medecin appuie sur pause expiratoire on ferme la l'electrovanne tant qu'il est appuye dessus
*/

void setup( void ) {
    Wire.begin();
    Serial.begin( BAUDRATE );
    setup_PAtmo();
    measflow.init();
}

void loop( void ){

}


void thread_25hertz(){
    //pour les trames
    sendPckt( 1 );
}

/* lecture de RawO2 et PAtmo toutes les 1 secondes. */
void thread_100Hertz( ){
    //pour les mesures et la commande moteur
    PAtmo = get_PAtmo();
}

/* lecture de Paw et QpatientSLM toutes les 1 millisecondes. */
void thread_1KHertz(){
    QPatientSLM = get_QPatientSLM();
    //Paw = get_Paw();
    //sendPckt( 0 ); //sending only Paw and QPatientSLM
}





void Cycle(){
    breathIn();

    breathOut();
}

void breathIn(){ //Inspiration

    electroValve( EV_CLOSE ); //close electovalve

    controle_motor( Ti , Speed );
}

void breathOut(){ //expiratoire
    if( !Flag_expiratoire )
        electroValve( EV_OPEN ); //open electovalve
}

void control_motor( float time , int speed ){
    //
}

void electroValve( bool state ){
    if( state ){
        //Open EV
    } else {
        //close EV
    }
}

float  get_QpatientBTPS ( uint16_t QPatientSLM , uint16_t PAtmo ){
    float QPatientBTPS = QPatientSLM * 1013/(PAtmo - 62.66) * (310/294);
    if( QPatientBTPS > QPHighT || QPatientBTPS < QPLowT ){
        if( DEBUG ) Serial.println("Error: QPatientBTPS overflow ");
        return 0;
    }
    return roundf( QPatientBTPS * 100.0f) / 100.0f;
}

int get_O2Concentration( uint16_t RawO2 ){
    return RawO2 * GAIN;
}

void get_PEP(  ){}

void get_PPlat( ){}

void get_PPeak( ){}

void get_FiO2( ){}

float get_QPatientSLM( void ) {
    return ( measflow.getvalue() - SFM3300_offset ) / SFM3300_scale; //Flow
}

void setup_PAtmo( void ){
#ifdef MS5611
    ms5611.begin( 4 );
    referencePressure = ms5611.readPressure();
#endif

#ifdef BME280
    BME280_setup();
#endif

#ifdef BMP280
    BMP280_setup();
#endif
}

uint16_t get_PAtmo() {
#ifdef MS5611
    return (uint16_t) ms5611.readPressure(); //without compensation
#endif

#ifdef BME280
    return (uint16_t) bme.readPressure() / 100;
#endif

#ifdef BMP280
    return (uint16_t) bmp.readPressure() / 100;
#endif
}

void send_Packet( uint8_t type ){
    uint8_t packet[ MAX_SENDING_PCKT_LEN ] = {0};

    build_packet_upload( packet , type );

    if( type == SENDING_PACKET_TYPE_FULL )
        sendToIHM( packet , MAX_SENDING_PCKT_LEN );
    else
        sendToIHM( packet , MIN_SENDING_PCKT_LEN );
}

//From Arduino to RPi
void sendToIHM( uint8_t* buf , uint8_t sizeToSend ) {

    for (int i=0 ; i < sizeToSend ; i++) {
        Serial.print( (int) buf[i] , HEX );
    }

}

//From RPi to Arduino
void receiveFromIHM( void ) {
    uint8_t buf[ MAX_RECEIVED_PCKT_LEN ] = {0};

    int i = 0;
    while( Serial.available() && i < MAX_RECEIVED_PCKT_LEN ){
        buf[i] = (int) Serial.read();
        i++;
    }

    decode_packet( buf , i );
}

bool build_packet_upload( uint8_t* packet, uint8_t packet_type){

    packet[0] = packet_type;  //

    switch( packet_type ){

        case SENDING_PACKET_TYPE_FULL:
            packet[5] = ( RawO2 >> 8 ) & 0xFF;   //RawO2
            packet[6] = RawO2 & 0xFF;
            packet[7] = ( PAtmo >> 8 ) & 0xFF;   //PAtmo
            packet[8] = PAtmo & 0xFF;

        case SENDING_PACKET_TYPE_PAW_QPATIENT:
            packet[1] = ( (uint16_t) round( Paw*10 ) >> 8 ) & 0xFF;
            packet[2] = (uint16_t) round( Paw*10 ) & 0xFF;          //Paw
            packet[3] = ( (uint16_t) round( QPatientSLM*10 ) >> 8 ) & 0xFF;
            packet[4] = (uint16_t) round( QPatientSLM*10 ) & 0xFF;  //QPatientSLM
            return 1;

        default:
            if( DEBUG ) Serial.println("Error: Packet Type ");
            return 0;
    }
}

void decode_packet_download( uint8_t* buf, uint8_t sizeOfBuf ){
    uint8_t rcv_pckt_type = buf[0];

    switch( rcv_pckt_type ){

        case RECEIVED_PACKET_TYPE_SETTINGS:
            if( sizeOfBuf == 8 ){
                FiO2  = buf[1];
                VT    = ( buf[2] << 8 ) + buf[3];
                FR    = buf[4];
                Debit = buf[5];
                PEP   = buf[6];
                Tplat = buf[7];
            } else {
                if( DEBUG ) Serial.println(F("Error: received Packet size problem"));
            }
            break;

        case RECEIVED_PACKET_TYPE_STOP:
            if( sizeOfBuf == 2 ){
                Flag_moteur = (bool) buf[1];
            } else {
                if( DEBUG ) Serial.println(F("Error: received Packet size problem"));
            }
            break;

        default:
            Serial.println(F("Error: Received Packet Type"));
            break;
    }
}

void BME280_setup(){
    if (! bme.begin(&Wire) ) {
        if( DEBUG ) Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    // suggested rate is 1/60Hz (1m)
    //delayTime = 30000; // in milliseconds
}

void BMP280_setup(){
    if (!bmp.begin()) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        while (1);
    }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

