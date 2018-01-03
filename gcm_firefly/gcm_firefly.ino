#include <LowPower.h>

#include <Crypto.h>
#include <AES.h>
#include <GCM.h>
#include <TransistorNoiseSource.h>
#include <RNG.h> // NEED TO Disable RNG_WATCHDOG 
GCM<AES128> gcm;
// Noise source to seed the random number generator.
TransistorNoiseSource noise(A1);



#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>

#ifdef TEMPERATURE_SENSOR
#include <OneWire.h> 
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 12
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);
#endif

#define FIREFLY_ADDR (0x10)

// Address of device on lora network
#define DEVICE_ADDRESS (0x07)
/* for feather32u4 */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

/* for feather m0 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
 */

#define RF95_FREQ 915.0

// radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

char key[] = { 0xaa, 0xbb, 0xbb, 0xcc, 0xdd, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x11, 0x22, 0x33, 0x44, 0x55 };
                


static uint8_t power_array[10];
static uint8_t power_array_idx = 0;
static uint32_t power_min;
static uint32_t power_now;
static uint32_t power_avg;
static uint32_t power_max;
static uint16_t val16 = 0;

void randomSetup() {
    RNG.begin("lora cloud", 950);
    // feed with temp sensor address?
    char sd[] = { '1', 34, 66, 102, 55 };
    RNG.stir(sd, sizeof(sd));

    // feed with a couple bytes of the key
    RNG.addNoiseSource(noise);
}

void radioInit()
{
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    // manually reset the radio
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    while (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    }
    Serial.println("LoRa radio init OK!");

    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }

    rf95.setTxPower(5, false);
}

struct authenticatedData {
    uint8_t to;
    uint8_t from;
    uint8_t id;
    uint8_t flags;
};

char message[] = "\x82\xa6values\x82\xabtemperature\xd1\x00\x00\xa5power\xcd\x00\x00\xa4type\xa6report";

struct Payload {
    uint8_t iv[12];
    uint8_t ct[sizeof(message) - 1];
    uint8_t tag[16];
};



void setup() 
{  
    
#ifdef TEMPERATURE_SENSOR
  sensors.begin();
#endif
  
  
  //memcpy(message, MESSAGE, sizeof(MESSAGE) - 1);
  //while (!Serial);
  Serial.begin(9600);
  delay(100);
  pinMode(13, OUTPUT);
  radioInit();
  randomSetup();
  Wire.begin();

  // Turn LEDs off by default
  Wire.beginTransmission(FIREFLY_ADDR);
  Wire.write('l');
  Wire.endTransmission();
 
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
    Serial.print("0x");
    for (int i = 0; i<length; i++) {
        if (data[i]<0x10) { Serial.print("0"); }
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
}

void sendMessage(int16_t temperature, uint16_t power_now)
{
    Payload p;
    authenticatedData ad;

  int16_t * temperatureInMsg = (int16_t*)&message[22];
  uint16_t * power = (uint16_t*)&message[31];
  
#ifdef TEMPERATURE_SENSOR
   *temperatureInMsg = htons((int16_t)temperature);
#else
  *temperatureInMsg = 0;
#endif
  *power = htons((uint16_t)power_now);


    //static int16_t * temperatureInMsg = (int16_t*)&p.msg[23 + 4];
    //static uint16_t * doorOpens = (uint16_t*)&p.msg[32 + 4];
    //char ct[sizeof(MESSAGE)];
    //char tag[16];

    // Send a message to rf95_server
    ad.to = 0x01;
    ad.from = DEVICE_ADDRESS;
    ad.id = 0x01;
    ad.flags = 0x02; // 0000 00x0 -- x says we use the validation
    rf95.setHeaderId(ad.id);
    rf95.setHeaderFrom(ad.from);
    rf95.setHeaderTo(ad.to);
    rf95.setHeaderFlags(ad.flags);

    //*temperatureInMsg = htons((int16_t)temperature);
    //*doorOpens = opens;

    RNG.rand(p.iv, 12);
    gcm.setKey(key, sizeof(key));
    gcm.setIV(p.iv, sizeof(p.iv));
    gcm.addAuthData((uint8_t *)&ad, 4);
    gcm.encrypt(p.ct, message, sizeof(message) - 1);
    gcm.computeTag(p.tag, sizeof(p.tag));

    
    //int len = sprintf(outPacket, "xyztemperaturevzpacketv", (int)temperature, packetnum++);// x is used for our verification byte
    // add in verification byte
    // Add em all together and then add 0xc1
    //uint8_t check = to + from + packetNumByte + flags + 0xc1;

    Serial.println("");
    Serial.println("ad: ");
    PrintHex8((uint8_t *)&ad, 4);
    Serial.println("");
    Serial.println("iv: ");
    PrintHex8(p.iv, 12);
    Serial.println("");
    Serial.println("tag: ");
    PrintHex8(p.tag, 16);
    Serial.println("");
    Serial.println("ct: ");
    PrintHex8(p.ct, 5);
    Serial.println("");
    PrintHex8((char *)p.iv, 12 + 16 + sizeof(message) - 1);
    Serial.println("");


    Serial.println("Sending to server");
    Serial.println("Sending..."); delay(10);

  
  
  Serial.println("Sending to server");
  Serial.println("Sending..."); delay(10);
  //rf95.send((uint8_t *)outPacket, sizeof(outPacket) - 1);

    
    //Serial.println();
    rf95.send((char *)p.iv, 12 + 16 + sizeof(message) - 1 );
    rf95.waitPacketSent();
}



void loop()
{
#ifdef TEMPERATURE_SENSOR
  sensors.requestTemperatures();
  Serial.print("Temperature is: "); 
  float temperature = sensors.getTempFByIndex(0);
  Serial.print(temperature);
#endif
  

   Wire.beginTransmission(FIREFLY_ADDR);
    Wire.write('x');
    Wire.endTransmission(false);
    
    // Try to fetch the raw sensor values
    Wire.requestFrom(FIREFLY_ADDR, 9, true);
    
    power_array_idx = 0;
    while(Wire.available())
    {
        power_array[power_array_idx] = Wire.read();
        power_array_idx++;
    }

    // Min
    val16 = power_array[1];
    val16 <<= 8;
    val16 |= power_array[2];
    power_min = val16;

    // Instant
    val16 = power_array[3];
    val16 <<= 8;
    val16 |= power_array[4];
    power_now = val16;

    // Avg
    val16 = power_array[5];
    val16 <<= 8;
    val16 |= power_array[6];
    power_avg = val16;

    // Max
    val16 = power_array[7];
    val16 <<= 8;
    val16 |= power_array[8]; 
    power_max = val16;

#if 0
        // Status
        Serial.print("LED status: ");
        Serial.println(power_array[0]);
    
        // Minimum
        Serial.print(" Min (W): ");
        Serial.println(power_min);
    
        // Last
        Serial.print("Last (W): ");
        Serial.println(power_now);
    
        // Avg
        Serial.print(" Avg (W): ");
        Serial.println(power_avg);
    
        // Max
        Serial.print(" Max (W): ");
        Serial.println(power_max);
#endif
  sendMessage(0, power_now);
  rf95.sleep();
  //digitalWrite(13, LOW);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  //digitalWrite(13, HIGH);

}
    
