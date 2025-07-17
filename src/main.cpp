#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <StreamBuf.h>
#include "SoftwareSerial.h"
#include <ModbusRTU.h>

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define RXD2 D7
#define TXD2 D6
#define RTS2 D5
#define BLE_SIZE 20
#define BLE_CONN_HANDLE_INVALID -1
#define BSIZE 1024
#define SLAVE_ID 1
#define FIRST_REG 0
#define REG_COUNT 17

#define REGN 1
#define SLAVE_ID 1

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
bool hasSentToBLE = false;
int counter = 0;
uint8_t channel = 0;
uint16_t connId = BLE_CONN_HANDLE_INVALID;
String byteArrayToString(uint8_t* byteArray, size_t size);
uint8_t bufRxFromBle[BSIZE];
uint8_t _hasMessage = 0;
String _lastCommand;
StreamBuf sbufRxFromBle(bufRxFromBle, BSIZE);
unsigned long lastActivityTime = millis();
const unsigned long inactivityLimit = 1 * 60 * 1000;
EspSoftwareSerial::UART swSer;
HardwareSerial Serial2(1);
ModbusRTU mb;

int msgCounter = 0;
unsigned long lastMillis = 0;
bool bluetooth_started = false;

bool cb(Modbus::ResultCode event, uint16_t transactionId, void *data)
{ 
  // Callback to monitor errors
  if (event != Modbus::EX_SUCCESS)
  {
    Serial.print("Error Sending Message Serial Device Error: 0x");
    Serial.println(event, HEX);
  }
  return true;
}

bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
  return true;
}


class BleRxCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            Serial.print("Received Value: ");
            for (int i = 0; i < rxValue.length(); i++) {
                Serial.print(rxValue[i]);
            }
            Serial.println();
            _lastCommand = String(rxValue.c_str());
            _hasMessage = 1;
            lastActivityTime = millis();
        }
    }
};

void sendBlockRegisters(int bloc);

class BleServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer) {
        Serial.println("CALLBACK");
        Serial.println("new device connected...");
        deviceConnected = true;
        connId = pServer->getConnId();
    };

    void onDisconnect(BLEServer *pServer) {
        Serial.println("CALLBACK");
        deviceConnected = false;
        connId = BLE_CONN_HANDLE_INVALID;
        Serial.println("Device disconnected...");
        pServer->getAdvertising()->start();
        Serial.println("Waiting for a client connection to notify...");
    }
};

void sendMessageToSlave(String inputString)
{
    char charArray[inputString.length() + 1];
    inputString.toCharArray(charArray, inputString.length() + 1);
    uint8_t byteArray[inputString.length()];
    for (int i = 0; i < inputString.length(); i++) {
        byteArray[i] = charArray[i];
    }
    Serial.println("Will send a message to host...");
    pTxCharacteristic->setValue(byteArray, inputString.length() + 1);
    pTxCharacteristic->notify();
    Serial.println("Message sent to host...");
}

// Nouvelle fonction pour créer la chaîne de caractères "adresse:valeur"
void envoyerMessageAvecAdresseValeur(int adresse, int valeur)
{
    String nouvelleChaine = String(adresse) + ":" + String(valeur);
    sendMessageToSlave(nouvelleChaine);
}

void envoyerMessageAvecAdresseValeur(int adresse, const char* valeur)
{
    String nouvelleChaine = String(adresse) + ":" + String(valeur);
    sendMessageToSlave(nouvelleChaine);
}

// Nouvelle fonction pour créer la chaîne de caractères "adresse:valeur"
void envoyerFloatMessageAvecAdresseValeur(int adresse, float valeur)
{
    String nouvelleChaine = String(adresse) + ":" + String(valeur);//valeur comes from?

    sendMessageToSlave(nouvelleChaine);
}

void sendBlockRegisters(int bloc) {
    uint8_t message[22]; 
    message[0] = bloc;   
    message[1] = 16;

    if (bloc == 0) {
        // Récupération des registres 2/3, 4/5, 6/7, 8/9
        uint16_t registres[] = {mb.Hreg(2), mb.Hreg(3), mb.Hreg(4), mb.Hreg(5), //reg 4 > index 6
                                mb.Hreg(6), mb.Hreg(7), mb.Hreg(8), mb.Hreg(9)};

                                
        int index = 2;

        for (int i = 0; i < 8; i++) {
            message[index++] = registres[i] >> 8;   // Byte haut
            message[index++] = registres[i] & 0xFF; // Byte bas
        }
    } else if (bloc == 1) {
        // Récupération des registres 10/11, 12/13, 14/15, 100
        uint16_t registres[] = {mb.Hreg(10), mb.Hreg(11), mb.Hreg(12), mb.Hreg(13), 
                                mb.Hreg(0), mb.Hreg(1), mb.Hreg(100)}; //its sending register 0 / 1 starting at index 10, should it work?
        int index = 2; //temeperature register is 10? supposed to be 0/1
        
        for (int i = 0; i < 8; i++) {
            message[index++] = registres[i] >> 8;   // Byte haut
            message[index++] = registres[i] & 0xFF; // Byte bas
        } // message have 20 bytes, if adding a new register you need to change message to 24bytes and i < 9
        //but ble works better with 20 bytes, but you can try

        Serial.println("Register 100" );
        Serial.println(mb.Hreg(100));

    } else if (bloc == 2) {
        uint16_t registres[] = {mb.Hreg(300)}; 

        int index = 2;
        
        for (int i = 0; i < 8; i++) {
            message[index++] = registres[i] >> 8;   // Byte haut
            message[index++] = registres[i] & 0xFF; // Byte bas
        }
    } else if (bloc == 3) {
        uint16_t registres[] = {mb.Hreg(200), mb.Hreg(201), mb.Hreg(202), mb.Hreg(203), mb.Hreg(204),
                                mb.Hreg(205), mb.Hreg(206), mb.Hreg(207)};//, mb.Hreg(208), mb.Hreg(209)}; 

        int index = 2;
        
        for (int i = 0; i < 8; i++) {
            message[index++] = registres[i] & 0xFF; // Byte bas
            message[index++] = registres[i] >> 8;   // Byte haut
        }
    } else if (bloc == 4) {
        uint16_t registres[] = {mb.Hreg(210), mb.Hreg(211), mb.Hreg(212), mb.Hreg(213), mb.Hreg(214),
                                mb.Hreg(215), mb.Hreg(216), mb.Hreg(217)};//, mb.Hreg(218), mb.Hreg(219)}; 

        int index = 2;
        
        for (int i = 0; i < 8; i++) {
            message[index++] = registres[i] & 0xFF; // Byte bas
            message[index++] = registres[i] >> 8;   // Byte haut
        }
    } else if (bloc == 5) {
        uint16_t registres[] = {mb.Hreg(220), mb.Hreg(221), mb.Hreg(222), mb.Hreg(223), mb.Hreg(224),
                                mb.Hreg(225), mb.Hreg(226), mb.Hreg(227)};//, mb.Hreg(228), mb.Hreg(229)}; 

        int index = 2;
        
        for (int i = 0; i < 8; i++) {
            message[index++] = registres[i] & 0xFF; // Byte bas
            message[index++] = registres[i] >> 8;   // Byte haut
        }
    }  else if (bloc == 6) {
        uint16_t registres[] = {mb.Hreg(14), mb.Hreg(15), mb.Hreg(16), mb.Hreg(17), //not here 16 / 17 it was before
                                mb.Hreg(20), mb.Hreg(21), mb.Hreg(22), mb.Hreg(23)}; 

        int index = 2;
        
        for (int i = 0; i < 8; i++) {
            message[index++] = registres[i] >> 8;   // Byte haut
            message[index++] = registres[i] & 0xFF; // Byte bas
        } 
        // Serial.println(mb.Hreg(16));
        // Serial.println(mb.Hreg(17));
        // Serial.println(mb.Hreg(18));
        // Serial.println(mb.Hreg(19));
        // Serial.println(mb.Hreg(20));
        // Serial.println(mb.Hreg(21));
        // Serial.println(mb.Hreg(22));
        // Serial.println(mb.Hreg(23));
    } else if (bloc == 7) {
        uint16_t registres[] = {mb.Hreg(22), mb.Hreg(23)}; 

        int index = 2;
        
        for (int i = 0; i < 8; i++) {
            message[index++] = registres[i] >> 8;   // Byte haut
            message[index++] = registres[i] & 0xFF; // Byte bas
        } 
    } 
    pTxCharacteristic->setValue(message, BLE_SIZE); //it will send only 20 bytes 
    pTxCharacteristic->notify();
}


void checkMessage()
{
    
    uint8_t result = mb.Hreg(300) & 0x000F; 

if (result & 0x0001) { 
    Serial.println("ACTIVE VALVE 1");
    digitalWrite(D0, HIGH);
} else {
    digitalWrite(D0, LOW); 
}

if (result & 0x0002) { 
    Serial.println("ACTIVE VALVE 2");
    digitalWrite(D1, HIGH);
} else {
    digitalWrite(D1, LOW); 
}

if (result & 0x0004) { 
    Serial.println("ACTIVE VALVE 3");
    digitalWrite(D2, HIGH);
} else {
    digitalWrite(D2, LOW); 
}

if (result & 0x0008) { 
    Serial.println("ACTIVE VALVE 4");
    digitalWrite(D3, HIGH);
} else {
    digitalWrite(D3, LOW); // Désactive la pin si le bit n'est pas actif
}

// Lecture automatique des données dès qu'une connexion est établie

        Serial.println("Reading initial data...");
        sendBlockRegisters(3);
        sendBlockRegisters(4);
        sendBlockRegisters(5);
        //delay(25);
        sendBlockRegisters(0);
        //delay(25);
        sendBlockRegisters(1);
        sendBlockRegisters(6);

    
}


uint16_t _simulatedValue = 0;

// Function to convert a float to two 16-bit integers
void floatToRegisters(float value, uint16_t &reg1, uint16_t &reg2) {
  uint32_t temp = ((uint32_t)&value);
  reg1 = temp >> 16;
  reg2 = temp & 0xFFFF;
}

// Function to convert a string to an array of 16-bit registers
void stringToRegisters(const char* str, uint16_t* regs, int regCount) {
  for (int i = 0; i < regCount; i++) {
    if (i*2 < strlen(str)) {
      regs[i] = (str[i*2] << 8);
      if (i*2 + 1 < strlen(str)) {
        regs[i] |= str[i*2 + 1];
      }
    } else {
      regs[i] = 0;
    }
  }
}

unsigned long previousMillis = 0;
const unsigned long interval = 1000;  // interval at which to increment (milliseconds)

void setup() {
    Serial.begin(115200);
    pinMode(RXD2, INPUT_PULLUP);
    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);

    digitalWrite(D0, LOW);
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    digitalWrite(D3, LOW);

    swSer.enableRxGPIOPullUp(true);
    swSer.setTransmitEnablePin(RTS2);
    swSer.begin(19200, SWSERIAL_8E1, RXD2, TXD2, false, 64, 64);

    mb.begin(&swSer);
    mb.slave(SLAVE_ID);

    for (uint16_t i = REGN; i <= 305; i++) {
    mb.addHreg(i, 0); // Initialiser chaque registre a 0
}
    
    for (uint16_t i = REGN; i <= 305; i++) {
        mb.addHreg(i);
    }
    mb.Hreg(210, 0x00);

}

String ushortAsciiArrayToString(unsigned short* array, int size) {
    String result = "";
    for (int i = 0; i < size; i++) {
        char highByte = (char)((array[i] >> 8) & 0xFF);
        char lowByte = (char)(array[i] & 0xFF);
        result += lowByte;
        result += highByte;

    }
    return result;
  }

void startBluetooth() {

    uint16_t registres[] = {mb.Hreg(210), mb.Hreg(211), mb.Hreg(212), mb.Hreg(213)};
    if (registres[0] == 0x00) return; //probably its not starting because no values here
    String serialNumber  =  ushortAsciiArrayToString(registres, 4);
    String advertise_name = "fensorSCAN-";
    advertise_name += serialNumber;
    Serial.println(advertise_name);
    BLEDevice::init(advertise_name.c_str());
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new BleServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    BLEDescriptor cccdDescriptor(BLEUUID((uint16_t)0x2902));
    uint8_t cccdValue[2] = {0x01, 0x00};
    cccdDescriptor.setValue(cccdValue, 2);
    
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(&cccdDescriptor);

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new BleRxCallbacks());
    
    pService->start();
    pServer->getAdvertising()->start();
    Serial.println("Waiting for a client connection to notify...");
    pServer->startAdvertising();
    bluetooth_started = true;
}

   void loop() {
       unsigned long currentMillis = millis();
       mb.task();
    //   uint16_t registres[] = {mb.Hreg(210), mb.Hreg(211)};
    //   String serialNumber =  ushortAsciiArrayToString(registres, 2);
    //   String advertise_name = "fensorscan-";
    //   advertise_name += serialNumber;
    //    Serial.println(advertise_name);
       if (bluetooth_started && currentMillis - lastMillis >= interval) {
           lastMillis = currentMillis;
           checkMessage();
       } else if(!bluetooth_started) {
            startBluetooth();
       }

        // if (msgCounter++ > 5) {
        //     Serial.println("alive");
        //     msgCounter = 0;
        // }

        //mb.task();

        yield();
   }