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

// Callback receives raw data from RTU Device and send back to BLE Client 
Modbus::ResultCode cbRtuRaw(uint8_t *data, uint8_t len, void *custom)
{
  auto src = (Modbus::frame_arg_t *)custom;
  int total_message_size = 1 + len + 2; // address + modbus msg + crc
  uint8_t bleMessage[BLE_SIZE];
  uint8_t rtuResponse[256];

  if (total_message_size > 256)
  {
    return Modbus::EX_PASSTHROUGH;
  }

  rtuResponse[0] = src->slaveId;

  // message
  for (size_t i = 0; i < len; i++)
  {
    rtuResponse[1 + i] = data[i];
  }
  rtuResponse[len + 1] = data[len];
  rtuResponse[len + 2] = data[len + 1];
  Serial.printf("RTU Message Recived Slave: %d, Fn: %02X, len: %d", src->slaveId, data[0], len);
  int numberBlePackets = total_message_size / BLE_SIZE;
  int finalBlePacketSize = total_message_size % BLE_SIZE;

  int dataIndex;

  dataIndex = 0;
  for (size_t i = 0; i < numberBlePackets; i++)
  {
    for (size_t j = 0; j < BLE_SIZE; j++)
    {
      bleMessage[j] = rtuResponse[dataIndex];
      dataIndex++;
    }

    Serial.println("Notify BLE Device 20 bytes");
    pTxCharacteristic->setValue(bleMessage, BLE_SIZE);
    pTxCharacteristic->notify();
  }

  if (finalBlePacketSize > 0)
  {
    for (size_t i = 0; i < finalBlePacketSize; i++)
    {
      bleMessage[i] = rtuResponse[dataIndex];
      dataIndex++;
    }
    Serial.println("Notify BLE Device final packet");
    pTxCharacteristic->setValue(bleMessage, finalBlePacketSize);
    pTxCharacteristic->notify();
  }

  return Modbus::EX_PASSTHROUGH;
}

uint16_t readModbus() {
    uint16_t res[REG_COUNT];
    if (!mb.slave()) {
        mb.readHreg(SLAVE_ID, FIRST_REG, res, REG_COUNT, cb);
        while(mb.slave()) {
            mb.task();
            delay(10);
        }
        return(res[0]);
    }
    return 0;
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
    String nouvelleChaine = String(adresse) + ":" + String(valeur);
    sendMessageToSlave(nouvelleChaine);
}

void envoyerValeurRegistre0_1()
{
    uint16_t reg0 = mb.Hreg(0);
    uint16_t reg1 = mb.Hreg(1);

    uint32_t combined = ((uint32_t)reg0 << 16) | reg1;

    float valeur;
    memcpy(&valeur, &combined, sizeof(float));

    Serial.println(String(valeur));
    envoyerFloatMessageAvecAdresseValeur(6, valeur);
}

void envoyerValeurRegistre2_3()
{
    uint16_t reg2 = mb.Hreg(2);
    uint16_t reg3 = mb.Hreg(3);

    uint32_t combined = ((uint32_t)reg2 << 16) | reg3;

    float valeur;
    memcpy(&valeur, &combined, sizeof(float));

    Serial.println(String(valeur));
    envoyerFloatMessageAvecAdresseValeur(7, valeur);
}

void envoyerValeurRegistre4_5()
{
    uint16_t reg4 = mb.Hreg(4);
    uint16_t reg5 = mb.Hreg(5);

    uint32_t combined = ((uint32_t)reg4 << 16) | reg5;

    float valeur;
    memcpy(&valeur, &combined, sizeof(float));

    Serial.println(String(valeur));
    envoyerFloatMessageAvecAdresseValeur(2, valeur);
}

void envoyerValeurRegistre6_7()
{
    uint16_t reg6 = mb.Hreg(6);
    uint16_t reg7 = mb.Hreg(7);

    uint32_t combined = ((uint32_t)reg6 << 16) | reg7;

    float valeur;
    memcpy(&valeur, &combined, sizeof(float));

    Serial.println(String(valeur));
    envoyerFloatMessageAvecAdresseValeur(4, valeur);
}

void envoyerValeurRegistre8_9()
{
    uint16_t reg8 = mb.Hreg(8);
    uint16_t reg9 = mb.Hreg(9);

    uint32_t combined = ((uint32_t)reg8 << 16) | reg9;

    float valeur;
    memcpy(&valeur, &combined, sizeof(float));

    Serial.println(String(valeur));
    envoyerFloatMessageAvecAdresseValeur(5, valeur);
}

void envoyerValeurRegistre10_11()
{
    uint16_t reg10 = mb.Hreg(10);
    uint16_t reg11 = mb.Hreg(11);

    uint32_t combined = ((uint32_t)reg10 << 16) | reg11;

    float valeur;
    memcpy(&valeur, &combined, sizeof(float));

    Serial.println(String(valeur));
    envoyerFloatMessageAvecAdresseValeur(1, valeur);
}

void envoyerValeurRegistre12_13()
{
    uint16_t reg12 = mb.Hreg(12);
    uint16_t reg13 = mb.Hreg(13);

    uint32_t combined = ((uint32_t)reg12 << 16) | reg13;

    float valeur;
    memcpy(&valeur, &combined, sizeof(float));

    Serial.println(String(valeur));
    envoyerFloatMessageAvecAdresseValeur(3, valeur);
}


void envoyerValeurRegistre100()
{
    uint16_t reg100 = mb.Hreg(100);

    // Convertir la valeur du registre en une chaîne décimale
    String message = String(reg100);

    // Debug pour vérifier le message
    Serial.print("Message envoyé : ");
    Serial.println(message);

    // Envoyer la valeur brute du registre
    envoyerMessageAvecAdresseValeur(8, message.c_str());
}



void checkMessage()
{
    if (!_hasMessage) return;
    lastActivityTime = millis();
    Serial.println("Command received");
    _lastCommand.trim();
    if (_lastCommand == "READ") {
      envoyerValeurRegistre0_1();
      envoyerValeurRegistre2_3();
      envoyerValeurRegistre4_5();
      envoyerValeurRegistre6_7();
      envoyerValeurRegistre8_9();
      envoyerValeurRegistre10_11();
      envoyerValeurRegistre12_13();
      envoyerValeurRegistre100();
      return;
    }  
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
    swSer.enableRxGPIOPullUp(true);
    swSer.setTransmitEnablePin(RTS2);
    swSer.begin(19200, SWSERIAL_8E1, RXD2, TXD2, false, 64, 64);

    mb.begin(&swSer);
    mb.slave(SLAVE_ID);
    
    for (uint16_t i = REGN; i <= 220; i++) {
        mb.addHreg(i);
    }

    BLEDevice::init("fensorscan");
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
}

int msgCounter = 0;
void loop()
{
    // if (millis() - lastActivityTime > inactivityLimit && deviceConnected) {
    //      Serial.println("Last time checking...");
    //      pServer->disconnect(connId);
    //      Serial.println("Disconnected due to inactivity...");
    //      pServer->startAdvertising();
    // }
    checkMessage();
    if (msgCounter++ > 5) {
      Serial.println("alive");
       msgCounter = 0;
    }
    mb.task();
    yield();
    delay(100);
}