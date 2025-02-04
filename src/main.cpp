#include "NimBLEDevice.h"
 
void setup() {
	Serial.begin(115200, SERIAL_8N1);
	Serial.printf("running");

    NimBLEDevice::init("");
    NimBLEScan *pScan = NimBLEDevice::getScan();
    NimBLEScanResults results = pScan->getResults(10 * 1000);
    NimBLEUUID service_HR_UUID(BLEUUID((uint16_t)0x180D));
	NimBLEUUID char_HR_UUID(BLEUUID((uint16_t)0x2A37));

    for (int i = 0; i < results.getCount(); i++) {
        const NimBLEAdvertisedDevice *device = results.getDevice(i);
        if (device->isAdvertisingService(service_HR_UUID)) {
            NimBLEClient *pClient = NimBLEDevice::createClient();
			Serial.printf("founded");
            if (!pClient) { // Make sure the client was created
				
                break;
            }
 
            if (pClient->connect(device)) {
                NimBLERemoteService *pService = pClient->getService(service_HR_UUID);
				

                if (pService != nullptr) {
                    NimBLERemoteCharacteristic *pCharacteristic = pService->getCharacteristic(char_HR_UUID);
 
                    if (pCharacteristic != nullptr) {
						Serial.printf("data ready");
						Serial.println((pCharacteristic->readValue()).size());
                        // print or do whatever you need with the value
                    }
                }
            } else {
                // failed to connect
				Serial.printf("failed to connect");
            }
 
            NimBLEDevice::deleteClient(pClient);
        }
    }
}

void loop()
{
  
}