#include <Arduino.h>
#include "settings.h"

#include "AudioPlayer.h"
#include "HallEffectSensor.h"
#include "Log.h"
#include "MemX.h"
#include "Port.h"
#include "Queues.h"
#include "Rfid.h"
#include "System.h"

#include <esp_task_wdt.h>

#if defined RFID_READER_TYPE_PN532_SPI || defined RFID_READER_TYPE_PN532_I2C
	#ifdef RFID_READER_TYPE_PN532_SPI
		#include <SPI.h>
        #include <PN532_SPI.h>
	#endif
	#if defined(RFID_READER_TYPE_PN532_I2C) || defined(PORT_EXPANDER_ENABLE)
		#include "Wire.h"
	#endif
	#ifdef RFID_READER_TYPE_PN532_I2C
        #include <PN532_I2C.h>

	#endif
    #include <PN532.h>

extern unsigned long Rfid_LastRfidCheckTimestamp;

#if (defined(I2C_2_ENABLE))
	extern TwoWire i2cBusTwo;
#endif

TaskHandle_t rfidTaskHandle;
static void Rfid_Task(void *parameter);

	#ifdef RFID_READER_TYPE_PN532_I2C
	
	#if !defined(I2C_2_ENABLE)
		#error "I2C_2_ENABLE must be defined for RFID_READER_TYPE_PN532_I2C"
	#endif
PN532_I2C pn532i2c(i2cBusTwo);
static PN532 pn532(pn532i2c);
	#endif

	#ifdef RFID_READER_TYPE_PN532_SPI
PN532_SPI pn532spi(SPI, RFID_CS);
static PN532 pn532(pn532spi);
	#endif

void Rfid_Init(void) {
    #ifdef RFID_READER_TYPE_PN532_SPI
	SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_CS);
	SPI.setFrequency(1000000);
	Log_Println("Initializing NFC (SPI)", LOGLEVEL_DEBUG);
	#endif
	

	#ifdef RFID_READER_TYPE_PN532_I2C
	pn532i2c.wakeup(); 	
	Log_Println("Initializing NFC (I2C)", LOGLEVEL_DEBUG);
	#endif

    #if defined(RFID_READER_TYPE_PN532_I2C) || defined(RFID_READER_TYPE_PN532_SPI)
    
	#ifdef PN532_ENABLE_LPCD
	// Check if wakeup-reason was card-detection (PN5180 only)
	// This only works if RFID.IRQ is connected to a GPIO and not to a port-expander
	esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
	if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
		Rfid_WakeupCheck();
	}

		// wakeup-check if IRQ is connected to port-expander, signal arrives as pushbutton
		#if (defined(PORT_EXPANDER_ENABLE) && (RFID_IRQ > 99))
	if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
		// read IRQ state from port-expander
		i2cBusTwo.begin(ext_IIC_DATA, ext_IIC_CLK);
		delay(50);
		Port_Init();
		uint8_t irqState = Port_Read(RFID_IRQ);
		if (irqState == LOW) {
			Log_Println("Wakeup caused by low power card-detection on port-expander", LOGLEVEL_NOTICE);
			Rfid_WakeupCheck();
		}
	}
		#endif

	// disable pin hold from deep sleep
	gpio_deep_sleep_hold_dis();
	gpio_hold_dis(gpio_num_t(RFID_CS)); // NSS
	gpio_hold_dis(gpio_num_t(RFID_RST)); // RST
		#if (RFID_IRQ >= 0 && RFID_IRQ <= MAX_GPIO)
	pinMode(RFID_IRQ, INPUT); // Not necessary for port-expander as for pca9555 all pins are configured as input per default
		#endif
	#endif

	
    uint32_t versiondata = pn532.getFirmwareVersion();
    if (! versiondata) {
        Log_Println(errorOccured, LOGLEVEL_DEBUG);
        //while (1); // halt
    }
	Log_Printf(LOGLEVEL_DEBUG, "NFC Reader found PN%i", (versiondata>>24) & 0xFF);
	Log_Printf(LOGLEVEL_DEBUG, "NFC Reader Firmware version %i.%i", (versiondata>>16) & 0xFF, (versiondata>>8) & 0xFF);

    // Set the max number of retry attempts to read from a card
    // This prevents us from waiting forever for a card, which is
    // the default behaviour of the PN532.
    pn532.setPassiveActivationRetries(0xFF);
  
    // configure board to read RFID tags
    pn532.SAMConfig();

	Log_Println(rfidScannerReady, LOGLEVEL_DEBUG);

	xTaskCreatePinnedToCore(
		Rfid_Task, /* Function to implement the task */
		"rfid", /* Name of the task */
		2048, /* Stack size in words */
		NULL, /* Task input parameter */
		2 | portPRIVILEGE_BIT, /* Priority of the task */
		&rfidTaskHandle, /* Task handle. */
		1 /* Core where the task should run */
	);
	#endif

}

void Rfid_Task(void *parameter) {
	#ifdef PAUSE_WHEN_RFID_REMOVED
	uint8_t control = 0x00;
	#endif	
	delay(500);

	for (;;) {
		if (RFID_SCAN_INTERVAL / 2 >= 20) {
			vTaskDelay(portTICK_PERIOD_MS * (RFID_SCAN_INTERVAL / 2));
		} else {
			vTaskDelay(portTICK_PERIOD_MS * 20);
		}
		byte cardId[cardIdSize];
		uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
		uint8_t cardLength;
		String cardIdString;
	#ifdef PAUSE_WHEN_RFID_REMOVED
		byte lastValidcardId[cardIdSize];
		bool cardAppliedCurrentRun = false;
		bool sameCardReapplied = false;
	#endif
		if ((millis() - Rfid_LastRfidCheckTimestamp) >= RFID_SCAN_INTERVAL) {
			// Log_Printf(LOGLEVEL_DEBUG, "%u", uxTaskGetStackHighWaterMark(NULL));

			Rfid_LastRfidCheckTimestamp = millis();
			// Reset the loop if no new card is present on the sensor/reader. This saves the entire process when idle.
			
			if(!pn532.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &cardLength, 100)) {
				continue;
			}
			

	#ifdef PAUSE_WHEN_RFID_REMOVED
			cardAppliedCurrentRun = true;
	#endif

	#ifndef PAUSE_WHEN_RFID_REMOVED
			// add stopping
	#endif
			memcpy(cardId, uid , cardIdSize);

	#ifdef HALLEFFECT_SENSOR_ENABLE
			cardId[cardIdSize - 1] = cardId[cardIdSize - 1] + gHallEffectSensor.waitForState(HallEffectWaitMS);
	#endif

	#ifdef PAUSE_WHEN_RFID_REMOVED
			if (memcmp((const void *) lastValidcardId, (const void *) cardId, sizeof(cardId)) == 0) {
				sameCardReapplied = true;
			}
	#endif
	
			String hexString;
			for (uint8_t i = 0u; i < cardIdSize; i++) {
				char str[4];
				snprintf(str, sizeof(str), "%02x%c", cardId[i], (i < cardIdSize - 1u) ? '-' : ' ');
				hexString += str;
			}
			Log_Printf(LOGLEVEL_NOTICE, rfidTagDetected, hexString.c_str());

			for (uint8_t i = 0u; i < cardIdSize; i++) {
				char num[4];
				snprintf(num, sizeof(num), "%03d", cardId[i]);
				cardIdString += num;
			}

	#ifdef PAUSE_WHEN_RFID_REMOVED
		#ifdef ACCEPT_SAME_RFID_AFTER_TRACK_END
			if (!sameCardReapplied || gPlayProperties.trackFinished || gPlayProperties.playlistFinished) { // Don't allow to send card to queue if it's the same card again if track or playlist is unfnished
		#else
			if (!sameCardReapplied) { // Don't allow to send card to queue if it's the same card again...
		#endif
				xQueueSend(gRfidCardQueue, cardIdString.c_str(), 0);
			} else {
				// If pause-button was pressed while card was not applied, playback could be active. If so: don't pause when card is reapplied again as the desired functionality would be reversed in this case.
				if (gPlayProperties.pausePlay && System_GetOperationMode() != OPMODE_BLUETOOTH_SINK) {
					AudioPlayer_TrackControlToQueueSender(PAUSEPLAY); // ... play/pause instead (but not for BT)
				}
			}
			memcpy(lastValidcardId, mfrc522.uid.uidByte, cardIdSize);
	#else
			xQueueSend(gRfidCardQueue, cardIdString.c_str(), 0); // If PAUSE_WHEN_RFID_REMOVED isn't active, every card-apply leads to new playlist-generation
	#endif
	
	#ifdef PAUSE_WHEN_RFID_REMOVED
			// https://github.com/miguelbalboa/rfid/issues/188; voodoo! :-)
			while (true) {
				if (RFID_SCAN_INTERVAL / 2 >= 20) {
					vTaskDelay(portTICK_PERIOD_MS * (RFID_SCAN_INTERVAL / 2));
				} else {
					vTaskDelay(portTICK_PERIOD_MS * 20);
				}
				control = 0;
				for (uint8_t i = 0u; i < 3; i++) {
					if(pn532.readPassiveTargetID(PN532_MIFARE_ISO14443A, cardId, &cardLength, 100)) {
						control |= 0x16;
						control += 0x1;
					}
					control += 0x4;
				}

				if (control == 13 || control == 14) {
					// card is still there
				} else {
					break;
				}
			}

			Log_Println(rfidTagRemoved, LOGLEVEL_NOTICE);
			if (!gPlayProperties.pausePlay && System_GetOperationMode() != OPMODE_BLUETOOTH_SINK) {
				AudioPlayer_TrackControlToQueueSender(PAUSEPLAY);
				Log_Println(rfidTagReapplied, LOGLEVEL_NOTICE);
			}
			//TODO: stop nfc
			cardAppliedCurrentRun = false;
	#endif
		}
	}
}


void Rfid_Cyclic(void) {
	// Not necessary as cyclic stuff performed by task Rfid_Task()
}

void Rfid_Exit(void) {
	#ifdef RFID_READER_TYPE_PN532_I2C 
	//TODO: power down nfc

	Log_Println("NFC powered down", LOGLEVEL_DEBUG);
	#endif
}

void Rfid_WakeupCheck(void) {
}
#endif