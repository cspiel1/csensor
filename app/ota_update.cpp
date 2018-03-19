#include "ota_update.h"
#include <rboot-api.h>

// download urls, set appropriately

static rBootHttpUpdate* otaUpdater = 0;

void otaUpdate_CallBack(rBootHttpUpdate& client, bool result) {
	(void) client;
	Serial.println("In callback...");
	if(result == true) {
		// success
		uint8_t slot;
		slot = rboot_get_current_rom();
		if (slot == 0) slot = 1; else slot = 0;
		// set to boot new rom and then reboot
		Serial.printf("Firmware updated, rebooting to rom %d...\r\n", slot);
		rboot_set_current_rom(slot);
		System.restart();
	} else {
		// fail
		Serial.println("Firmware update failed!");
	}
}

void otaUpdate(String v) {
	uint8_t slot;
	rboot_config bootconf;

	// need a clean object, otherwise if run before and failed will not run again
	if (otaUpdater) delete otaUpdater;
	otaUpdater = new rBootHttpUpdate();

	// select rom slot to flash
	bootconf = rboot_get_config();
	slot = bootconf.current_rom;
	if (slot == 0) slot = 1; else slot = 0;
	Serial.println(String("Slot") + slot);

	// flash rom to position indicated in the rBoot config rom table

    if (v.length()) {
        Serial.printf("Updating to "); Serial.println(v);
        Serial.println(String(ROM_0_URL) + "-" + v + ".bin");
        otaUpdater->addItem(bootconf.roms[slot], String(ROM_0_URL) + "-" + v + ".bin");
    } else {
        Serial.printf("Updating to default ");
        Serial.println(String(ROM_0_URL) + "-default.bin");
        otaUpdater->addItem(bootconf.roms[slot], String(ROM_0_URL) + "-default.bin");
    }

	// request switch and reboot on success
	//otaUpdater->switchToRom(slot);
	// and/or set a callback (called on failure or success without switching requested)
	otaUpdater->setCallback(otaUpdate_CallBack);

	// start update
	otaUpdater->start();
}

