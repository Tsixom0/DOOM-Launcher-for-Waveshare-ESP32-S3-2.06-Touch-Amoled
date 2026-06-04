#include <Arduino.h>
#include <SD_MMC.h>

extern "C" {
  #include <doomgeneric.h>
}

#include "pin_config.h"

static const char *SD_WAD_FS_PATH = "/doom1.wad";
static const char *SD_WAD_STDIO_PATH = "/sdcard/doom1.wad";
static const char *SAVE_DIR = "/sdcard/doomsaves";
static const char *SAVE_DIR_FS_PATH = "/doomsaves";

// ---------------- SD helpers ----------------

void printDirectory(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("[SD] Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("[SD] Failed to open directory");
    return;
  }

  if (!root.isDirectory()) {
    Serial.println("[SD] Not a directory");
    root.close();
    return;
  }

  File file = root.openNextFile();

  while (file) {
    if (file.isDirectory()) {
      Serial.print("[SD] DIR : ");
      Serial.println(file.name());

      if (levels) {
        printDirectory(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("[SD] FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }

    file = root.openNextFile();
  }

  root.close();
}

bool mountSDCard() {
  Serial.println("[SD] Mounting SD_MMC 1-bit at /sdcard...");

  // Your pin_config.h:
  // SDMMC_CLK  = 2
  // SDMMC_CMD  = 1
  // SDMMC_DATA = 3
  // SDMMC_CS   = 17
  //
  // In 1-bit SDMMC mode, CLK/CMD/DATA are used.
  // CS/D3 is not part of the 1-bit data path, but keeping it pulled up
  // can help some cards/boards.
  pinMode(SDMMC_CS, INPUT_PULLUP);

  SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);

  bool oneBitMode = true;
  bool formatIfMountFailed = false;
  int maxOpenFiles = 5;

  // SDMMC_FREQ_DEFAULT is faster.
  // If mounting fails, try changing this to SDMMC_FREQ_PROBING.
  if (!SD_MMC.begin("/sdcard", oneBitMode, formatIfMountFailed, SDMMC_FREQ_DEFAULT, maxOpenFiles)) {
    Serial.println("[SD] Mount failed with SDMMC_FREQ_DEFAULT.");
    Serial.println("[SD] Retrying with SDMMC_FREQ_PROBING...");

    SD_MMC.end();
    delay(250);

    if (!SD_MMC.begin("/sdcard", oneBitMode, formatIfMountFailed, SDMMC_FREQ_PROBING, maxOpenFiles)) {
      Serial.println("[SD] Mount failed.");
      return false;
    }
  }

  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("[SD] No SD card detected.");
    SD_MMC.end();
    return false;
  }

  Serial.print("[SD] Card type: ");

  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC/SDXC");
  } else {
    Serial.println("UNKNOWN");
  }

  Serial.printf("[SD] cardSize=%.2f MB\n", SD_MMC.cardSize() / 1024.0 / 1024.0);
  Serial.printf("[SD] total=%.2f MB used=%.2f MB\n",
                SD_MMC.totalBytes() / 1024.0 / 1024.0,
                SD_MMC.usedBytes() / 1024.0 / 1024.0);

  printDirectory(SD_MMC, "/", 1);

  return true;
}

bool checkSDWad() {
  Serial.printf("[WAD] Checking %s...\n", SD_WAD_STDIO_PATH);

  if (!SD_MMC.exists(SD_WAD_FS_PATH)) {
    Serial.println("[WAD] /doom1.wad missing on SD root.");
    Serial.println("[FIX] Put doom1.wad directly in the SD card root.");
    Serial.println("[FIX] Example: SD:/doom1.wad");
    return false;
  }

  File wad = SD_MMC.open(SD_WAD_FS_PATH, FILE_READ);
  if (!wad) {
    Serial.println("[WAD] Found /doom1.wad but could not open it.");
    return false;
  }

  char magic[5] = {0};
  size_t readBytes = wad.read((uint8_t *)magic, 4);
  size_t wadSize = wad.size();
  wad.close();

  Serial.printf("[WAD] Found %s size=%u bytes / %.2f MB magic=%s\n",
                SD_WAD_FS_PATH,
                (unsigned)wadSize,
                wadSize / 1024.0 / 1024.0,
                magic);

  if (readBytes != 4) {
    Serial.println("[WAD] Could not read WAD header.");
    return false;
  }

  if (strcmp(magic, "IWAD") != 0 && strcmp(magic, "PWAD") != 0) {
    Serial.println("[WAD] Bad magic. Expected IWAD/PWAD.");
    Serial.println("[FIX] Make sure this is a real doom1.wad file, not a zip/shortcut.");
    return false;
  }

  if (wadSize < 3UL * 1024UL * 1024UL) {
    Serial.println("[WAD] File looks too small for doom1.wad.");
    return false;
  }

  Serial.println("[WAD] SD doom1.wad OK");
  return true;
}

bool ensureSaveDir() {
  Serial.printf("[SAVE] Checking save directory: %s\n", SAVE_DIR);

  if (SD_MMC.exists(SAVE_DIR_FS_PATH)) {
    File dir = SD_MMC.open(SAVE_DIR_FS_PATH);
    if (dir && dir.isDirectory()) {
      dir.close();
      Serial.println("[SAVE] Save directory exists.");
      return true;
    }
    if (dir) dir.close();

    Serial.println("[SAVE] Path exists but is not a directory.");
    return false;
  }

  Serial.println("[SAVE] Save directory missing. Creating...");
  if (!SD_MMC.mkdir(SAVE_DIR_FS_PATH)) {
    Serial.println("[SAVE] Failed to create /doomsaves.");
    return false;
  }

  Serial.println("[SAVE] Created /doomsaves.");
  return true;
}

// ---------------- Arduino setup / loop ----------------

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(BTN_BOOT, INPUT_PULLUP);
  pinMode(BTN_PWR, INPUT_PULLUP);

  Serial.println();
  Serial.println("=================================");
  Serial.println(" DoomWatch_Run v1.6 SD ONLY + DOOMSAVES");
  Serial.println("=================================");
  Serial.printf("[SYS] Flash: %.2f MB\n", ESP.getFlashChipSize() / 1024.0 / 1024.0);
  Serial.printf("[SYS] PSRAM: %.2f MB\n", ESP.getPsramSize() / 1024.0 / 1024.0);
  Serial.printf("[SYS] Heap : %.2f KB\n", ESP.getFreeHeap() / 1024.0);

  if (!mountSDCard()) {
    Serial.println("[FATAL] SD card mount failed. Stopping.");
    while (true) {
      delay(1000);
    }
  }

  if (!checkSDWad()) {
    Serial.println("[FATAL] SD WAD check failed. Stopping.");
    while (true) {
      delay(1000);
    }
  }

  if (!ensureSaveDir()) {
    Serial.println("[FATAL] Save directory check failed. Stopping.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("[DOOM] Starting doomgeneric from SD card...");
  Serial.printf("[DOOM] WAD path : %s\n", SD_WAD_STDIO_PATH);
  Serial.printf("[DOOM] Save dir : %s\n", SAVE_DIR);
  Serial.println("[DOOM] GFX mode : rgb565");
  Serial.println("[DOOM] Music    : disabled");
  Serial.println("[DOOM] SFX      : enabled");

  // Sound effects enabled.
  // Music disabled.
  // Save dir is SD card /doomsaves folder.
  // Graphics mode is RGB565 for the optimized display backend.
  static char arg0[] = "doom";
  static char arg1[] = "-iwad";
  static char arg2[] = "/sdcard/doom1.wad";
  static char arg3[] = "-nomusic";
  static char arg4[] = "-savedir";
  static char arg5[] = "/sdcard/doomsaves";
  static char arg6[] = "-gfxmode";
  static char arg7[] = "rgb565";
  static char arg8[] = "-window";

  static char *argv[] = {
    arg0,
    arg1,
    arg2,
    arg3,
    arg4,
    arg5,
    arg6,
    arg7,
    arg8
  };

  int argc = sizeof(argv) / sizeof(argv[0]);

  doomgeneric_Create(argc, argv);

  Serial.println("[DOOM] doomgeneric_Create returned. Entering tick loop.");
}

void loop() {
  doomgeneric_Tick();

  // Yield to USB/RTOS watchdog.
  delay(0);
}
