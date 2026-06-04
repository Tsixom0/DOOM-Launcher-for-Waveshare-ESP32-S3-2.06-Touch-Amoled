#DOOM Watch Launcher

An SD-card-based DOOM launcher for ESP32-S3 touch AMOLED devices. Inspired by the classic DOOM.

##Features
Clean, touch-optimized AMOLED interface
Quick weapon switching via top-left tap

##Requirements
Hardware: www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-2.06
##SD Card: Formatted with FAT32, containing your doom1.wad in the root folder (/sdcard/doom1.wad)

#Libraries used:
Arduino_GFX_Library
Arduino_DriveBus_Library
doomgeneric library

#Controls
Launcher Controls
Button	Action
PWR	btn = Use
BOOT btn = Fire
In-Game Controls
Input	Action
Touch top-left corner	Cycle weapons
Touch top-right corner	Menu
Touch anywhere else (drag)	Move / Turn (invisible joystick)
BOOT + PWR together	Inject save-name macro

#Credits
DOOM Engine: Original DOOM and Chocolate Doom (doomgeneric)
Libraries:
Arduino GFX Library
Arduino DriveBus Library
XPowersLib
License

This project is MIT Licensed. See LICENSE for details.
