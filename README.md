The leading developers of 4pi firmware are currently Kliment, sivu, bilsef, and midopple, though many others contribute with their patches.

This is a firmware for 4pi electronics setups. It supports printing from SD card, active heatbed control, and many other goodies.

This work is licensed under the GNU GPL v3 or (at the user's discretion) any later version.

It is a port of Sprinter firmware, which was licensed under GPL v3.

# Complete beginners guide #


From a fresh Ubuntu install how to update the firmware of your RepRap?
This clone is a generic configuration for a printer, you must edit it to match your machine.
Some details may not fit your hardware, be sure to check what you are doing.

## Linux ##

### Software installation ###

1. Install the required packages to compile the firmware. (Also install an IDE of choice or just use an editor.)
   ```
   $ sudo apt-get update
   $ sudo apt-get upgrade
   $ sudo apt-get install build-essential
   $ gcc -v
   $ make -v
   ```

2. Install an ATMEL capable flashing program.  
   [ATMEL SAM-BA](http://www.atmel.com/tools/ATMELSAM-BAIN-SYSTEMPROGRAMMER.aspx)
    -or-
   [BOSSA](http://sourceforge.net/projects/b-o-s-s-a/)

3. Clone the 4pi git repository. 
   ```
   git clone  https://github.com/kliment/4pi-firmware.git
   ```
   Optionally, switch to the desired branch
   ```
   git branch -a
   git checkout THE_BRANCH_YOU_WANT
   ```
  
### Firmware compilation and upload ###

4. Using your IDE of choice or text editor modify init_configuration.h in 4pi-firmware/src/
   ```
  **SPECIFICALLY CHECK/MODIFY THESE VALUES FOR YOUR MACHINE**
  define THERMISTORHEATER 11
  define THERMISTORBED 11
  define _AXIS_STEP_PER_UNIT {80, 80, 3200/0.8,700}
  define _AXIS_CURRENT {128, 128, 128, 128, 128}
  define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors
  define _X_MAX_LENGTH 200
  define _Y_MAX_LENGTH 200
  define _Z_MAX_LENGTH 100
  define _MAX_FEEDRATE {400, 400, 2, 45} // (mm/sec)
  define _HOMING_FEEDRATE {1500,1500,120} // (mm/min) !!
  define _ACCELERATION 1000 // Axis Normal acceleration mm/s^2
  define _RETRACT_ACCELERATION 2000 // Extruder Normal acceleration mm/s^2
  define _MAX_ACCELERATION_UNITS_PER_SQ_SECOND {5000,5000,50,5000} // X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
  ```

5. Compile the firmware by going to the *\src* directory and running *make*. This should create a *\bin* and *\obj* folder in the *\src* folder

6. Connect your 4pi to your computer via USB

7. Using your desired flashing software that you installed above, upload the firmware (found at \src\bin\Sprinter-4pi-at91sam3u4-flash.bin) to your 4pi.
When the flash is completed unplug the 4pi to power it down. Upon reboot the bootloader will be hidden. To access the bootloader to flash a newer firmware, you must short the *RESET* pads on top of the 4pi and power cycle again.



## Windows ##

### Software installation ###

1. Install 4pi drivers.  
   1A. [32bit Windows driver](https://raw.github.com/kliment/4pi-firmware/master/at91lib/usb/device/cdc-serial/drv/6119.inf)  
   1B. [64bit Windows driver](http://koti.kapsi.fi/~kliment/4pi/CDC_64bit.inf)

2. Install a compiler.  
   Some have had good success with [Sourcery Codeworkbench Lite](http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition/request?id=e023fac2-e611-476b-a702-90eabb2aeca8&downloadlite=scblite2012&fmpath=/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition/form)

3. Install an ATMEL capable flashing program.  
   [ATMEL SAM-BA](http://www.atmel.com/tools/ATMELSAM-BAIN-SYSTEMPROGRAMMER.aspx)
    -or-
   [BOSSA](http://sourceforge.net/projects/b-o-s-s-a/)

4. Download the firmware (or branch of the firmware).  
   from https://github.com/kliment/4pi-firmware


### Firmware compilation and upload ###

5. Using your IDE of choice or text editor modify init_configuration.h in 4pi-firmware\src\
   ```
  **SPECIFICALLY CHECK/MODIFY THESE VALUES FOR YOUR MACHINE**
  define THERMISTORHEATER 11
  define THERMISTORBED 11
  define _AXIS_STEP_PER_UNIT {80, 80, 3200/0.8,700}
  define _AXIS_CURRENT {128, 128, 128, 128, 128}
  define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors
  define _X_MAX_LENGTH 200
  define _Y_MAX_LENGTH 200
  define _Z_MAX_LENGTH 100
  define _MAX_FEEDRATE {400, 400, 2, 45} // (mm/sec)
  define _HOMING_FEEDRATE {1500,1500,120} // (mm/min) !!
  define _ACCELERATION 1000 // Axis Normal acceleration mm/s^2
  define _RETRACT_ACCELERATION 2000 // Extruder Normal acceleration mm/s^2
  define _MAX_ACCELERATION_UNITS_PER_SQ_SECOND {5000,5000,50,5000} // X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
   ```

6. Compile the firmware by using what ever compiler you chose to download (if using Codebench, the command line is *cs-make.exe*). This should create a *\bin* and *\obj* folder in the *\src* folder

7. Connect your 4pi to your computer via USB

8. Using your desired flashing software that you installed above, upload the firmware (found at *\src\bin\Sprinter-4pi-at91sam3u4-flash.bin*) to your 4pi.
When the flash is completed unplug the 4pi to power it down. Upon reboot the bootloader will be hidden. To access the bootloader to flash a newer firmware, you must short the *RESET* pads on top of the 4pi and power cycle again.

**Congratulations**, you have just upgraded the firmware of your 4pi!

---

You can use pronterface.py to do some manual verifications by moving the printer's tip along 
the axes and verifying that the physical displacements match the ones indicated on the interface. 

Special thanks to Kliment for all of his amazing work and efforts, and to the [Grbl project](http://github.com/grbl/grbl) for providing a smooth motion planner.
