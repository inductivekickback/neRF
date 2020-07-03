Augments the [Nerf Mega Mastodon](https://nerf.fandom.com/wiki/Mega_Mastodon) blaster with a [Nordic nRF52840 dongle](https://www.nordicsemi.com/Software-and-tools/Development-Kits/nRF52840-Dongle), RGB LED, and vibration motor so it becomes fly-by-wire and can optionally be controlled over Bluetooth. Without a Bluetooth connection the blaster's original operation is essentially unchanged.
### About
The Mastodon is an interesting blaster because of its unique combination of fully-electric operation and deterministic dart handling. When fully loaded, up to 24 darts can be fired individually without further interaction from the user. The current replacements for the Mastodon either require user interaction to fire each dart (e.g. [Ultra ONE](https://nerf.fandom.com/wiki/ONE)) or shoot at an unpredictable rate (e.g. [Prometheus MXVIII-20K](https://nerf.fandom.com/wiki/Prometheus_MXVIII-20K). A common characteristic that is shared by all of these automatic blasters is a ["flywheel" mechanism](https://nerf.fandom.com/wiki/Flywheel) that throws the dart in a similar fashion to baseball pitching machines.

Without a Bluetooth connection the blaster's operation is changed slightly:
* After firing 24 darts the blaster automatically stops shooting until the triggers are released and pressed again
* No dart will be fired until the wheels have had a second to spin up
* An RGB LED on the top of the blaster signals that the wheels have spun up (green), an error has occurred (blinking red), or a Bluetooth connection is active (blue)

Connecting to the blaster via Bluetooth adds the following functionality:
* The blaster's ammo count can be set and a notification can be sent after each dart is fired
* Three selective firing modes are available: single-shot, three-round burst, and fully automatic
* Both triggers can be individually overridden
* Both triggers can be locked out
* Haptic feedback can be initiated for a specified amount of time
### Building and flashing
This project is built from the v1.2.0 tag of [nRF Connect SDK](https://www.nordicsemi.com/Software-and-tools/Software/nRF-Connect-SDK).
### Building and flashing on nRF52840 dongle
Enabling the log and USB console can prevent the application from starting if there is no USB
host (i.e. if running from the blaster's batteries instead of a USB port).

If the log isn't required then make sure that CONIFIG_LOG is not set and build as usual:
west build -b nrf52840_pca10059

Otherwise, enable CONFIG_LOG and overlay the usb_console.conf:
west build -b nrf52840_pca10059 -- -DOVERLAY_CONFIG=usb_console.conf

Then generate a DFU package:
nrfutil pkg generate --hw-version 52 --sd-req=0x00 --application build/zephyr/zephyr.hex --application-version 1 nerf.zip

Press the reset switch while plugging in dongle to enter bootloader (red LED will pulse)

And perform the DFU:
nrfutil dfu usb-serial -pkg nerf.zip -p /dev/ttyACM0

Note that the nrfutil might need to be run with elevated privileges on some systems. Also, it's
common for the process to fail but it always eventually succeeds on successive tries.
