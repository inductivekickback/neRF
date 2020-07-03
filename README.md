# neRF
Augments the Nerf Mega Mastodon blaster with a Nordic nRF52840, RGB LED, and vibration motor so it
becomes fly-by-wire and can optionally be controlled over Bluetooth. Without a Bluetooth connection the
blaster's original operation is essentially unchanged.

# Building and flashing on nRF52840 dongle
Enabling the log and USB console can prevent the application from starting if there is no USB
host (i.e. if running from the blaster's batteries instead of a USB port).

If the log isn't required then make sure that CONIFIG_LOG is not set and build as usual:
west build -b nrf52840_pca10059

Otherwise, enable CONFIG_LOG and overlay the usb_console.conf:
west build -b nrf52840_pca10059 -- -DOVERLAY_CONFIG=usb_console.conf

Then generate a DFU package:
nrfutil pkg generate --hw-version 52 --sd-req=0x00 --application build/zephyr/zephyr.hex --application-version 1 nerf.zip

Press reset switch while plugging in dongle to enter bootloader (red LED will pulse)

And perform the DFU:
nrfutil dfu usb-serial -pkg nerf.zip -p /dev/ttyACM0

Note that the nrfutil might need to be run with elevated privileges on some systems. Also, it's
common for the process to fail but it always eventually succeeds on successive tries.

