The NCT7491 is a thermal monitor and fan controller.

It Provides:
 - on-chip temperature sensing
 - two remote temperature sensors (transistor-based)
 - three pwm fan control outputs
 - four tach monitoring inputs
 - automatic pwm fan speed control
 - thermal limit detection/protection
 - eight smbus thermal sensors

The [datasheet](http://www.onsemi.com/pub/Collateral/NCT7491-D.PDF) is openly available.

Build the driver:

    $ make KDIR=/path/to/kernel-sources/

or

    $ export KDIR=/path/to/kernel-sources/
    $ make
    $ make modules_install
