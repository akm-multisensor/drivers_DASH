# Development Environment
* Board : OMAP5432 uEVM
* Kernel : 4.6.3-armv7-lpae-x1
* Interface : I2C

## Kernel source
The following wiki page describes the detail of applying new kernel.
https://eewiki.net/display/linuxonarm/OMAP5432+uEVM

## Sample of dts
Add the following dts file to kernel directory.
The following sample assumes that the slave address is 0x0C, i.e. CAD0=CAD1=GND.

arch/arm/boot/dts/omap5-akm-sensors.dtsi

## Patch kernel
Apply the patch to the kernel.

~~~
$ patch -p0 < omap5_4.6.3.patch
~~~

