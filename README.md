# Development Environment
* Board : Beagle Bone Black Rev C
* Kernel : 3.14.37-ti-r57
* Interface : I2C

## Sample of dts
Add the following dts file to kernel directory.
The following sample assumes that the slave address is 0x0C, i.e. CAD0=CAD1=GND.

arch/arm/boot/dts/am335x-bone-akm-sensors.dtsi

## Patch kernel
Apply the patch to the kernel.

~~~
$ patch -p0 < bbb_3.14.37.patch
~~~

