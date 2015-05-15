## Development Environment 1
* Board : Beagle Bone Black Rev C
* Kernel : 3.14.37-ti-r57
* Interface : I2C

### Sample of dts 1
Add the following dts file to kernel directory.

arch/arm/boot/dts/am335x-bone-akm-sensors.dtsi
```
/*
 * Copyright (C) 2015 Asahi Kasei Microdevices
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

&ocp {
	/* i2c */
	P9_17_pinmux {
		status = "disabled";
	};
	P9_18_pinmux {
		status = "disabled";
	};
	/* AKM irq pin */
	P9_23_pinmux {
		status = "disabled";
	};
	/* AKM reset pin */
	P9_27_pinmux {
		status = "disabled";
	};
};

&i2c1 {
	pinctrl-names = "default";
	pinctrl-0 = <&i2c1_pins>;

	status = "okay";
	clock-frequency = <400000>;

	ak09912: ak09912@c {
		status = "okay";
		compatible = "akm,ak09912";
		pinctrl-names = "default";
		pinctrl-0 = <&gpio1_17_pins>;

		reg = <0xc>;
		interrupt-parent = <&gpio1>;
		interrupts = <17 0>;

		axis_order_x = /bits/ 8 <0>;
		axis_order_y = /bits/ 8 <1>;
		axis_order_z = /bits/ 8 <2>;
		axis_sign_x = /bits/ 8 <0>;
		axis_sign_y = /bits/ 8 <0>;
		axis_sign_z = /bits/ 8 <0>;
	};
};

```
## Development Environment 2
* Board : Wandboard Quad
* Kernel : 3.10.53
* Interface : I2C

### Add BIN_ATTR definition
Old kernel does not have the definition of ```__BIN_ATTR```.
You need to add the following lines in ```ak0991x.c```. Theese lines are copied from the file ```include/linux/sysfs.h``` in Kernel 3.14 source.
```C
#ifndef __BIN_ATTR
#define __BIN_ATTR(_name, _mode, _read, _write, _size) {	\
	.attr = { .name = __stringify(_name), .mode = _mode }	\
	.read   = _read,					\
	.write  = _write					\
	.size   = _size,					\
}
#endif
```

### Sample of dts 2
Edit the following dts file in kernel directory.

arch/arm/boot/dts/imx6qdl-wandboard.dtsi
```patch
--- imx6qdl-wandboard.dtsi.org  2015-05-15 19:13:58.251174099 +0900
+++ imx6qdl-wandboard.dtsi      2015-05-15 15:36:34.387592637 +0900
@@ -285,6 +285,25 @@
                flip_x=<0>;
                flip_y=<0>;
        };
+
+       ak09912: ak09912@c {
+               status = "okay";
+               compatible = "akm,ak09912";
+               pinctrl-0 = <&pinctrl_gpio>;
+               pinctrl-names = "default";
+
+               reg = <0xc>;
+               gpios = <&gpio1 24 0>;
+               interrupt-parent = <&gpio1>;
+               interrupts = <24 0>;
+
+               axis_order_x = /bits/ 8 <0>;
+               axis_order_y = /bits/ 8 <1>;
+               axis_order_z = /bits/ 8 <2>;
+               axis_sign_x = /bits/ 8 <0>;
+               axis_sign_y = /bits/ 8 <0>;
+               axis_sign_z = /bits/ 8 <0>;
+       };
 };

 &i2c3 {
```
