## Development Environment
* Board : Beagle Bone Black Rev C
* Kernel : 3.14.37-ti-r57
* Interface : I2C

### Sample of dts
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
