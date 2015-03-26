## Development Environment
* Board : OMAP5 uEVM
* Kernel : 3.14.1
* Interface : I2C

### Sample of dts
```
&i2c2 {
	pinctrl-names = "default";
	pinctrl-0 = <&i2c2_pins>;

	clock-frequency = <400000>;

	ak09912: ak09912@c {
		compatible = "akm,ak09912";
		reg = <0xc>;
		interrupts = <7 IRQ_TYPE_EDGE_RISING>; /* gpio line 199 */
		interrupt-parent = <&gpio7>;
		axis_order_x = /bits/ 8 <0>;
		axis_order_y = /bits/ 8 <1>;
		axis_order_z = /bits/ 8 <2>;
		axis_sign_x = /bits/ 8 <0>;
		axis_sign_y = /bits/ 8 <0>;
		axis_sign_z = /bits/ 8 <0>;
	};
};
```
