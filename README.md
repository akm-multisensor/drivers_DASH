# Development Environment
* Board : Dragonboard 410c
* Kernel : 3.10.49
* Interface : I2C

## Sample of dts
Add the following dts file to kernel directory.
The following sample assumes that the slave address is 0x0C, i.e. CAD0=CAD1=GND.

In the file 'arch/arm/boot/dts/qcom/apq8016-sbc.dtsi', AK8963 is already 
installed, so you need to modify a little bit to enable new device driver.
```Patch
                };
 
                akm@c {
-                       compatible = "ak,ak8963";
+                       compatible = "akm,ak09915";
                        reg = <0x0c>;
                        pinctrl-names = "ak8963_default", "ak8963_sleep";
                        pinctrl-0 = <&ak8963_default>;
```

In case of Dragonboard build script, .config file is created every re-compile
process. So I think modifying def_config file is one of the easiest way to 
enable new device driver.

```Patch
diff --git a/arch/arm64/configs/msm_defconfig b/arch/arm64/configs/msm_defconfig
index 67e6706..9b466d4 100644
--- a/arch/arm64/configs/msm_defconfig
+++ b/arch/arm64/configs/msm_defconfig
@@ -268,9 +268,14 @@ CONFIG_SENSORS_LIS3DH=y
 CONFIG_INPUT_UINPUT=y
 CONFIG_INPUT_GPIO=m
 CONFIG_INPUT_ADXL34X=y
+CONFIG_INPUT_ADXL34X_I2C=y
 CONFIG_SENSORS_MMC3416X=y
-CONFIG_SENSORS_AKM09911=y
-CONFIG_SENSORS_AKM8963=y
+# CONFIG_SENSORS_AKM09911 is not set
+# CONFIG_SENSORS_AKM8963 is not set
+# CONFIG_INPUT_AK0991X is not set
+# CONFIG_INPUT_AK0991X_OD is not set
+# CONFIG_INPUT_AK0991X_I2C is not set
+# CONFIG_INPUT_AK0991X_SPI is not set
 CONFIG_SENSORS_BMA2X2=y
 # CONFIG_SERIO_I8042 is not set
 # CONFIG_VT is not set
@@ -573,3 +578,13 @@ CONFIG_CRYPTO_AES_ARM64_CE_BLK=y
 CONFIG_CRYPTO_AES_ARM64_NEON_BLK=y
 CONFIG_QMI_ENCDEC=y
 CONFIG_STRICT_MEMORY_RWX=y
+CONFIG_USB_NET_AX8817X = y
+CONFIG_IIO=y
+CONFIG_IIO_BUFFER=y
+CONFIG_IIO_KFIFO_BUF=y
+CONFIG_IIO_TRIGGERED_BUFFER=y
+CONFIG_IIO_TRIGGER=y
+CONFIG_IIO_CONSUMERS_PER_TRIGGER=2
+CONFIG_IIO_AKM_MAGN_AK0991X=y
+CONFIG_IIO_AKM_MAGN_AK0991X_I2C=y
+CONFIG_IIO_AKM_MAGN_AK0991X_SPI=y
```

## iio driver
Modify Kconfig & Makefile in the directory 'drivers/iio/magnetometer'

```Patch
diff --git a/drivers/iio/magnetometer/Kconfig b/drivers/iio/magnetometer/Kconfig
index bd1cfb6..bac095c 100644
--- a/drivers/iio/magnetometer/Kconfig
+++ b/drivers/iio/magnetometer/Kconfig
@@ -55,4 +55,29 @@ config IIO_ST_MAGN_SPI_3AXIS
        depends on IIO_ST_MAGN_3AXIS
        depends on IIO_ST_SENSORS_SPI
 
+config IIO_AKM_MAGN_AK0991X
+    tristate "Asahi Kasei AK0991X 3-Axis Magnetometer"
+       select IIO_BUFFER
+       select IIO_TRIGGERED_BUFFER
+       help
+         Say yes here to build support for Asahi Kasei AK09911/AK09912
+         3-Axis Magnetometer.
+
+         To compile this driver as a module, choose M here: the module
+         will be called ak0991x.
+
+config IIO_AKM_MAGN_AK0991X_I2C
+    tristate "support I2C bus connection"
+       depends on IIO_AKM_MAGN_AK0991X && I2C
+       default y
+       help
+         Say y here is you have AK0991X hooked to an I2C bus.
+
+config IIO_AKM_MAGN_AK0991X_SPI
+    tristate "support SPI bus connection"
+       depends on IIO_AKM_MAGN_AK0991X && SPI
+       default y
+       help
+         Say y here if you have AK0991X hooked to an SPI bus.
+
 endmenu
diff --git a/drivers/iio/magnetometer/Makefile b/drivers/iio/magnetometer/Makefile
index 7f328e3..75169e9 100644
--- a/drivers/iio/magnetometer/Makefile
+++ b/drivers/iio/magnetometer/Makefile
@@ -11,3 +11,7 @@ st_magn-$(CONFIG_IIO_BUFFER) += st_magn_buffer.o
 
 obj-$(CONFIG_IIO_ST_MAGN_I2C_3AXIS) += st_magn_i2c.o
 obj-$(CONFIG_IIO_ST_MAGN_SPI_3AXIS) += st_magn_spi.o
+
+obj-$(CONFIG_IIO_AKM_MAGN_AK0991X) += ak0991x-iio.o
+obj-$(CONFIG_IIO_AKM_MAGN_AK0991X_I2C) += ak0991x-i2c.o
+obj-$(CONFIG_IIO_AKM_MAGN_AK0991X_SPI) += ak0991x-spi.o
```

## input driver
Modify Kconfig & Makefile in the directory 'drivers/input/misc'

```Patch
diff --git a/drivers/input/misc/Kconfig b/drivers/input/misc/Kconfig
index 99ceadf..b7ef9bd 100644
--- a/drivers/input/misc/Kconfig
+++ b/drivers/input/misc/Kconfig
@@ -649,6 +649,54 @@ config INPUT_ADXL34X_SPI
          To compile this driver as a module, choose M here: the
          module will be called adxl34x-spi.
 
+config INPUT_AK0991X
+       tristate "Asahi KASEI AK0991X 3-Axis Magnetometer"
+       default n
+       help
+         Say Y here if you have a Magnetometer interface using the
+         AK0991x controller, and your board-specific initialization
+         code includes that in its table of devices.
+
+         If you use this option, please say N for INPUT_AK0991X_OD.
+         This module will confilct with it.
+
+         To compile this driver as a module, choose M here: the
+         module will be called ak0991x.
+
+config INPUT_AK0991X_OD
+       tristate "Asahi KASEI AK0991X Magnetometer (Open Drain)"
+       default n
+       help
+         Say Y here if you have a Magnetometer interface using the
+         AK0991xD controller, and your board-specific initialization
+         code includes that in its table of devices.
+
+         If you use this option, please say N for INPUT_AK0991X.
+         This module will confilct with it.
+
+         To compile this driver as a module, choose M here: the
+         module will be called ak0991x.
+
+config INPUT_AK0991X_I2C
+       tristate "support I2C bus connection"
+       depends on INPUT_AK0991X && I2C
+       default y
+       help
+         Say Y here if you have AK0991x hooked to an I2C bus.
+
+         To compile this driver as a module, choose M here: the
+         module will be called ak0991x-i2c.
+
+config INPUT_AK0991X_SPI
+       tristate "support SPI bus connection"
+       depends on INPUT_AK0991X && SPI
+       default n
+       help
+         Say Y here if you have AK0991x hooked to a SPI bus.
+
+         To compile this driver as a module, choose M here: the
+         module will be called ak0991x-spi.
+
 config INPUT_IMS_PCU
        tristate "IMS Passenger Control Unit driver"
        depends on USB
diff --git a/drivers/input/misc/Makefile b/drivers/input/misc/Makefile
index 99a162f..03b7b7c 100644
--- a/drivers/input/misc/Makefile
+++ b/drivers/input/misc/Makefile
@@ -79,6 +79,13 @@ obj-$(CONFIG_SENSORS_BMA2X2) += bstclass.o
 obj-$(CONFIG_SENSORS_BMA2X2)   += bma2x2.o
 obj-$(CONFIG_SENSORS_LTR553)   += ltr553.o
 
+# akm added
+obj-$(CONFIG_INPUT_AK0991X)            += ak0991x.o
+obj-$(CONFIG_INPUT_AK0991X_OD)         += ak0991x-od.o
+obj-$(CONFIG_INPUT_AK0991X_I2C)                += ak0991x-i2c.o
+obj-$(CONFIG_INPUT_AK0991X_SPI)                += ak0991x-spi.o
+# end akm
+#
 ifeq ($(CONFIG_SENSORS_BMA2X2_ENABLE_INT1),y)
        EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT1
 endif
```
