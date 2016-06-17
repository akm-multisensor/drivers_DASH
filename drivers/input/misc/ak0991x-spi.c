/* drivers/input/misc/ak0991x-spi.c - AK0991X compass driver
 *
 * Copyright (C) 2014 ASAHI KASEI MICRODEVICES CORPORATION.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*#define DEBUG*/
/*#define VERBOSE_DEBUG*/

#include <linux/ak0991x.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include "ak0991x_input.h"

#define MAX_SPI_FREQ_HZ		2500000
#define AK0991X_CMD_WRITE(addr)	(0x7F & (addr))
#define AK0991X_CMD_READ(addr)	(0x80 | (addr))

/***** SPI Tx/Rx operation ******************************************/
static int akspi_rxdata(struct device *dev, unsigned char *rxdata,
						int length)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char reg;
	int ret;

	reg = AK0991X_CMD_READ(rxdata[0]);

	ret = spi_write_then_read(spi, &reg, 1, rxdata, length);
	if (ret < 0) {
		dev_err(dev, "%s: transfer failed.", __func__);
		return ret;
	}

	dev_vdbg(dev, "RxData: len=%02x, addr=%02x  data=%02x",
		length, rxdata[0], rxdata[1]);
	return 0;
}

static int akspi_txdata(struct device *dev, unsigned char *txdata,
						int length)
{
	struct spi_device *spi = to_spi_device(dev);
	int ret;

	txdata[0] = AK0991X_CMD_WRITE(txdata[0]);

	ret = spi_write(spi, txdata, length);
	if (ret < 0) {
		dev_err(dev, "%s: transfer failed.", __func__);
		return ret;
	}

	dev_vdbg(dev, "TxData: len=%02x, addr=%02x data=%02x",
		length, txdata[0], txdata[1]);
	return 0;
}


static const struct ak0991x_bus_ops ak0991x_spi_bops = {
	.bustype = BUS_SPI,
	.rxdata  = akspi_rxdata,
	.txdata  = akspi_txdata,
};

/***** Probe/Remove function ****************************************/
static int ak0991x_spi_probe(struct spi_device *spi)
{
	struct ak0991x_data *akm;

	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_err(&spi->dev, "SPI CLK %d Hz too fast\n",
				spi->max_speed_hz);
		return -EINVAL;
	}

	akm = ak0991x_probe(&spi->dev, spi->irq, &ak0991x_spi_bops);
	if (IS_ERR(akm))
		return PTR_ERR(akm);

	/* Success */
	spi_set_drvdata(spi, akm);
	return 0;
}

static int ak0991x_spi_remove(struct spi_device *spi)
{
	struct ak0991x_data *akm = dev_get_drvdata(&spi->dev);
	return ak0991x_remove(akm);
}

/***** Power management *********************************************/
static int ak0991x_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct ak0991x_data *akm = dev_get_drvdata(&spi->dev);

	return ak0991x_suspend(akm);
}

static int ak0991x_spi_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct ak0991x_data *akm = dev_get_drvdata(&spi->dev);

	return ak0991x_resume(akm);
}

static const struct dev_pm_ops ak0991x_spi_pops = {
	.suspend	= ak0991x_spi_suspend,
	.resume		= ak0991x_spi_resume,
};

/***** SPI interface ***********************************************/
static struct spi_driver ak0991x_spi_driver = {
	.probe		= ak0991x_spi_probe,
	.remove		= ak0991x_spi_remove,
	.driver = {
		.name = AKM_DRIVER_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.pm = &ak0991x_spi_pops,
	},
};

static int __init ak0991x_spi_init(void)
{
	return spi_register_driver(&ak0991x_spi_driver);
}

static void __exit ak0991x_spi_exit(void)
{
	spi_unregister_driver(&ak0991x_spi_driver);
}

module_init(ak0991x_spi_init);
module_exit(ak0991x_spi_exit);

MODULE_AUTHOR("Asahi Kasei Microdevices Corp. <multi-s@om.asahi-kasei.co.jp>");
MODULE_DESCRIPTION("AK0991X SPI compass driver");
MODULE_LICENSE("GPL");
