/*
 * spi_gpio.c - SPI master driver using generic bitbanged GPIO
 *
 * Copyright (C) 2006,2008 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/spi_gpio.h>
#include <mach/sys_config.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#define DEVICE_NAME "io_emul_spi"
#define DRIVER_NAME "io_emul_spi"

struct spi_emul_platform_data
{
	u32     miso_handle;
	u32     mosi_handle;
	u32     sck_handle;
	u32     cs_handle;

	char*   pin_set;
	char*   miso_pin;
	char*   mosi_pin;
	char*   sck_pin;
	char*   cs_pin;
	
	int mode;
	int max_speed_hz;
	int bits_per_word;
};

/*
 * This bitbanging SPI master driver should help make systems usable
 * when a native hardware SPI engine is not available, perhaps because
 * its driver isn't yet working or because the I/O pins it requires
 * are used for other purposes.
 *
 * platform_device->driver_data ... points to spi_gpio
 *
 * spi->controller_state ... reserved for bitbang framework code
 * spi->controller_data ... holds chipselect GPIO
 *
 * spi->master->dev.driver_data ... points to spi_gpio->bitbang
 */

struct spi_gpio {
	struct spi_bitbang		bitbang;
	struct spi_emul_platform_data	pdata;
	struct platform_device		*pdev;
	struct spi_device		spi_dev;
};


static inline const struct spi_emul_platform_data * __pure
spi_to_pdata(const struct spi_device *spi)
{
	const struct spi_gpio		*spi_gpio;

	spi_gpio = container_of(spi, struct spi_gpio, spi_dev);
	return &spi_gpio->pdata;
}

/* this is #defined to avoid unused-variable warnings when inlining */
#define pdata		spi_to_pdata(spi)

static inline void setsck(const struct spi_device *spi, int is_on)
{
	gpio_write_one_pin_value(pdata->sck_handle, !!is_on, NULL);
}

static inline void setmosi(const struct spi_device *spi, int is_on)
{
	gpio_write_one_pin_value(pdata->mosi_handle, !!is_on, NULL);
}

static inline int getmiso(const struct spi_device *spi)
{
	return !!gpio_read_one_pin_value(pdata->miso_handle, NULL);
}

#undef pdata

/*
 * NOTE:  this clocks "as fast as we can".  It "should" be a function of the
 * requested device clock.  Software overhead means we usually have trouble
 * reaching even one Mbit/sec (except when we can inline bitops), so for now
 * we'll just assume we never need additional per-bit slowdowns.
 */
#define spidelay(nsecs)	do {} while (0)

#include "spi_bitbang_txrx.h"

/*
 * These functions can leverage inline expansion of GPIO calls to shrink
 * costs for a txrx bit, often by factors of around ten (by instruction
 * count).  That is particularly visible for larger word sizes, but helps
 * even with default 8-bit words.
 *
 * REVISIT overheads calling these functions for each word also have
 * significant performance costs.  Having txrx_bufs() calls that inline
 * the txrx_word() logic would help performance, e.g. on larger blocks
 * used with flash storage or MMC/SD.  There should also be ways to make
 * GCC be less stupid about reloading registers inside the I/O loops,
 * even without inlined GPIO calls; __attribute__((hot)) on GCC 4.3?
 */

static u32 spi_gpio_txrx_word_mode0(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, 0, word, bits);
}

static u32 spi_gpio_txrx_word_mode1(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, 0, word, bits);
}

static u32 spi_gpio_txrx_word_mode2(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, 0, word, bits);
}

static u32 spi_gpio_txrx_word_mode3(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, 0, word, bits);
}

/*
 * These functions do not call setmosi or getmiso if respective flag
 * (SPI_MASTER_NO_RX or SPI_MASTER_NO_TX) is set, so they are safe to
 * call when such pin is not present or defined in the controller.
 * A separate set of callbacks is defined to get highest possible
 * speed in the generic case (when both MISO and MOSI lines are
 * available), as optimiser will remove the checks when argument is
 * constant.
 */

static u32 spi_gpio_spec_txrx_word_mode0(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	unsigned flags = spi->master->flags;
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, flags, word, bits);
}

static u32 spi_gpio_spec_txrx_word_mode1(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	unsigned flags = spi->master->flags;
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, flags, word, bits);
}

static u32 spi_gpio_spec_txrx_word_mode2(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	unsigned flags = spi->master->flags;
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, flags, word, bits);
}

static u32 spi_gpio_spec_txrx_word_mode3(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	unsigned flags = spi->master->flags;
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, flags, word, bits);
}

/*----------------------------------------------------------------------*/

static void spi_gpio_chipselect(struct spi_device *spi, int is_active)
{
	/* set initial clock polarity */
	if (is_active)
		setsck(spi, spi->mode & SPI_CPOL);
	
	gpio_write_one_pin_value(spi_to_pdata(spi)->cs_handle, !is_active, NULL);
}

static int spi_gpio_setup(struct spi_device *spi)
{
	int		status = 0;

	if (spi->bits_per_word > 32)
		return -EINVAL;

	if (!status)
		status = spi_bitbang_setup(spi);

	return status;
}

static void spi_gpio_cleanup(struct spi_device *spi)
{
	spi_bitbang_cleanup(spi);
}

struct	spi_gpio	g_spi_gpio;

static unsigned bitbang_txrx_8(
		struct spi_device	*spi,
		u32			(*txrx_word)(struct spi_device *spi,
			unsigned nsecs,
			u32 word, u8 bits),
		unsigned		ns,
		struct spi_transfer	*t
		) {
	unsigned		bits = spi->bits_per_word;
	unsigned		count = t->len;
	const u8		*tx = t->tx_buf;
	u8			*rx = t->rx_buf;

	while (likely(count > 0)) {
		u8		word = 0;

		if (tx)
			word = *tx++;
		word = txrx_word(spi, ns, word, bits);
		if (rx)
			*rx++ = word;
		count -= 1;
	}
	return t->len - count;
}

static int do_transfer(struct spi_device *spi, struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	unsigned long	flags;
	unsigned char 	buf[16];
	
	const struct spi_gpio		*spi_gpio;
	spi_gpio = container_of(spi, struct spi_gpio, spi_dev);

	
	struct spi_bitbang* bitbang = &spi_gpio->bitbang;
	
	int i;
	unsigned		nsecs = 100;
	unsigned		tmp;
	int			status;


	bitbang->chipselect(spi, BITBANG_CS_ACTIVE);
	ndelay(nsecs);

	for(i = 0; i < n_xfers; i++) {

		struct spi_transfer t0, *t = &t0;
		t0.tx_buf = 0;
		t0.rx_buf = 0;
		t0.len = u_xfers[i].len;
		t0.cs_change = u_xfers[i].cs_change;
		t0.bits_per_word = u_xfers[i].bits_per_word;
		t0.delay_usecs = u_xfers[i].delay_usecs;
		t0.speed_hz = u_xfers[i].speed_hz;

		

	
		if (t->len > sizeof(buf)) {
			printk("transfer lenght too large , should be less than %d\n", sizeof(buf));
			break;	
		}
		if (u_xfers[i].rx_buf) {
			t->rx_buf = buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_xfers[i].rx_buf,
						t->len))
				break;
		}
		
		
		if (u_xfers[i].tx_buf) {
			t->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_xfers[i].tx_buf,
					t->len))
				break;
		}
		


		if (!t->speed_hz)
			t->speed_hz = spi->max_speed_hz;
		if (t->speed_hz) {
			nsecs = (1000000000/2) / t->speed_hz;
		}

		if (!t->tx_buf && !t->rx_buf && t->len) {
			status = -EINVAL;
			break;
		}


		
		if (t->len) {
			status = bitbang_txrx_8(spi, bitbang->txrx_word[spi->mode], nsecs, t);
		}
		


		status = 0;

		/* protocol tweaks before next transfer */
		if (t->delay_usecs)
			udelay(t->delay_usecs);
			
		if (u_xfers[i].rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_xfers[i].rx_buf, buf,
					t->len)) {
				status = -EFAULT;
				break;
			}
		}
		

	}
	
	ndelay(nsecs);
	bitbang->chipselect(spi, BITBANG_CS_INACTIVE);
	ndelay(nsecs);
	
	return i;
}

static ssize_t spidev_read(struct file *file, char __user *buf, size_t count,
		loff_t *offset)
{
	return 0;
}

static ssize_t spidev_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset)
{
	return 0;
}

static long spidev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned long funcs;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;
	
	int retval = 0;
	struct spi_device	*spi = file->private_data;

	switch (cmd) {
		default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}

		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* execute transfer*/
		retval = do_transfer(spi, ioc, n_ioc);
		
		kfree(ioc);
		break;
	}
	return retval;
}

static int spidev_open(struct inode *inode, struct file *file)
{
	file->private_data = &g_spi_gpio.spi_dev;
	return 0;
}

static int spidev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations spidev_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= spidev_read,
	.write		= spidev_write,
	.unlocked_ioctl	= spidev_ioctl,
	.open		= spidev_open,
	.release	= spidev_release,
};

static struct cdev *spi_cdev;
static dev_t devid ;
static struct class *spi_dev_class;

static int __init spi_gpio_probe(struct platform_device *pdev)
{
	int				status;
	struct spi_gpio			*spi_gpio = &g_spi_gpio;
	struct spi_emul_platform_data	*pdata;
	int ret;
	
	pdata = pdev->dev.platform_data;
	
	
	alloc_chrdev_region(&devid, 0, 1, DRIVER_NAME);
	spi_cdev = cdev_alloc();
	cdev_init(spi_cdev, &spidev_fops);
	spi_cdev->owner = THIS_MODULE;
	ret = cdev_add(spi_cdev, devid, 1);

	if (ret) {
		printk("failed to register character device /dev/io_emul_spi\n");
		return -1;
	}
	
	spi_dev_class = class_create(THIS_MODULE, DRIVER_NAME);
	device_create(spi_dev_class, NULL,
	              devid, NULL, DRIVER_NAME); 
	              
	
	pdata->miso_handle = gpio_request_ex(pdata->pin_set, pdata->miso_pin);
	pdata->mosi_handle = gpio_request_ex(pdata->pin_set, pdata->mosi_pin);
	pdata->sck_handle = gpio_request_ex(pdata->pin_set, pdata->sck_pin);
	pdata->cs_handle = gpio_request_ex(pdata->pin_set, pdata->cs_pin);

	if (!pdata->miso_handle || !pdata->mosi_handle || !pdata->sck_handle || !pdata->cs_handle) {
		printk("request gpio failed\n");
		status = -ENODEV;
		goto gpio_free;
	}


	gpio_set_one_pin_io_status(pdata->miso_handle, 0, NULL);
	gpio_set_one_pin_pull(pdata->miso_handle, 0 , NULL);

	gpio_set_one_pin_io_status(pdata->mosi_handle, 1, NULL);
	gpio_set_one_pin_pull(pdata->mosi_handle, 0, NULL);

	gpio_set_one_pin_io_status(pdata->sck_handle, 1, NULL);
	gpio_set_one_pin_pull(pdata->sck_handle, 0, NULL);

	gpio_set_one_pin_io_status(pdata->cs_handle, 1, NULL);
	gpio_set_one_pin_pull(pdata->cs_handle, 0, NULL);

	platform_set_drvdata(pdev, spi_gpio);

	spi_gpio->pdev = pdev;
	if (pdata)
		spi_gpio->pdata = *pdata;

	spi_gpio->bitbang.chipselect = spi_gpio_chipselect;

	spi_gpio->bitbang.txrx_word[SPI_MODE_0] = spi_gpio_txrx_word_mode0;
	spi_gpio->bitbang.txrx_word[SPI_MODE_1] = spi_gpio_txrx_word_mode1;
	spi_gpio->bitbang.txrx_word[SPI_MODE_2] = spi_gpio_txrx_word_mode2;
	spi_gpio->bitbang.txrx_word[SPI_MODE_3] = spi_gpio_txrx_word_mode3;

	spi_gpio->bitbang.flags = SPI_CS_HIGH;
	spi_gpio->spi_dev.bits_per_word = pdata->bits_per_word;
	spi_gpio->spi_dev.max_speed_hz = pdata->max_speed_hz;
	spi_gpio->spi_dev.mode = pdata->mode;
	
	spi_gpio_chipselect(&spi_gpio->spi_dev, BITBANG_CS_INACTIVE);
	
	return 0;
gpio_free:
	gpio_release(pdata->miso_handle, 0);
	gpio_release(pdata->mosi_handle, 0);
	gpio_release(pdata->sck_handle, 0);
	gpio_release(pdata->cs_handle, 0);	
	return status;
}

static int __exit spi_gpio_remove(struct platform_device *pdev)
{
	struct spi_gpio			*spi_gpio;
	struct spi_emul_platform_data	*pdata;
	int				status;

	gpio_release(pdata->miso_handle, 0);
	gpio_release(pdata->mosi_handle, 0);
	gpio_release(pdata->sck_handle, 0);
	gpio_release(pdata->cs_handle, 0);

	class_destroy(spi_dev_class);
	cdev_del(spi_cdev);
		
	return status;
}

MODULE_ALIAS("platform:" DRIVER_NAME);

static struct platform_driver spi_gpio_driver = {
	.driver.name	= DRIVER_NAME,
	.driver.owner	= THIS_MODULE,
	.probe		= spi_gpio_probe,
	.remove		= __exit_p(spi_gpio_remove),
};

static struct spi_emul_platform_data spi_emul_data  = {
	.pin_set = "mmc0_para",
	.miso_pin = "sdc_d0",	
	.mosi_pin = "sdc_d1",
	.sck_pin = "sdc_d2",	
	.cs_pin = "sdc_d3",
	.mode		= SPI_MODE_0,
	.max_speed_hz	= 1000000,
	.bits_per_word = 8,
};

struct platform_device spi_emul_device = {
	.name	= DEVICE_NAME,	
	.id	= 0,	
	.dev = { .platform_data = &spi_emul_data },
};

static int __init spi_emul_init(void)
{
	int ret = platform_device_register(&spi_emul_device);
	if (ret) {
		printk("failed to register spi emulation device!!\n");
		return -1;
	}
	ret = platform_driver_register(&spi_gpio_driver);
	if (ret) {
		platform_device_unregister(&spi_emul_device);
		printk("failed to register spi emulation driver!!\n");
		return -1;		
	}
	
	return 0;
}
module_init(spi_emul_init);

static void __exit spi_emul_exit(void)
{
	platform_driver_unregister(&spi_gpio_driver);
	platform_device_unregister(&spi_emul_device);
}
module_exit(spi_emul_exit);


MODULE_DESCRIPTION("SPI Emulation Module");
MODULE_AUTHOR("Derek Quan");
MODULE_LICENSE("GPL");
