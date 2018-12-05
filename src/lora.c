//
// Created by marcel on 26.10.18.
//


#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include "lora.h"
#include <linux/poll.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include <linux/delay.h>

MODULE_AUTHOR("Marcel Schmidt <msc@maennl.info>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_DESCRIPTION("LoRa RFM95W");

#define RH_SPI_WRITE_MASK 0x80

#define PA_OUTPUT_PA_BOOST_PIN  1
#define PA_OUTPUT_RFO_PIN       0

#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LR_OCP               0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FEI_MSB                            0x28
#define REG_FEI_MID                            0x29
#define REG_FEI_LSB                            0x2A
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PaDac                 0x4d//add REG_PaDac

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80
#define RFO                      0x70
// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define ERR_INVALID_BIT_RANGE                 -11


#define GPIO_PIN_DI0 17

#define CDEV_FIFO_SIZE 1024

static DEFINE_MUTEX(lora_char_mutex);
static DECLARE_KFIFO(cdev_fifo, unsigned char, CDEV_FIFO_SIZE);
static DECLARE_WAIT_QUEUE_HEAD(cdev_wait);


static unsigned int irqNumber;
static bool di0_fired = false;
static bool sending = false;
static bool receiving = false;
static long current_bw=0;

static unsigned int dev_major = 0;
static struct class *lora_cdev_class;
static struct device *lora_cdev_device = NULL;

static ssize_t lora_cdev_write(struct file *filp,
                               const char __user *buf,
                               size_t count, loff_t
                               *f_pos);

static ssize_t lora_cdev_read(struct file *filp,
                              char __user *buf,
                              size_t count, loff_t
                              *f_pos);

static unsigned int lora_cdev_poll(struct file *, poll_table *);

static long lora_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static int lora_cdev_open(struct inode *inode, struct file *filp);

static int lora_cdev_release(struct inode *inode, struct file *filp);

static int lora_spi_init(void);

static void lora_free_spi_device(void);

static int lora_gpio_init(void);

static void lora_free_gpio(void);

static int lora_check_module(void);

static void lora_init_module(void);

static void lora_reset_module(void);

static size_t lora_transmit(const uint8_t *buffer, size_t size);

static void lora_read_register(unsigned char ch, unsigned char *rxBuffer, unsigned int rxBuffernLen);

static void lora_write_register(unsigned char *txBuffer, unsigned int txBuffernLen);

static void lora_write(unsigned char reg, unsigned char value);

static unsigned char lora_read(unsigned char reg);

static void lora_set_frequency(long frequency);

static unsigned char lora_get_version(void);

static void lora_set_opmode(unsigned char opmode);

static unsigned char lora_get_bytes_received(void);

static void lora_set_tx_power(int level, int outputPin);

static void lora_set_tx_power_max(int level);

static void lora_set_spreading_factor(int sf);

static void lora_set_signal_bandwidth(long sbw);
static int8_t lora_get_raw_snr(void);
static long lora_get_snr(void);

static int16_t lora_get_reg_value(uint8_t reg, uint8_t msb , uint8_t lsb );

static void lora_set_coding_rate4(int denominator);

static void lora_set_prereamble_length(long length);

static void lora_set_sync_word(int sw);

static int lora_begin_packet(bool implicitHeader);

static int lora_end_packet(void);

static void lora_enable_crc(void);

static void lora_disable_crc(void);

static unsigned char lora_random(void);

static void lora_set_explicit_header_mode(void);

static void lora_set_implicit_header_mode(void);

static void lora_idle(void);

static void lora_sleep(void);

static void lora_receive(int size);

static int lora_available(void);

static int lora_read_fifo_byte(void);

static void lora_package_received(char *buffer);

static int lora_parse_packet(int size);

static irq_handler_t lora_di0_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

static int8_t lora_get_rssi(void);

static const struct file_operations loradev_fops = {
        .owner =    THIS_MODULE,
        /* REVISIT switch to aio primitives, so that userspace
         * gets more complete API coverage.  It'll simplify things
         * too, except for the locking.
         */
        .write =    lora_cdev_write,
        .read =        lora_cdev_read,
        .unlocked_ioctl = lora_cdev_ioctl,
        .compat_ioctl = NULL,
        .open =        lora_cdev_open,
        .release =    lora_cdev_release,
        .llseek =    no_llseek,
        .poll = lora_cdev_poll,
};

static struct spi_device *spi_device = NULL;
static unsigned int SPI_LORA_BUS_NUM = 0;
static unsigned int RST_PIN = 6;
static long LORA_BAND = 868E6;
static bool _implicitHeaderMode = false;
static int _packetIndex = 0;
static bool enableDebug = false;
static int loglevel = 2;

static int __init

lora_init(void) {
    int result = 0;

    pr_info("LoRa SPI Kernel Driver by Männl Elektronik GmbH\n");


    result = register_chrdev(dev_major, "rfm95w", &loradev_fops);
    if (result < 0)
        return result;

    dev_major = (unsigned int) result;

    pr_info("LoRA Char-Dev: major number %d\n", dev_major);

    // Device-Klasse anmelden
    lora_cdev_class = class_create(THIS_MODULE, "lora");
    if (IS_ERR(lora_cdev_class)) {
        unregister_chrdev(dev_major, "rfm95w");
        return PTR_ERR(lora_cdev_class);
    }

    // Device-Treiber anlegen
    lora_cdev_device = device_create(lora_cdev_class, NULL, MKDEV(dev_major, 0), NULL, "rfm95w");
    if (IS_ERR(lora_cdev_device)) {
        class_unregister(lora_cdev_class);
        class_destroy(lora_cdev_class);
        unregister_chrdev(dev_major, "rfm95w");
        pr_alert("CDR failed to create a device\n");
        return PTR_ERR(lora_cdev_device);
    }

    /*
    lora_class = class_create(THIS_MODULE, "rfm95w");
    if (IS_ERR(lora_class)) {
        unregister_chrdev(dev_major, "rfm95w");
        return PTR_ERR(lora_class);
    }
*/

    result = lora_spi_init();
    if (result < 0) {
        device_destroy(lora_cdev_class, MKDEV(dev_major, 0));
        class_unregister(lora_cdev_class);
        class_destroy(lora_cdev_class);
        unregister_chrdev(dev_major, "rfm95w");
        return result;
    }

    result = lora_gpio_init();
    if (result < 0) {
        lora_free_spi_device();
        device_destroy(lora_cdev_class, MKDEV(dev_major, 0));
        class_unregister(lora_cdev_class);
        class_destroy(lora_cdev_class);
        unregister_chrdev(dev_major, "rfm95w");
        return result;
    }

/*
    result = spi_register_driver(&lora_spi_driver);
    if (result < 0) {
        class_destroy(lora_class);
        unregister_chrdev(dev_major, lora_spi_driver.driver.name);
    }
*/

    lora_reset_module();

    result = lora_check_module();
    if (result < 0) {
        lora_free_spi_device();
        device_destroy(lora_cdev_class, MKDEV(dev_major, 0));
        class_unregister(lora_cdev_class);
        class_destroy(lora_cdev_class);
        unregister_chrdev(dev_major, "rfm95w");
        return result;
    }

    lora_init_module();

    mutex_init(&lora_char_mutex);

    INIT_KFIFO(cdev_fifo);

    pr_info("Modul bereit...");

    return 0;
}

static void __exit lora_exit(void) {
//    spi_unregister_driver(&lora_spi_driver);
    lora_free_spi_device();
    lora_free_gpio();
    // class_destroy(lora_class);

    device_destroy(lora_cdev_class, MKDEV(dev_major, 0));
    class_unregister(lora_cdev_class);
    class_destroy(lora_cdev_class);
    unregister_chrdev(dev_major, "rfm95w");

    mutex_destroy(&lora_char_mutex);

    pr_info("LoRa SPI Kernel Driver unloaded\n");
}

static ssize_t lora_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
    printk("lora_write");
    pr_info("Lora: Received %zu characters from the user, transmitting...\n", count);
    pr_info("Lora: %s", buf);
    lora_begin_packet(false);
    lora_transmit(buf, count);
    lora_end_packet();
    lora_receive(0);
    return count;
}

static ssize_t lora_cdev_read(struct file *filp, char __user *buf,
                              size_t count, loff_t
                              *f_pos) {

    int retval;
    unsigned int copied = 0;

    if (kfifo_is_empty(&cdev_fifo)) {
        return 0;
    }

    retval = kfifo_to_user(&cdev_fifo, buf, kfifo_len(&cdev_fifo), &copied);

    if (retval == 0) {
        pr_info("CDR: Sent %d characters to the user\n", copied);
    }

    return retval ? retval : copied;

}

static int lora_begin_packet(bool implicitHeader) {
    unsigned char irqFlags;

    sending = true;

    // put in standby mode
    lora_idle();

    // IRQ.Flags resetten
    irqFlags = lora_read(REG_IRQ_FLAGS);
    pr_info("BP: Flags %x", irqFlags);
    lora_write(REG_IRQ_FLAGS, 0x00);

    if (implicitHeader) {
        lora_set_implicit_header_mode();
    } else {
        lora_set_explicit_header_mode();
    }
    // reset FIFO address and paload length
    lora_write(REG_FIFO_ADDR_PTR, 0);
    lora_write(REG_PAYLOAD_LENGTH, 0);
    return 1;
}

static int lora_end_packet(void) {
    unsigned char flags;
    long loops = 50;
    // put in TX mode
    lora_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    // wait for TX done
    flags = lora_read(REG_IRQ_FLAGS);
    while (((flags & IRQ_TX_DONE_MASK) == 0) && (loops > 0) && (sending)) {
        msleep(5);
        flags = lora_read(REG_IRQ_FLAGS);
        loops--;
    };
    // clear IRQ's
    lora_write(REG_IRQ_FLAGS, (unsigned char) (flags & ~IRQ_TX_DONE_MASK));
    sending = false;
    return 1;
}

static size_t lora_transmit(const uint8_t *buffer, size_t size) {
    size_t
    i;
    int currentLength = lora_read(REG_PAYLOAD_LENGTH);
    // check size
    if ((currentLength + size) > MAX_PKT_LENGTH) {
        size = (size_t) (MAX_PKT_LENGTH - currentLength);
    }
    // write data
    for (i = 0; i < size; i++) {
        lora_write(REG_FIFO, buffer[i]);
    }
    // update length
    lora_write(REG_PAYLOAD_LENGTH, (unsigned char) (currentLength + size));
    return size;
}

static long lora_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    const char *buffer = "Testnachricht";
    int retval = -1;
    int size = 0;

    if (_IOC_TYPE(cmd) != LORA_CMD_BASE) return -EINVAL;

    switch (cmd) {
        case LORA_CMD_MODULE_RESET:
            lora_reset_module();
            retval = 0;
            break;
        case LORA_CMD_MODULE_VERSION:
            retval = put_user(lora_get_version(), (__u8 __user *) arg);
            break;
        case LORA_CMD_MODULE_INIT:
            lora_init_module();
            retval = 0;
            break;
        case LORA_CMD_SLEEP:
            lora_sleep();
            retval = 0;
            break;
        case LORA_CMD_IDLE:
            lora_idle();
            retval = 0;
            break;
        case LORA_CMD_RECEIVE:
            size = 0;
            retval = get_user(size, (u16 __user *) arg);
            if (retval == 0) {
                lora_receive(size);
            } else {
                lora_receive(0);
            }
            break;
        case LORA_CMD_GET_PACKAGE:
            if (arg != 0) {
                lora_package_received(((package *) arg)->buffer);
                ((package *) arg)->rssi = lora_get_rssi();
                ((package *) arg)->snr = lora_get_snr();
                retval = 0;
            }
            break;
        case LORA_CMD_PARSE_PACKAGE:
            size = 0;

            retval = get_user(size, (u16 __user *) arg);
            printk("Search4: %i", size);

            if (retval == 0) {
                size = lora_parse_packet(size);
                printk("Paketgröße A: %i", size);
            } else {
                size = lora_parse_packet(0);
                printk("Paketgröße B: %i", size);
            }
            retval = put_user(size, (__u16 __user *) arg);
            break;
        case LORA_CMD_DI0_FIRED:
            retval = put_user(di0_fired, (bool __user *) arg);
            break;
        case LORA_CMD_DATA_AVAILABLE:
            retval = put_user(lora_available(), (__u16 __user *) arg);
            break;
        case LORA_CMD_TRANSMIT:
            lora_begin_packet(false);
            lora_transmit(buffer, 13);
            lora_end_packet();
            lora_receive(0);
            retval = 0;
            break;
        default:
            retval = -1;
    }


    return retval;
}

static int lora_cdev_open(struct inode *inode, struct file *filp) {
    int status = 0;
    if (!mutex_trylock(&lora_char_mutex)) {
        pr_alert("LORA: Device in use!");
        return -EBUSY;
    }

    lora_reset_module();
    lora_init_module();

    return status;
}

static int lora_cdev_release(struct inode *inode, struct file *filp) {
    mutex_unlock(&lora_char_mutex);
    return 0;
}


static unsigned int lora_cdev_poll(struct file *filp, poll_table *wait) {
    if (!receiving && !sending) {
        lora_receive(0);
    }
    poll_wait(filp, &cdev_wait, wait);
    if (kfifo_len(&cdev_fifo) > 0) {
        return POLLIN | POLLRDNORM | POLLPRI;
    }
    return 0;
}

static int lora_spi_init(void) {

    struct spi_master *master;
    int result = 0;

    struct spi_board_info spi_device_info = {
            .modalias = "lora-spi-driver",
            .max_speed_hz = (u32) 4E6, //speed your device (slave) can handle
            .bus_num = (u16) SPI_LORA_BUS_NUM,
            .chip_select = 0,
            .mode = SPI_MODE_0,
    };

    master = spi_busnum_to_master(spi_device_info.bus_num);
    if (!master) {
        printk("SPI-MASTER not found.\n");
        return -ENODEV;
    }

    spi_device = spi_new_device(master, &spi_device_info);

    if (!spi_device) {
        printk("FAILED to create spi slave.\n");
        return -ENODEV;
    }

    spi_device->bits_per_word = 8;

    result = spi_setup(spi_device);

    if (result) {
        printk("FAILED to setup spi slave.\n");
        spi_unregister_device(spi_device);
        return -ENODEV;
    }

    return 0;
}

static void lora_free_spi_device(void) {
    if (spi_device != NULL) {
        spi_unregister_device(spi_device);
        spi_device = NULL;
    }
}

static irq_handler_t lora_di0_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
    char x = 0x01;
#ifdef  GPIO_PIN_DI0
    if (!sending) {


        printk("Receiving-DI0!");
        di0_fired = true;
        printk("V: %i", gpio_get_value(GPIO_PIN_DI0));
        if (kfifo_avail(&cdev_fifo) > 0) {
            pr_info("Sende ein Byte in FIFO\n");

            kfifo_in(&cdev_fifo, &x, 1);
            wake_up_interruptible(&cdev_wait);
        } else {
            pr_info("LORA: Kein Platz im FIFO\n");
        }
    } else {
        printk("Sending-DI0!");
        sending = false;
    }
#endif
    return (irq_handler_t) IRQ_HANDLED;
}

static int lora_gpio_init(void) {
    int result;

    if (enableDebug && loglevel > 1) {
        printk("RST-PIN: %i\n", RST_PIN);
    }

    result = gpio_request(RST_PIN, "RST-Pin");
    if (result != 0) {
        printk("RST-Pin nicht verfügbar.\n");
        return -ENODEV;
    }

    gpio_direction_output(RST_PIN, 1);
    gpio_export(RST_PIN, false);

#ifdef GPIO_PIN_DI0
    result = gpio_request(GPIO_PIN_DI0, "DI0-Pin");
    if (result != 0) {
        printk("DI0-Pin nicht verfügbar.\n");
        return -ENODEV;
    }
    gpio_direction_input(GPIO_PIN_DI0);
    gpio_export(GPIO_PIN_DI0, false);

    irqNumber = (unsigned int) gpio_to_irq(GPIO_PIN_DI0);

    result = request_irq(irqNumber,
                         (irq_handler_t) lora_di0_handler,
                         IRQF_TRIGGER_RISING,
                         "lora_di0_handler",
                         NULL);


#endif

    return 0;
}

static void lora_free_gpio(void) {
    gpio_unexport(RST_PIN);
    gpio_free(RST_PIN);
#ifdef GPIO_PIN_DI0
    free_irq(irqNumber, NULL);
    gpio_unexport(GPIO_PIN_DI0);
    gpio_free(GPIO_PIN_DI0);
#endif
}

static void lora_init_module(void) {
    lora_sleep();


    lora_set_frequency(LORA_BAND);


    lora_write(REG_FIFO_TX_BASE_ADDR, 0);
    lora_write(REG_FIFO_RX_BASE_ADDR, 0);


    // set LNA boost
    lora_write(REG_LNA, (unsigned char) (lora_read(REG_LNA) | 0x03));
    // set auto AGC
    lora_write(REG_MODEM_CONFIG_3, 0x04);
    // set output power to 12 dBm
    lora_set_tx_power_max(20);  //PA_BOOST
    // set Spreading Factor to 7 (6~12)
    lora_set_spreading_factor(9);
    // put in standby mode
    lora_set_signal_bandwidth(125E3);
    lora_set_coding_rate4(5);
    lora_set_sync_word(0x18);
    lora_set_prereamble_length(32);


    lora_enable_crc();

    lora_write(REG_DIO_MAPPING_1, 0x00);

    lora_idle();
    pr_info("Modul initialisiert.");
}

static void lora_reset_module(void) {
    if (enableDebug && loglevel > 1) {
        printk("Modul-Reset");
    }
    gpio_set_value(RST_PIN, 0);
    usleep_range(1000, 10000);
    gpio_set_value(RST_PIN, 1);
    usleep_range(10000, 20000);
}


static void lora_read_register(unsigned char ch, unsigned char *rxBuffer, unsigned int rxBuffernLen) {
    if (spi_device != NULL) {
        spi_write_then_read(spi_device, &ch, sizeof(ch), rxBuffer, rxBuffernLen);
    }
}

static void lora_write_register(unsigned char *txBuffer, unsigned int txBuffernLen) {
    if (spi_device != NULL) {
        if (enableDebug && loglevel > 10) {
            printk("Writing: %x %x", txBuffer[0], txBuffer[1]);
        }
        spi_write(spi_device, txBuffer, txBuffernLen);
    }
}

static void lora_write(unsigned char reg, unsigned char value) {
    unsigned char ch[2] = {(unsigned char) (reg | RH_SPI_WRITE_MASK), value};

    lora_write_register((unsigned char *) &ch, 2);
}

static unsigned char lora_read(unsigned char reg) {
    unsigned char rx = 0x0;
    lora_read_register((unsigned char) (reg & 0x7f), &rx, 1);
    return rx;
}

static void lora_set_frequency(long frequency) {
    uint64_t frf = ((uint64_t) frequency << 19) / 32000000;
    lora_write(REG_FRF_MSB, (uint8_t) (frf >> 16));
    lora_write(REG_FRF_MID, (uint8_t) (frf >> 8));
    lora_write(REG_FRF_LSB, (uint8_t) (frf >> 0));
}

static unsigned char lora_get_version(void) {
    return lora_read(REG_VERSION);
}


static void lora_set_opmode(unsigned char opmode) {
    lora_write(REG_OP_MODE, opmode);
}

static unsigned char lora_get_bytes_received(void) {
    unsigned char rx = 0x0;
    lora_read_register(0x13, &rx, 1);
    return rx;
}

static void lora_set_tx_power(int level, int outputPin) {
    if (PA_OUTPUT_RFO_PIN == outputPin) {
        // RFO
        if (level < -1) {
            level = -1;
        } else if (level > 14) {
            level = 14;
        }
        lora_write(REG_PaDac, 0x84);
        lora_write(REG_PA_CONFIG, (unsigned char) (RFO | (level + 1)));
    } else {
        // PA BOOST
        if (level < 2) {
            level = 2;
        } else if (level > 17) {
            level = 17;
        }
        //writeRegister(REG_LR_OCP,0x3f);
        lora_write(REG_PaDac, 0x84);
        lora_write(REG_PA_CONFIG, (unsigned char) (PA_BOOST | (level - 2)));


    }
}


static void lora_set_tx_power_max(int level) {
    if (level < 5) {
        level = 5;
    } else if (level > 20) {
        level = 20;
    }
    lora_write(REG_LR_OCP, 0x3f);
    lora_write(REG_PaDac, 0x87);//Open PA_BOOST
    lora_write(REG_PA_CONFIG, (unsigned char) (PA_BOOST | (level - 5)));
}


static void lora_set_spreading_factor(int sf) {
    if (sf < 6) {
        sf = 6;
    } else if (sf > 12) {
        sf = 12;
    }
    if (sf == 6) {
        lora_write(REG_DETECTION_OPTIMIZE, 0xc5);
        lora_write(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        lora_write(REG_DETECTION_OPTIMIZE, 0xc3);
        lora_write(REG_DETECTION_THRESHOLD, 0x0a);
    }
    lora_write(REG_MODEM_CONFIG_2, (unsigned char) ((lora_read(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0)));
}

static void lora_set_signal_bandwidth(long sbw) {
    int bw;

    if (sbw <= 7.8E3) { bw = 0; }
    else if (sbw <= 10.4E3) { bw = 1; }
    else if (sbw <= 15.6E3) { bw = 2; }
    else if (sbw <= 20.8E3) { bw = 3; }
    else if (sbw <= 31.25E3) { bw = 4; }
    else if (sbw <= 41.7E3) { bw = 5; }
    else if (sbw <= 62.5E3) { bw = 6; }
    else if (sbw <= 125E3) { bw = 7; }
    else if (sbw <= 250E3) { bw = 8; }
    else { bw = 9; }
    current_bw=sbw;
    lora_write(REG_MODEM_CONFIG_1, (unsigned char) ((lora_read(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4)));
}

static void lora_set_coding_rate4(int denominator) {
    int cr;
    if (denominator < 5) {
        denominator = 5;
    } else if (denominator > 8) {
        denominator = 8;
    }
    cr = denominator - 4;
    lora_write(REG_MODEM_CONFIG_1, (unsigned char) ((lora_read(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1)));
}

static void lora_set_prereamble_length(long length) {
    lora_write(REG_PREAMBLE_MSB, (uint8_t) (length >> 8));
    lora_write(REG_PREAMBLE_LSB, (uint8_t) (length >> 0));
}

static void lora_set_sync_word(int sw) {
    lora_write(REG_SYNC_WORD, (unsigned char) sw);
}

static void lora_enable_crc(void) {
    lora_write(REG_MODEM_CONFIG_2, (unsigned char) (lora_read(REG_MODEM_CONFIG_2) | 0x04));
}

static void lora_disable_crc(void) {
    lora_write(REG_MODEM_CONFIG_2, (unsigned char) (lora_read(REG_MODEM_CONFIG_2) & 0xfb));
}

static unsigned char lora_random(void) {
    return lora_read(REG_RSSI_WIDEBAND);
}

static void lora_set_explicit_header_mode(void) {
    lora_write(REG_MODEM_CONFIG_1, (unsigned char) (lora_read(REG_MODEM_CONFIG_1) & 0xfe));
    _implicitHeaderMode = false;
}

static void lora_set_implicit_header_mode(void) {
    lora_write(REG_MODEM_CONFIG_1, (unsigned char) (lora_read(REG_MODEM_CONFIG_1) | 0x01));
    _implicitHeaderMode = true;
}


static void lora_idle(void) {
    lora_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    di0_fired = false;
    sending = false;
    receiving = false;
}

static void lora_sleep(void) {
    lora_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    di0_fired = false;
    sending = false;
    receiving = false;
}


static void lora_receive(int size) {
    unsigned char irqFlags;
    if (sending) {
        pr_info("Sendemodus aktiv, warten....");
        while (sending) {
            msleep(25);
        }
    }

    if (size > 0) {
        lora_set_implicit_header_mode();
        lora_write(REG_PAYLOAD_LENGTH, (unsigned char) (size & 0xff));
    } else {
        lora_set_explicit_header_mode();
    }
    // IRQ.Flags resetten
    irqFlags = lora_read(REG_IRQ_FLAGS);
    lora_write(REG_IRQ_FLAGS, (unsigned char) (irqFlags & ~IRQ_RX_DONE_MASK));

    lora_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
    di0_fired = false;
    receiving = true;
}

static int lora_available(void) {
    return (lora_read(REG_RX_NB_BYTES) - _packetIndex);
}


static int lora_read_fifo_byte(void) {
    if (!lora_available()) {
        return -1;
    }
    _packetIndex++;
    return lora_read(REG_FIFO);
}

static void lora_package_received(char *buffer) {
    int irqFlags;
    unsigned int packetLength;
    int c;
    int i;
    unsigned char packet[MAX_PKT_LENGTH + 1];
    int ppos;

    printk("Package!");

    memset(&packet, 0, MAX_PKT_LENGTH);

    irqFlags = lora_read(REG_IRQ_FLAGS);
    lora_write(REG_IRQ_FLAGS, (unsigned char) irqFlags);


    if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        // received a packet
        _packetIndex = 0;
        // read packet length
        packetLength = _implicitHeaderMode ? lora_read(REG_PAYLOAD_LENGTH) : lora_read(REG_RX_NB_BYTES);
        // set FIFO address to current RX address
        lora_write(REG_FIFO_ADDR_PTR, lora_read(REG_FIFO_RX_CURRENT_ADDR));

        printk("Paketgröße: %u", packetLength);
        ppos = 0;
        packet[ppos] = 0x00;
        for (i = 0; i < packetLength; i++) {
            if (ppos < MAX_PKT_LENGTH) {
                c = lora_read_fifo_byte();
                if (c > 0) {
                    packet[ppos] = (unsigned char) c;
                    ppos++;
                } else {
                    i = packetLength;
                }
            } else {
                i = packetLength;
            }
        }
        packet[ppos] = 0x00;

        printk("Payload: %s", (char *) &packet);
        if (buffer != NULL) {
            memcpy(buffer, &packet, (size_t) ppos);
        }
    } else {
        printk("LoRa CRC Error");
        if (buffer != NULL) {
            memcpy(buffer, &packet, (size_t) 1);
        }
    }
    // reset FIFO address
    lora_write(REG_FIFO_ADDR_PTR, 0);
}


static int lora_check_module(void) {
    int result;

    result = lora_get_version();

    if (result == 0) {
        printk("LoRa-Modul antwortert nicht auf die Versionabfrage.\n");
        return -ENODEV;
    }

    printk("LoRa-Modul-Version: %x\n", result);

    return result;
}


static int lora_parse_packet(int size) {
    int packetLength = 0;
    int irqFlags = lora_read(REG_IRQ_FLAGS);

    if (size > 0) {
        lora_set_implicit_header_mode();
        lora_write(REG_PAYLOAD_LENGTH, (unsigned char) (size & 0xff));
    } else {
        lora_set_explicit_header_mode();
    }

    // clear IRQ's
    lora_write(REG_IRQ_FLAGS, (unsigned char) irqFlags);

    if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        pr_info("Paket empfangen");
        // received a packet
        _packetIndex = 0;
        // read packet length
        if (_implicitHeaderMode) {
            packetLength = lora_read(REG_PAYLOAD_LENGTH);
        } else {
            packetLength = lora_read(REG_RX_NB_BYTES);
        }
        // set FIFO address to current RX address
        lora_write(REG_FIFO_ADDR_PTR, lora_read(REG_FIFO_RX_CURRENT_ADDR));
        // put in standby mode
        lora_idle();
    } else if (lora_read(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
        pr_info("Nicht im Empfangsmodus, wechsle Modus...");
        // not currently in RX mode
        // reset FIFO address
        lora_write(REG_FIFO_ADDR_PTR, 0);
        // put in single RX mode
        lora_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }
    di0_fired = false;
    return packetLength;
}

static int16_t lora_get_reg_value(uint8_t reg, uint8_t msb , uint8_t lsb ){
    uint8_t rawValue;
    uint8_t maskedValue;
     if((msb > 7) || (lsb > 7) || (lsb > msb)) {
    return(ERR_INVALID_BIT_RANGE);
  }

  rawValue = lora_read(reg);
   maskedValue = (uint8_t) (rawValue & ((0b11111111 << lsb) & (0b11111111 >> (7 - msb))));
  return(maskedValue);
}


static int8_t lora_get_raw_snr(void) {
    // get SNR value
    int8_t rawSNR = (int8_t) lora_read(REG_PKT_SNR_VALUE);
    return rawSNR;
}

static long lora_get_snr(void) {
    long snr;
    snr=(long)lora_get_raw_snr();
    snr= (snr * 1000);
    snr=snr/4;
    return snr;
}

static int8_t lora_get_rssi(void) {
    int8_t lastPacketRSSI;
    long lastPacketSNR;
    if (LORA_BAND < 868E6) {
        lastPacketRSSI = (int8_t) (-164 + lora_read(REG_PKT_RSSI_VALUE));
    } else {
        lastPacketRSSI = (int8_t) (-157 + lora_read(REG_PKT_RSSI_VALUE));
    }

    lastPacketSNR = lora_get_snr();
    if ( lastPacketSNR < 0) {
        lastPacketRSSI += (lastPacketSNR/1000);
    }

    return lastPacketRSSI;
}

module_init(lora_init);
module_exit(lora_exit);
