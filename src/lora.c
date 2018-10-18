//
// Created by marcel on 11.10.18.
//

#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marcel Schmidt <msc@maennl.info>");
MODULE_DESCRIPTION("LoRa SPI module");

#define RH_SPI_WRITE_MASK 0x80
#define LONG_RANGE_MODE_LORA 0x80
#define LONG_RANGE_MODE_FSK_OOL 0x0
#define ACCESS_SHARED_REG 0x40
#define LOW_FREQUENCE_MODE_ON 0x08
#define SLEEP_MODE 0x00
#define STANDBY_MODE 0x01
#define FSTX_MODE 0x02
#define TX_MODE 0x03
#define FSRX_MODE 0x04
#define RXCONTINUOUS_MODE 0x05
#define RXSINGLE_MODE 0x06
#define CAD_MODE 0x07

static struct spi_device *spi_device;
static int SPI_LORA_BUS_NUM = 0;
module_param(SPI_LORA_BUS_NUM,int, 0000);
MODULE_PARM_DESC(SPI_LORA_BUS_NUM,"SPI Busnummer für LoRa-Modul");


static void lora_read_register(unsigned char ch, unsigned char *rxBuffer, unsigned int rxBuffernLen) {
    if (spi_device != NULL) {
        spi_write_then_read(spi_device, &ch, sizeof(ch), rxBuffer, rxBuffernLen);
    }
}

static void lora_write_register(unsigned char *txBuffer, unsigned int txBuffernLen) {
    if (spi_device != NULL) {
        printk("Writing: %x %x", txBuffer[0], txBuffer[1]);
        spi_write(spi_device, txBuffer, txBuffernLen);
    }
}


static unsigned char lora_get_version(void) {
    unsigned char ch = 0x42;
    unsigned char rx = 0x0;

    lora_read_register(ch, &rx, 1);

    return rx;
}

static void lora_set_opmode(unsigned char opmode) {
    unsigned char ch[2] = {(unsigned char) (0x01 | RH_SPI_WRITE_MASK), opmode};

    lora_write_register((unsigned char *) &ch, 2);
}

static unsigned char lora_get_bytes_received(void) {
    unsigned char rx = 0x0;
    lora_read_register(0x13, &rx, 1);
    return rx;
}


static int __init spi_init(void) {
    int ret;
    unsigned char ch = 0x1F;
    unsigned char rx[2] = {0xFF, 0x00};
    struct spi_master *master;

    //Register information about your slave device:
    struct spi_board_info spi_device_info = {
            .modalias = "lora-spi-driver",
            .max_speed_hz = 4E6, //speed your device (slave) can handle
            .bus_num = SPI_LORA_BUS_NUM,
            .chip_select = 0,
            .mode = SPI_MODE_0,
    };

    printk("LoRa SPI Driver\n");
    printk("SPI Busnummer: %i\n", SPI_LORA_BUS_NUM);


    /*To send data we have to know what spi port/pins should be used. This information
      can be found in the device-tree. */
    master = spi_busnum_to_master(spi_device_info.bus_num);
    if (!master) {
        printk("SPI-MASTER not found.\n");
        return -ENODEV;
    }



    // create a new slave device, given the master and device info
    spi_device = spi_new_device(master, &spi_device_info);

    if (!spi_device) {
        printk("FAILED to create spi slave.\n");
        return -ENODEV;
    }

    spi_device->bits_per_word = 8;

    ret = spi_setup(spi_device);

    if (ret) {
        printk("FAILED to setup spi slave.\n");
        spi_unregister_device(spi_device);
        return -ENODEV;
    }

    printk("LoRa-Modul-Version: %x\n", lora_get_version());
    printk("Modul wird initialisiert\n");

    lora_set_opmode(SLEEP_MODE);
    lora_set_opmode(LONG_RANGE_MODE_LORA | RXCONTINUOUS_MODE);

    printk("Bytes im letzten Paket: %u", lora_get_bytes_received());

    ch = 0x01;
    spi_write_then_read(spi_device, &ch, sizeof(ch), &rx, sizeof(rx));
    printk("LoRa OP Mode: %x\n", rx[0]);
    ch = 0x3c;
    spi_write_then_read(spi_device, &ch, sizeof(ch), &rx, sizeof(rx));
    printk("LoRa Temperature sensor: %x\n", rx[0]);
    ch = 0x00;
    spi_write_then_read(spi_device, &ch, sizeof(ch), &rx, sizeof(rx));
    printk("Received B: %x %x %x %x \n", rx[0], rx[1], ~rx[0], ~rx[1]);

    return 0;
}


static void __exit spi_exit(void) {
    unsigned char ch = 0Xff;

    if (spi_device) {
        spi_write(spi_device, &ch, sizeof(ch));
        spi_unregister_device(spi_device);
    }
}


module_init(spi_init);
module_exit(spi_exit);




