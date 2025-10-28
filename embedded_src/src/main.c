#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

void main(void)
{
    printk("The Amazing Balancing Ball!\n");
    while (1) {
        k_msleep(1000);
        printk("Tick\n");
    }
}
