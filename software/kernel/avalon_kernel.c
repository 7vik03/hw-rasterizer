#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "avalon_kernel.h"

#define DRIVER_NAME "rasterizer"

// this is the struct that will get the virtual address of the slave's register region and a shadow of the control register
//?the shadow control reg is the same as it was for the vga_ball lab(i guess we used it for color)
struct rasterizer_dev {
    struct resource         res;        /* physical address region from DT  */
    void __iomem           *virtbase;   /* kernel VA for MMIO access        */
    rasterizer_control_t    control;    /* shadow of CONTROL register       */
} dev;

static void writeTrianglePacket(const triangle_packet_t *pkt)
{

    const __u32 *words = (const __u32 *)pkt;//cast to uint32_pointer to use in the iowrite32 function
    int i;

    //send the 17 words of the triangle packet to the hardware using iowrite32
    for (i = 0; i < RAST_PACKET_NUM_WORDS; i++) {
        iowrite32(words[i],
                  dev.virtbase + RAST_PACKET_WORD_BASE + (i * 4));
    }
    //strobe the commit register  to signal the hardware about the packet being pushed
    iowrite32(1, dev.virtbase + RAST_COMMIT_OFFSET);
}

//read the status register and save the 
static void read_status(rasterizer_status_t *status)
{
    __u32 raw = ioread32(dev.virtbase + RAST_STATUS_OFFSET);

    status->fifo_level = raw & RAST_STATUS_LEVEL_MASK;
    status->fifo_full  = (raw & RAST_STATUS_FULL_BIT) ? 1 : 0;
}

//write to the control register and update the shadow control register in the rasterizer_dev struct
static void write_control(const rasterizer_control_t *ctrl)
{
    __u32 raw = 0;

    raw |= (ctrl->irq_enable   ? RAST_CTRL_IRQ_EN_BIT : 0);
    raw |= ((ctrl->low_watermark & RAST_CTRL_WMARK_MASK) << RAST_CTRL_WMARK_SHIFT);

    iowrite32(raw, dev.virtbase + RAST_CONTROL_OFFSET);

    dev.control = *ctrl;
}

//main ioctl handler for the driver, call this from the userspace program to submit packets, read status, and set/get control
static long rasterizer_ioctl(struct file *f, unsigned int cmd,
                              unsigned long arg)
{
    rasterizer_arg_t ra;
    rasterizer_status_t current_status;

    switch (cmd) {

    case RASTERIZER_SUBMIT:
        if (copy_from_user(&ra, (rasterizer_arg_t __user *)arg,
                           sizeof(rasterizer_arg_t)))
            return -EACCES;

	//check fifo_full first using a separate status variable so ra.packet stays intact
	read_status(&current_status);
	if (current_status.fifo_full) {
	    return -EAGAIN;
	}
	// the extra read can be taken out if we want the code to have a policy to check status before calling submit
        writeTrianglePacket(&ra.packet);
        break;

    case RASTERIZER_STATUS:
        read_status(&ra.status);

        if (copy_to_user((rasterizer_arg_t __user *)arg, &ra,
                         sizeof(rasterizer_arg_t)))
            return -EACCES;
        break;

    case RASTERIZER_SET_CONTROL:
        if (copy_from_user(&ra, (rasterizer_arg_t __user *)arg,
                           sizeof(rasterizer_arg_t)))
            return -EACCES;

        write_control(&ra.control);
        break;

    case RASTERIZER_GET_CONTROL:
        ra.control = dev.control;

        if (copy_to_user((rasterizer_arg_t __user *)arg, &ra,
                         sizeof(rasterizer_arg_t)))
            return -EACCES;
        break;

    default:
        return -EINVAL;
    }

    return 0;
}

static const struct file_operations rasterizer_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = rasterizer_ioctl,
};

static struct miscdevice rasterizer_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = DRIVER_NAME,
    .fops  = &rasterizer_fops,
};

static int __init rasterizer_probe(struct platform_device *pdev)
{
    int ret;

    /* Set sane defaults for the control shadow */
    dev.control.irq_enable    = 0;
    dev.control.low_watermark = 0;

    /* Register as a misc device → creates /dev/rasterizer */
    ret = misc_register(&rasterizer_misc_device);
    if (ret) {
        pr_err(DRIVER_NAME ": misc_register failed (%d)\n", ret);
        return ret;
    }

    /* Get physical address from device tree */
    ret = of_address_to_resource(pdev->dev.of_node, 0, &dev.res);
    if (ret) {
        pr_err(DRIVER_NAME ": of_address_to_resource failed (%d)\n", ret);
        ret = -ENOENT;
        goto out_deregister;
    }

    /* Reserve the physical address region */
    if (request_mem_region(dev.res.start, resource_size(&dev.res),
                           DRIVER_NAME) == NULL) {
        pr_err(DRIVER_NAME ": request_mem_region failed\n");
        ret = -EBUSY;
        goto out_deregister;
    }

    /* Map physical region into kernel virtual address space */
    dev.virtbase = of_iomap(pdev->dev.of_node, 0);
    if (dev.virtbase == NULL) {
        pr_err(DRIVER_NAME ": of_iomap failed\n");
        ret = -ENOMEM;
        goto out_release_mem_region;
    }

    /*
     * Write the control register to a known-good initial state:
     * IRQs disabled, watermark 0.  This also clears any stale state
     * left by a previous insmod/rmmod cycle.
     */
    write_control(&dev.control);

    pr_info(DRIVER_NAME ": probe OK, virtbase=%p phys=0x%08llx size=%llu\n",
            dev.virtbase,
            (unsigned long long)dev.res.start,
            (unsigned long long)resource_size(&dev.res));

    return 0;

out_release_mem_region:
    release_mem_region(dev.res.start, resource_size(&dev.res));
out_deregister:
    misc_deregister(&rasterizer_misc_device);
    return ret;
}

//when rmmod is called
static int rasterizer_remove(struct platform_device *pdev)
{
    rasterizer_control_t off = { .irq_enable = 0, .low_watermark = 0 };
    write_control(&off);

    iounmap(dev.virtbase);
    release_mem_region(dev.res.start, resource_size(&dev.res));
    misc_deregister(&rasterizer_misc_device);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rasterizer_of_match[] = {
    { .compatible = "csee4840,rasterizer-1.0" },
    {},
};
MODULE_DEVICE_TABLE(of, rasterizer_of_match);
#endif

//register the driver
static struct platform_driver rasterizer_driver = {
    .driver = {
        .name           = DRIVER_NAME,
        .owner          = THIS_MODULE,
        .of_match_table = of_match_ptr(rasterizer_of_match),
    },
    .remove = __exit_p(rasterizer_remove),
};


static int __init rasterizer_init(void)
{
    pr_info(DRIVER_NAME ": init\n");
    return platform_driver_probe(&rasterizer_driver, rasterizer_probe);
}

static void __exit rasterizer_exit(void)
{
    platform_driver_unregister(&rasterizer_driver);
    pr_info(DRIVER_NAME ": exit\n");
}

module_init(rasterizer_init);
module_exit(rasterizer_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("CSEE4840 Rasterizer Team, Columbia University");
MODULE_DESCRIPTION("Avalon-MM driver for hardware triangle rasterizer");
