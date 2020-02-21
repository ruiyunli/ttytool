/*
* Dummy serial driver by sdliu <sdliu@hongdian.com>
* The real hardware was installed in the MCU which is attached to our SOC(HI3520D).
* This driver depends on a serial manager which will process in the userspace
*/

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/tty_flip.h>
#include <linux/circ_buf.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <linux/platform_device.h>

// #include <asm/sizes.h>

#define prt_dbg
#ifdef prt_dbg
#define drintk printk
#else
#define drintk(...) 
#endif

#define DUMMY_SERIAL_NR         4
#define DUMMY_DEBUG_ON

static int dummy_serial_major = 0;
static int dummy_serial_minor_start = 0;
static struct dummy_file_data* dummy_array[2*DUMMY_SERIAL_NR];

unsigned int dummy_serial_nr = 2;
module_param(dummy_serial_nr, uint, S_IRUGO);

struct dummy_platform_data {
    // unsigned char port_idx;

    unsigned long fifo_size;
};

struct dummy_cache {
  struct circ_buf fifo;
  spinlock_t lock;
};

struct dummy_file_data {
    struct dummy_platform_data* platform_data;

    struct dummy_cache* cache_local;
    struct dummy_cache* cache_buddy;
  
    struct cdev c_dev;
};


static struct class* dummy_class;

#define dummy_circ_empty(circ)		((circ)->head == (circ)->tail)
    

int dummy_open(struct inode* i, struct file* file)
{
    struct dummy_file_data* data =  container_of(i->i_cdev, struct dummy_file_data, c_dev);
    if(!data)
      printk("invalid private data\n");
    
    file->private_data = data;

    drintk("minor %d private_data %p\n", iminor(i), file->private_data);

    return 0;
}

int dummy_release(struct inode* i, struct file* file)
{
    struct dummy_file_data* dummy = (struct dummy_file_data*)file->private_data;
    file->private_data = NULL;
    return 0;
}

ssize_t dummy_read(struct file* file, char __user* buf, size_t size, loff_t* offset)
{
    struct dummy_file_data* dummy = (struct dummy_file_data*)file->private_data;
    struct circ_buf* circ = &dummy->cache_local->fifo;
    spinlock_t* lock = &dummy->cache_local->lock;
    int fifo_size=0, fifo_len=0, read_len=0, ret=0, len=0;
    
    spin_lock(lock);

    fifo_size = dummy->platform_data->fifo_size;
    fifo_len = CIRC_CNT(circ->head, circ->tail, fifo_size);
    read_len = size > fifo_len ? fifo_len : size;

    while (read_len >= 0) {
		len = CIRC_CNT_TO_END(circ->head, circ->tail, fifo_size);
        if(len > read_len)
          len = read_len;
        if(len <= 0)
          break;

#ifdef DUMMY_DEBUG_ON
        printk("dummy_read size:%d fifo_len:%d len:%d\n", size, fifo_len, len);
#endif
		copy_to_user(buf, circ->buf + circ->tail, len);
        circ->tail = (circ->tail + len) & (fifo_size - 1);

        buf += len;
        read_len -= len;
        ret += len;
	}

    spin_unlock(lock);
    return ret;
}

ssize_t dummy_write(struct file* file, const char __user* buf, size_t size, loff_t* offset)
{
    struct dummy_file_data* dummy = (struct dummy_file_data*)file->private_data;
    struct circ_buf* circ = &dummy->cache_local->fifo;
    spinlock_t* lock = &dummy->cache_local->lock;
    int fifo_size=0, fifo_space=0, write_len=0, ret=0, len=0;
    
    spin_lock(lock);
    
    fifo_size = dummy->platform_data->fifo_size;
    fifo_space = CIRC_SPACE(circ->head, circ->tail, fifo_size);
    write_len = size > fifo_space ? fifo_space : size;

#ifdef DUMMY_DEBUG_ON
    printk("dummy write size:%d\n", size);
#endif
    
    while ( write_len > 0) {
		len = CIRC_SPACE_TO_END(circ->head, circ->tail, fifo_size);
		if (write_len < len)
			len = write_len;
		if (len <= 0)
			break;

#ifdef DUMMY_DEBUG_ON        
        printk("dummy_write size:%d fifo_space:%d len:%d\n", size, fifo_space, len);
#endif
        
		copy_from_user(circ->buf + circ->head, buf, len);
		circ->head = (circ->head + len) & (fifo_size - 1);
        
		buf += len;
	    write_len -= len;
		ret += len;
	}

    spin_unlock(lock);
    return ret;
}

long dummy_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    struct dummy_file_data* dummy = (struct dummy_file_data*)file->private_data;

    return 0;
}

unsigned int dummy_poll(struct file* file, struct poll_table_struct* pwait)
{
    struct dummy_file_data* dummy = (struct dummy_file_data*)file->private_data;
    struct circ_buf* circ = &dummy->cache_local->fifo;
    spinlock_t * lock = &dummy->cache_local->lock;
    unsigned int mask = 0, fifo_size = 0, cnt = 0;
    
    spin_lock(lock);
    
    fifo_size = dummy->platform_data->fifo_size;
    cnt = CIRC_CNT(circ->head, circ->tail, fifo_size);

    spin_unlock(lock);
    
    //printk("%d cnt %d\n", dummy->index, cnt);
    
    if (cnt)
        mask |= POLLIN | POLLRDNORM;

    return mask;
}

int dummy_fasync(int fd, struct file* filp, int mode)
{
    //struct dummy_file_data* dummy = (struct dummy_file_data*)filp->private_data;
    int ret = 0;
    return ret;
}

struct file_operations dummy_fops = {
    .open = dummy_open,
    .release = dummy_release,
    .read = dummy_read,
    .write = dummy_write,
    .unlocked_ioctl = dummy_ioctl,
    .poll = dummy_poll,
    .fasync = dummy_fasync,
};


static struct dummy_file_data* create_one_device(const char* dev_name, int major, int minor, int fifo_size)
{
    struct dummy_file_data* dummy = NULL;
    struct device* tmp = NULL;
    struct dummy_cache* cache = NULL;
    dev_t dev = 0;

#ifdef DUMMY_DEBUG_ON
    printk("start create one device:%s major:%d minor:%d size:%d\n", dev_name, major, minor, fifo_size);
#endif

    dummy = (struct dummy_file_data*)kmalloc(sizeof(struct dummy_file_data), GFP_KERNEL);
    if (!dummy) {
        printk("malloc dummy error\n");
        return NULL;
    }

    memset(dummy, 0, sizeof(struct dummy_file_data));

    cache =(struct dummy_cache*)kmalloc(sizeof(struct dummy_cache), GFP_KERNEL);
    if ( !cache ) {
        printk("dummy cache kmalloc error\n");
        goto CACHE_ERR;
    }
    cache->fifo.buf = (unsigned char*)kmalloc(fifo_size, GFP_KERNEL);
    if ( !cache->fifo.buf ) {
        printk("dummy fifo buffer kmalloc error\n");
        goto CACHE_ERR;
    }

    cache->fifo.head = 0;
    cache->fifo.tail = 0;
    spin_lock_init(&cache->lock);
    
    dummy->cache_local = cache;
    
    dev = MKDEV(major, minor);
    
    cdev_init(&dummy->c_dev, &dummy_fops);
    dummy->c_dev.owner = THIS_MODULE;
    cdev_add(&dummy->c_dev, dev, DUMMY_SERIAL_NR);

    tmp = device_create(dummy_class, NULL, dev, NULL, dev_name);
    if (NULL == tmp) {
        printk("create device err! %d, %s\n", dev, dev_name);
    }
    
    printk("create dummy serial manager device /dev/%s\n", dev_name);
    return dummy;

CACHE_ERR:
    if (cache && cache->fifo.buf)
        kfree(cache->fifo.buf);

    if(cache)
        kfree(cache);
    
    kfree(dummy);
    return NULL;
}

int create_manager_device(struct platform_device* pdev, int major, int minor_start, int index)
{
    struct dummy_file_data* dummyx = NULL, *dummyy = NULL;
    struct dummy_platform_data* data = NULL;
    char dev_name[64] = { 0 };

    data = (struct dummy_platform_data*)pdev->dev.platform_data;
    if (!data) {
        printk("not platform data\n");
        return -EINVAL;
    }

#ifdef DUMMY_DEBUG_ON
    printk("start create x and y\n");
#endif

    // create x device
    sprintf(dev_name, "serialx%d", index);
    dummyx = create_one_device(dev_name, major, minor_start + 2*index, data->fifo_size);
    if(!dummyx){
      printk("create_one_device failed dev:%s major:%d index:%d\n", dev_name, major, index);
      return -1;
    }

    // create y device
    sprintf(dev_name, "serialy%d", index);
    dummyy = create_one_device(dev_name, major, minor_start + 2*index+1, data->fifo_size);
    if(!dummyy){
      printk("create_one_device failed dev:%s major:%d index:%d\n", dev_name, major, index);
      return -1;
    }

#ifdef DUMMY_DEBUG_ON
    printk("finish create x:%p and y:%p\n", dummyx, dummyy);
#endif

    dummyx->cache_buddy = dummyy->cache_local;
    dummyy->cache_buddy = dummyx->cache_local;

    dummyx->platform_data = data;
    dummyy->platform_data = data;
    
    dummy_array[2*index] = dummyx;
    dummy_array[2*index+1] = dummyy;
    return 0;
}


static int serial_dummy_probe(struct platform_device* pdev)
{
    int i = 0;
    int ret = 0;
    char dev_name[64] = { 0 };
    dev_t dev = 0;
    
    sprintf(dev_name, "serialdum%d", 0);
    ret = alloc_chrdev_region(&dev, 0, DUMMY_SERIAL_NR, dev_name);
    if (!ret) {
      dummy_serial_major = MAJOR(dev);
      dummy_serial_minor_start = MINOR(dev);
      // printk("major %d minor_start %d\n", dummy_serial_major, dummy_serial_minor_start);
    }
    else {
      printk("register cdev err, ret=%d\n", ret);
      return ret;
    }

    for (i = 0; i < dummy_serial_nr; i++) {
        if(2*i >= DUMMY_SERIAL_NR)
        {
            printk("dummy device overflow, cur:%d max:%d\n", 2*i, DUMMY_SERIAL_NR);
            break;
        }
        
        ret = create_manager_device(pdev, dummy_serial_major, dummy_serial_minor_start, i);
        if (ret) {
            printk("create dummy manager device err, index = %d\n", i);
        }
    }
    return 0;
}

static int serial_dummy_remove(struct platform_device* dev)
{
    struct dummy_file_data* dummy = NULL;
    int i = 0;
    dev_t dev_num = 0;

    for (i = 0; i < dummy_serial_nr; i++) {
        dummy = dummy_array[i];

        dev_num = MKDEV(dummy_serial_major, dummy_serial_minor_start + i);
        device_destroy(dummy_class, dev_num);
        cdev_del(&dummy->c_dev);

        if (dummy->cache_local && dummy->cache_local->fifo.buf)
            kfree(dummy->cache_local->fifo.buf);
        if (dummy->cache_local)
            kfree(dummy->cache_local);

        kfree(dummy);
    }

    dev_num = MKDEV(dummy_serial_major, dummy_serial_minor_start);
    unregister_chrdev_region(dev_num, DUMMY_SERIAL_NR);

    return 0;
}

static struct platform_driver dummy_serial_dirver = {
    .probe = serial_dummy_probe,
    .remove = serial_dummy_remove,
    .driver = {
        .name = "dummy_serial",
        .owner = THIS_MODULE,
    }
};

static struct dummy_platform_data dummy_serial_dev_data = {
        .fifo_size = 2 * 1024,
};

void dummy_serial_release(struct device* dev);
static struct platform_device dummy_serial_dev = {
    .name = "dummy_serial",
    .dev = {
        .release = dummy_serial_release,
        .platform_data = (void*)(&dummy_serial_dev_data),
    }
};

void dummy_serial_release(struct device* dev)
{


}

static int __init serial_dummy_init(void)
{
    int ret = 0;

    printk("Hi3520d dummy serial drv, by sdliu ver.20151202\n");

    if (2*dummy_serial_nr > DUMMY_SERIAL_NR) {
        printk("dummy serial nr(%d) is more than max(%d)\n", \
            2*dummy_serial_nr, DUMMY_SERIAL_NR);
        return -EINVAL;
    }

    // 注册平台设备(应当分离出去)
    ret = platform_device_register(&dummy_serial_dev);
    dummy_class = class_create(THIS_MODULE, "dumtty");
    ret = platform_driver_register(&dummy_serial_dirver);
    if (ret) {
        printk("dummy platform drv register err ret = %d\n", ret);
        platform_device_unregister(&dummy_serial_dev);
    }

    return ret;
}

static void __exit serial_dummy_exit(void)
{
    platform_driver_unregister(&dummy_serial_dirver);
    class_destroy(dummy_class);
    platform_device_unregister(&dummy_serial_dev);
}

module_init(serial_dummy_init);
module_exit(serial_dummy_exit);

MODULE_AUTHOR("sdliu");
MODULE_DESCRIPTION("DUMMY serial driver");
MODULE_LICENSE("GPL");


