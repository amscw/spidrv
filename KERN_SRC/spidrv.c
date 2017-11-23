/*
===============================================================================
Driver Name		:		spidrv
Author			:		MOSKVIN (AO "NTC ELINS")
License			:		GPL
Description		:		LINUX DEVICE DRIVER PROJECT
===============================================================================
*/

#include "spidrv.h"
#include "spidrv_ioctl.h"

//-------------------------------------------------------------------------------------------------
// MACRO
//-------------------------------------------------------------------------------------------------
#define SPIDRV_N_MINORS 1
#define SPIDRV_FIRST_MINOR 0
#define SPIDRV_NODE_NAME "axi_SpiMaster"
#define SPIDRV_BUFF_SIZE 1024
#define SPIDRV_PLATFORM_NAME DRIVER_NAME

MODULE_LICENSE("GPL");
MODULE_AUTHOR("MOSKVIN");


//-------------------------------------------------------------------------------------------------
// Type declarations
//-------------------------------------------------------------------------------------------------
// private driver's data
typedef struct privatedata {
	int nMinor;
	void __iomem *io_base;
	spinlock_t lock;
	char buff[SPIDRV_BUFF_SIZE];
	struct kobject *registers;
	struct resource resource;
	struct cdev cdev;
	struct device *spidrv_device;
	int flag_device_opened;
	int flag_wait_int;
	wait_queue_head_t wq;
} spidrv_private;

struct platform_private {
	struct platform_device *pdev;
	int irq;
	spidrv_private charpriv;
};

// attribute types
// файлы полей регистров
struct reg_attribute {
	struct attribute default_attribute;
	unsigned int value;
	unsigned char bitOffset;
	unsigned char bitSize;
	spidrv_private *priv;
} __attribute__((__packed__));

// файлы, расположенные в каталоге registers
struct registers_attribute {
	struct attribute default_attribute;
} __attribute__((__packed__));

//-------------------------------------------------------------------------------------------------
// MACRO
//-------------------------------------------------------------------------------------------------
#define BITSIZE_TO_MASK(mask, size) {\
	unsigned int tmp = size;\
	mask = 0;\
	while (tmp) \
		mask |= (1 << --tmp);\
}

// Register map definitions
// NAME: kobject's name
// ADDRESS: attribute's address field
#define REG_CONTROL_NAME	"control"
#define REG_CONTROL_ADDR	0x0000
#define REG_WRDATA_NAME		"wrdata"
#define REG_WRDATA_ADDR		0x0004
#define REG_RDDATA_NAME		"rddata"
#define REG_RDDATA_ADDR		0x0008
#define REG_MADDR_NAME		"maddr"
#define REG_MADDR_ADDR		0x000C
#define REG_CSMASK_NAME		"csmask"
#define REG_CSMASK_ADDR		0x0010
#define REG_SETTINGS_NAME	"settings"
#define REG_SETTINGS_ADDR	0x0014

// метапрограммное колдовство: макросы для быстрого создания типов объектов kobject и их атрибутов
#define MAKE_REG_FIELD(reg, field, file, offset, size) \
	static struct reg_attribute reg##_##field = {\
		{\
			.name = file,\
			.mode = S_IRUGO | S_IWUSR,\
		}, 0, offset, size, NULL\
	}

#define MAKE_SYSFS_OPS(reg) \
	static ssize_t sysfs_##reg##_show(struct kobject *kobj, struct attribute *attr, char *buf)\
	{\
		unsigned long flags = 0;\
		struct reg_attribute *regattr = container_of(attr, struct reg_attribute, default_attribute);\
		unsigned int mask;\
		\
		BITSIZE_TO_MASK(mask, regattr->bitSize);\
		spin_lock_irqsave(&regattr->priv->lock, flags);\
		regattr->value = ioread32((void __iomem*)(regattr->priv->io_base + REG_##reg##_ADDR));\
		spin_unlock_irqrestore(&regattr->priv->lock, flags);\
		regattr->value >>= regattr->bitOffset;\
		regattr->value &= mask;\
		PDEBUG("read from %s@0x%X[%d:%d] = 0x%X\n", regattr->default_attribute.name, REG_##reg##_ADDR, regattr->bitOffset + regattr->bitSize - 1, regattr->bitOffset, regattr->value);\
		return scnprintf(buf, PAGE_SIZE, "%d\n", regattr->value);\
	}\
	\
	static ssize_t sysfs_##reg##_store(struct kobject *kobj, struct attribute* attr, const char *buf, size_t len)\
	{\
		unsigned long flags = 0;\
		struct reg_attribute *regattr = container_of(attr, struct reg_attribute, default_attribute);\
		unsigned int mask, tmp;\
		\
		BITSIZE_TO_MASK(mask, regattr->bitSize);\
		sscanf(buf, "%d", &tmp);\
		spin_lock_irqsave(&regattr->priv->lock, flags);\
		regattr->value = ioread32((void __iomem*)(regattr->priv->io_base + REG_##reg##_ADDR));\
		regattr->value &= ~(mask << regattr->bitOffset);\
		regattr->value |= ((tmp & mask) << regattr->bitOffset);\
		iowrite32(regattr->value, (void __iomem*)(regattr->priv->io_base + REG_##reg##_ADDR));\
		spin_unlock_irqrestore(&regattr->priv->lock, flags);\
		PDEBUG("write 0x%X to %s@0x%X[%d:%d]\n", (tmp & mask), regattr->default_attribute.name, REG_##reg##_ADDR, regattr->bitOffset + regattr->bitSize - 1, regattr->bitOffset);\
		return len;\
	}\
	\
	static struct sysfs_ops sysfs_##reg##_ops = {\
		.show = sysfs_##reg##_show,\
		.store = sysfs_##reg##_store,\
	};\

#define MAKE_KOBJ_TYPE(reg) \
	MAKE_SYSFS_OPS(reg);\
	static struct kobj_type kobj_##reg##_type = {\
		.sysfs_ops = &sysfs_##reg##_ops,\
		.default_attrs = reg##_attributes,\
	};

// write/read operations
#define GET_MAPPED_ADDR(reg) ((void __iomem *)(reg##_value.priv->io_base + REG_##reg##_ADDR))
#define WR_REG(reg, value) ( iowrite32(value, GET_MAPPED_ADDR(reg)) )
#define RD_REG(reg) ( ioread32(GET_MAPPED_ADDR(reg)) )

//-------------------------------------------------------------------------------------------------
// Prototypes
//-------------------------------------------------------------------------------------------------
static int spidrv_probe(struct platform_device *pdev);
static int spidrv_remove(struct platform_device *pdev);
static int spidrv_open(struct inode *inode,struct file *filp);
static int spidrv_release(struct inode *inode,struct file *filp);
static ssize_t spidrv_read(struct file *filp, char __user *ubuff,size_t count,loff_t *offp);
static ssize_t spidrv_write(struct file *filp,	const char __user *ubuff, size_t count, loff_t *offp);
static ssize_t sysfs_show(struct kobject *kobj, struct attribute *attr, char *buf);
static ssize_t sysfs_store(struct kobject *kobj, struct attribute* attr, const char *buf, size_t len);
static long spidrv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

//-------------------------------------------------------------------------------------------------
// Variables
//-------------------------------------------------------------------------------------------------
int spidrv_major=0;
dev_t spidrv_device_num;
struct class *spidrv_class;
atomic_t dev_cnt = ATOMIC_INIT(SPIDRV_FIRST_MINOR - 1);

static struct of_device_id spidrv_of_match[] = {
	{ .compatible = "xlnx,axi-SpiMaster-1.0", },
	{}
};
MODULE_DEVICE_TABLE(of, spidrv_of_match);

struct platform_driver spidrv_driver = {
		.driver = {
			.name	= DRIVER_NAME,
			.owner	= THIS_MODULE,
			.of_match_table = of_match_ptr(spidrv_of_match),
		},
		.probe 		= spidrv_probe,
		.remove		= spidrv_remove,
};

static struct file_operations spidrv_fops= {
	.owner				= THIS_MODULE,
	.open				= spidrv_open,
	.release			= spidrv_release,
	.read				= spidrv_read,
	.write				= spidrv_write,
	.unlocked_ioctl		= spidrv_ioctl,
};

// hardcoded sysfs device model
// каталог registers
static struct sysfs_ops sysfs_registers_ops = {
	.show = sysfs_show,
	.store = sysfs_store,
};

static struct kobj_type kobj_registers_type = {
	.sysfs_ops = &sysfs_registers_ops,
	.default_attrs = NULL,
};

// поля регистров (атрибуты) задаем руками, тип генерируется автоматически
// формат: имя регистра, имя атрибута, имя файла, смещение в битах, размер в битах

// CONTROL register
MAKE_REG_FIELD(CONTROL, cr, "CR", 0, 1);
MAKE_REG_FIELD(CONTROL, sr_spi_rdy, "SR_SPI_RDY", 1, 1);
MAKE_REG_FIELD(CONTROL, sr_spi_dre, "SR_SPI_DRE", 2, 1);
MAKE_REG_FIELD(CONTROL, sr_spi_trc, "SR_SPI_TRC", 3, 1);
MAKE_REG_FIELD(CONTROL, ir_spi_rdy, "IR_SPI_RDY", 4, 1);
MAKE_REG_FIELD(CONTROL, ir_spi_dre, "IR_SPI_DRE", 5, 1);
MAKE_REG_FIELD(CONTROL, ir_spi_trc, "IR_SPI_TRC", 6, 1);
MAKE_REG_FIELD(CONTROL, rst, "RST", 7, 1);
MAKE_REG_FIELD(CONTROL, value, "value",	0, 32);
static struct attribute *CONTROL_attributes[] = {
	&CONTROL_cr.default_attribute,
	&CONTROL_sr_spi_rdy.default_attribute,
	&CONTROL_sr_spi_dre.default_attribute,
	&CONTROL_sr_spi_trc.default_attribute,
	&CONTROL_ir_spi_rdy.default_attribute,
	&CONTROL_ir_spi_dre.default_attribute,
	&CONTROL_ir_spi_trc.default_attribute,
	&CONTROL_rst.default_attribute,
	&CONTROL_value.default_attribute,
	NULL
};
MAKE_KOBJ_TYPE(CONTROL);

// WRDATA register
MAKE_REG_FIELD(WRDATA, value, "value", 0, 32);
static struct attribute *WRDATA_attributes[] = {
	&WRDATA_value.default_attribute,
	NULL
};
MAKE_KOBJ_TYPE(WRDATA);

// RDDATA register
MAKE_REG_FIELD(RDDATA, value, "value", 0, 32);
static struct attribute *RDDATA_attributes[] = {
	&RDDATA_value.default_attribute,
	NULL
};
MAKE_KOBJ_TYPE(RDDATA);

// MADDR register
MAKE_REG_FIELD(MADDR, wr_addr, "WR_ADDR", 0, 5);
MAKE_REG_FIELD(MADDR, rd_addr, "RD_ADDR", 5, 5);
MAKE_REG_FIELD(MADDR, value, "value", 0, 32);
static struct attribute *MADDR_attributes[] = {
	&MADDR_wr_addr.default_attribute,
	&MADDR_rd_addr.default_attribute,
	&MADDR_value.default_attribute,
	NULL
};
MAKE_KOBJ_TYPE(MADDR);

// CSMASK register
MAKE_REG_FIELD(CSMASK, value, "value", 0, 32);
static struct attribute *CSMASK_attributes[] = {
	&CSMASK_value.default_attribute,
	NULL
};
MAKE_KOBJ_TYPE(CSMASK);

// SETTINGS register
MAKE_REG_FIELD(SETTINGS, dw, "DW", 0, 5);
MAKE_REG_FIELD(SETTINGS, start_wait, "START_WAIT", 5, 4);
MAKE_REG_FIELD(SETTINGS, end_wait, "END_WAIT", 9, 4);
MAKE_REG_FIELD(SETTINGS, clk_div, "CLK_DIV", 13, 8);
MAKE_REG_FIELD(SETTINGS, cs_min_width, "CS_MIN_WIDTH", 21, 4);
MAKE_REG_FIELD(SETTINGS, cpol, "CPOL", 25, 1);
MAKE_REG_FIELD(SETTINGS, cpha, "CPHA", 26, 1);
MAKE_REG_FIELD(SETTINGS, msb_first, "MSB_FIRST", 27, 1);
MAKE_REG_FIELD(SETTINGS, en_cs_manual, "EN_CS_MANUAL", 28, 1);
MAKE_REG_FIELD(SETTINGS, value, "value", 0, 32);
static struct attribute *SETTINGS_attributes[] = {
	&SETTINGS_dw.default_attribute,
	&SETTINGS_start_wait.default_attribute,
	&SETTINGS_end_wait.default_attribute,
	&SETTINGS_clk_div.default_attribute,
	&SETTINGS_cs_min_width.default_attribute,
	&SETTINGS_cpol.default_attribute,
	&SETTINGS_cpha.default_attribute,
	&SETTINGS_msb_first.default_attribute,
	&SETTINGS_en_cs_manual.default_attribute,
	&SETTINGS_value.default_attribute,
	NULL
};
MAKE_KOBJ_TYPE(SETTINGS);

// связываем все вместе:
// -имя регистра
// -указатель на ktype
// -указатель на kobject
static struct reg_descr {
	const char* const name;
	struct kobj_type *type;
	struct kobject *kobj;
} regs[] = {
	{ REG_CONTROL_NAME, &kobj_CONTROL_type, NULL },
	{ REG_WRDATA_NAME, &kobj_WRDATA_type, NULL },
	{ REG_RDDATA_NAME, &kobj_RDDATA_type, NULL },
	{ REG_MADDR_NAME, &kobj_MADDR_type, NULL },
	{ REG_CSMASK_NAME, &kobj_CSMASK_type, NULL },
	{ REG_SETTINGS_NAME, &kobj_SETTINGS_type, NULL },
	{}
};

//-------------------------------------------------------------------------------------------------
// Functions
//-------------------------------------------------------------------------------------------------
static inline void cleanup_regs(void)
{
	struct reg_descr *reg = regs;
	while (reg->name != NULL)
	{
		if (reg->kobj)
		{
			kobject_put(reg->kobj);
			reg->kobj = NULL;
		}
		reg++;
	}
}

static int create_regs(struct kobject *parent, spidrv_private *priv)
{
	int res, i;
	struct reg_descr *reg = regs;
	struct reg_attribute *attr;

	while (reg->name != NULL)
	{
		// установить priv для всех атрибутов
		for (attr = (struct reg_attribute*)reg->type->default_attrs[0], i = 0; attr;attr = (struct reg_attribute*)reg->type->default_attrs[++i])
		 	attr->priv = priv;

		// выделить память под kobject
		reg->kobj = kzalloc(sizeof *reg->kobj, GFP_KERNEL);
		if (!reg->kobj)
		{
			cleanup_regs();
			return -ENOMEM;
		}
		// зарегистрировать kobject
		kobject_init(reg->kobj, reg->type);
		res = kobject_add(reg->kobj, parent, "%s", reg->name);
		if (res != 0)
		{
			cleanup_regs();
			return res;
		}
		reg++;
	}
	return 0;
}

static ssize_t sysfs_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	PDEBUG("sysfs_show not implemented\n");
	return 0;
}

static ssize_t sysfs_store(struct kobject *kobj, struct attribute* attr, const char *buf, size_t len)
{
	PDEBUG("sysfs_store not implemented\n");
	return 0;
}

static irqreturn_t spidrv_irq_handler(int irq, void *dev_id /*, struct pt_regs *regs */ )
{
	struct platform_private *priv = (struct platform_private*)dev_id;
	SPI_CONTROL_TypeDef spi_control;

	spi_control.reg.value = RD_REG(CONTROL);

	// сбросить флаг прерывания
	spi_control.reg.bits.SR_SPI_RDY = 0;
	WR_REG(CONTROL, spi_control.reg.value);
	// PDEBUG("In char driver spidrv_irq_handler() function \n");
	priv->charpriv.flag_wait_int = 1;
	wake_up_interruptible(&priv->charpriv.wq);
	return IRQ_HANDLED;
}

static int spidrv_open(struct inode *inode,struct file *filp)
{
	spidrv_private *charpriv = container_of(inode->i_cdev, spidrv_private, cdev);
	struct platform_private *priv = container_of(charpriv, struct platform_private, charpriv);

	// PINFO("In char driver open() function\n");

	if (charpriv->flag_device_opened)
	{
		PERR("device already opened!\n");
		return -EBUSY;
	}

	if (request_irq(priv->irq, spidrv_irq_handler, IRQF_SHARED, DRIVER_NAME, priv))
	{
		PERR("Failed to request for irq\n");
		return -ENOMEM;
	}

	filp->private_data = charpriv;
	charpriv->flag_device_opened = 1;
	return 0;
}					

static int spidrv_release(struct inode *inode,struct file *filp)
{
	spidrv_private *charpriv = filp->private_data;
	struct platform_private *priv = container_of(charpriv, struct platform_private, charpriv);

	// PINFO("In char driver release() function\n");

	free_irq(priv->irq, priv);
	charpriv->flag_device_opened = 0;
	return 0;
}

static ssize_t spidrv_read(struct file *filp, char __user *ubuff,size_t count,loff_t *offp)
{
	int n=0, res;
	spidrv_private *charpriv = filp->private_data;
	SPI_CONTROL_TypeDef spi_control;
	SPI_SETTINGS_TypeDef spi_settings;
	u32 rd_data;

	// PINFO("In char driver read() function\n");

	spi_control.reg.value = RD_REG(CONTROL);
	spi_settings.reg.value = RD_REG(SETTINGS);
	// старт транзакции чтения
	spi_control.reg.bits.CR = 1;
	WR_REG(CONTROL, spi_control.reg.value);
	// ждем готовности принятых данных
	res = wait_event_interruptible_timeout(charpriv->wq, charpriv->flag_wait_int != 0, msecs_to_jiffies(1000));
	if (res == 0)
	{
		// @condition evaluated to %false after the @timeout elapsed
		// PDEBUG("read operation timeout! \n");
		return 0;
	}
	rd_data = RD_REG(RDDATA);
	n = spi_settings.reg.bits.DW / 8 + ( (spi_settings.reg.bits.DW % 8) ? 1 : 0 );
	copy_to_user(ubuff, &rd_data, n);
	charpriv->flag_wait_int = 0;
	// PDEBUG("received: %d bytes\n", n);
	return n;
}

static ssize_t spidrv_write(struct file *filp,	const char __user *ubuff, size_t count, loff_t *offp)
{
	/* TODO Auto-generated Function */
	int n=0;

	spidrv_private *priv;
	priv = filp->private_data;
	// PINFO("In char driver write() function\n");
	return n;
}

static long spidrv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int res;
	SPI_CONTROL_TypeDef spi_control;

	// validate type
	if (_IOC_TYPE(cmd) != SPIDRV_IOC_MAGIC)
		return -ENOTTY;
	// validate number
	if (_IOC_NR(cmd) > SPIDRV_IOC_MAXNR)
		return -ENOTTY;
	// validate access
	if (_IOC_DIR(cmd) & _IOC_READ)
		res = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		res = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (res)
		return -EFAULT;

	// can process
	switch (cmd)
	{
	case SPIDRV_IOCRESET:
		PDEBUG("ioctl: Performing reset\n");
		spi_control.reg.value = RD_REG(CONTROL);
		spi_control.reg.bits.RST = 1;
		WR_REG(CONTROL, spi_control.reg.value);
		break;

	default:
		return -ENOTTY;
	}
	return 0;
}

static int spidrv_probe(struct platform_device *pdev)
{
	int res;
	dev_t curr_dev;
	unsigned int minor = atomic_inc_return(&dev_cnt);
	struct platform_private *priv;
	spidrv_private *charpriv;
	SPI_CONTROL_TypeDef spi_control;
	SPI_SETTINGS_TypeDef spi_settings;

	PINFO("In probe() function\n");

	if (minor == SPIDRV_N_MINORS + SPIDRV_FIRST_MINOR)
		return -EAGAIN;

	curr_dev = MKDEV(MAJOR(spidrv_device_num), MINOR(spidrv_device_num) + minor);
	PDEBUG("current device number: major = %d, minor = %d\n", MAJOR(curr_dev), MINOR(curr_dev));

	// 1. выделение памяти под структуру устройства
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		PERR("Failed to allocate memory for the private data structure\n");
		return -ENOMEM;
	}
	charpriv = &priv->charpriv;


	// 2. создание файловой системы sysfs
	charpriv->registers = kzalloc(sizeof *charpriv->registers, GFP_KERNEL);
	if (!charpriv->registers)
	{
		PERR("Failed to allocate kobject\n");
		goto fail_alloc_kobj;
	}
	kobject_init(charpriv->registers, &kobj_registers_type);
	res = kobject_add(charpriv->registers, NULL, "%s-%s", DRIVER_NAME, "registers");
	if (res != 0)
	{
		PERR("Failed to register kobject\n");
		goto fail_register_kobj;
	}
	res = create_regs(charpriv->registers, charpriv);
	if (res != 0)
	{
		PERR("Failed to create registers (%d)\n", res);
		goto fail_register_kobj;
	}

	// 3. маппинг памяти устройства
	res = of_address_to_resource(pdev->dev.of_node, 0, &charpriv->resource);
	if (res != 0)
	{
		PERR("Failed to retrive memory space resource\n");
		goto fail_retrive_mem;
	} else {
		PDEBUG("resource memory start: %x\n", charpriv->resource.start);
		PDEBUG("resource memory end:   %x\n", charpriv->resource.end);
	}
	// запрос адресного проатранства у ядра, чтобы не возникло коллизии с другими модулями
	if (request_mem_region(charpriv->resource.start, resource_size(&charpriv->resource), DRIVER_NAME) == NULL)
	{
		PERR("Failed to mapping memory region\n");
		goto fail_retrive_mem;
	}
	charpriv->io_base = of_iomap(pdev->dev.of_node, 0);
	if (!charpriv->io_base)
	{
		PERR("Failed to mapping region\n");
		goto fail_map_mem;
	} else {
		PDEBUG("device mapped to %p\n", charpriv->io_base);
	}

	// 4. настройка прерываний
	priv->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	PDEBUG("IRQ number = %d\n", priv->irq);
	charpriv->flag_wait_int = 0;
	init_waitqueue_head(&charpriv->wq);
	spin_lock_init(&charpriv->lock);

	// 5. настройка устройства
	spi_control.reg.value = 0;
	spi_control.reg.bits.IR_SPI_RDY = 1;
	spi_control.reg.bits.RST = 1;

	spi_settings.reg.value = 0;
	spi_settings.reg.bits.DW = 16;
	spi_settings.reg.bits.CLK_DIV = 10;
	spi_settings.reg.bits.EN_CS_MANUAL = 1;
	spi_settings.reg.bits.MSB_FIRST = 1;
	spi_settings.reg.bits.CPHA = 1;

	WR_REG(CONTROL, spi_control.reg.value);
	WR_REG(SETTINGS, spi_settings.reg.value);
	WR_REG(CSMASK, 0);

	// регистрация устройства
	device_create(spidrv_class, NULL, curr_dev, priv, SPIDRV_NODE_NAME"%d", minor);
	cdev_init(&priv->charpriv.cdev, &spidrv_fops);
	cdev_add(&priv->charpriv.cdev, curr_dev, 1);

	platform_set_drvdata(pdev, priv);
	PINFO("device probed successfully!\n");
	return 0;


fail_map_mem:
	release_mem_region(charpriv->resource.start, resource_size(&charpriv->resource));

fail_retrive_mem:
	cleanup_regs();

fail_register_kobj:
	kobject_put(charpriv->registers);
	charpriv->registers = NULL;

fail_alloc_kobj:
	kfree(priv);
	return -ENOMEM;
}

static int spidrv_remove(struct platform_device *pdev)
{
	unsigned int minor = atomic_read(&dev_cnt);
	dev_t curr_dev;
	struct platform_private *priv = platform_get_drvdata(pdev);
	spidrv_private *charpriv = &priv->charpriv;

	PINFO("In remove() function\n");

	atomic_dec(&dev_cnt);
	curr_dev = MKDEV(MAJOR(spidrv_device_num), MINOR(spidrv_device_num) + minor);
	PDEBUG("current device number: major = %d, minor = %d\n", MAJOR(curr_dev), MINOR(curr_dev));

	// удалить устройство
	device_destroy(spidrv_class, priv->charpriv.cdev.dev);

	// разрегистрация символьного драйвера
	cdev_del(&priv->charpriv.cdev);

	// освобождение адресного пространства
	iounmap(charpriv->io_base);
	release_mem_region(charpriv->resource.start, resource_size(&charpriv->resource));

	// удалить файлы и каталоги sysfs
	cleanup_regs();
	kobject_put(charpriv->registers);
	platform_set_drvdata(pdev, NULL);

	/* Free the device specific structure */
	kfree(priv);

	return 0;
}


static int __init spidrv_init(void)
{
	int res;

	res = alloc_chrdev_region(&spidrv_device_num, SPIDRV_FIRST_MINOR, SPIDRV_N_MINORS, DRIVER_NAME);
	if(res<0) {
		PERR("register device no failed\n");
		return -1;
	}
	spidrv_major = MAJOR(spidrv_device_num);

	spidrv_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (!spidrv_class) {
		PERR("Failed to create the class\n");
		res = PTR_ERR(spidrv_class);
		return res;
	}

	res = platform_driver_register(&spidrv_driver);
	if (res) {
		PERR("Failed to register the platform driver\n");
		return res;
	}

	PINFO("INIT\n");
	return 0;
}

static void __exit spidrv_exit(void)
{
	PINFO("EXIT\n");

	// TODO: FOR DEBUG ONLY
	// platform_driver_unregister(&spidrv_driver);
	class_destroy(spidrv_class);
	unregister_chrdev_region(spidrv_device_num, SPIDRV_N_MINORS);
}

module_init(spidrv_init);
module_exit(spidrv_exit);
