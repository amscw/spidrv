/*
 * spidrv_ioctl.h
 *
 *  Created on: 31.10.2017
 *      Author: user
 */

#ifndef SPIDRV_IOCTL_H_
#define SPIDRV_IOCTL_H_

#include <linux/types.h>
#include <linux/ioctl.h>

typedef struct
{
	union
	{
		struct
		{
			/* u32 */ unsigned CR: 1;
			/* u32 */ unsigned SR_SPI_RDY: 1;
			/* u32 */ unsigned SR_SPI_DRE: 1;
			/* u32 */ unsigned SR_SPI_TRC: 1;
			/* u32 */ unsigned IR_SPI_RDY: 1;
			/* u32 */ unsigned IR_SPI_DRE: 1;
			/* u32 */ unsigned IR_SPI_TRC: 1;
			/* u32 */ unsigned RST: 1;
			/* u32 */ unsigned reserved: 24;
		} __attribute__((__packed__)) bits ;
		/* u32 */ unsigned int value;
	} reg;
} SPI_CONTROL_TypeDef;

typedef struct
{
	union
	{
		struct
		{
			/* u32 */ unsigned DW: 5;
			/* u32 */ unsigned START_WAIT: 4;
			/* u32 */ unsigned END_WAIT: 4;
			/* u32 */ unsigned CLK_DIV: 8;
			/* u32 */ unsigned CS_MIN_WIDTH: 4;
			/* u32 */ unsigned CPOL: 1;
			/* u32 */ unsigned CPHA: 1;
			/* u32 */ unsigned MSB_FIRST: 1;
			/* u32 */ unsigned EN_CS_MANUAL: 1;
			/* u32 */ unsigned reserved: 3;

		}__attribute__((__packed__)) bits ;
		/* u32 */ unsigned int value;
	} reg;
} SPI_SETTINGS_TypeDef ;

/* Use 'k' as spidev magic number */
#define SPIDRV_IOC_MAGIC 'k'

/* SPIDRV commands */
#define SPIDRV_IOCRESET 	_IO(SPIDRV_IOC_MAGIC, 0)
#define SPIDRV_IOCSSETTINGS _IOW(SPIDRV_IOC_MAGIC, 1, SPI_SETTINGS_TypeDef)

#define SPIDRV_IOC_MAXNR 2

#endif /* SPIDRV_IOCTL_H_ */
