#ifndef SM2683LT_IOCTL_H
#define SM2683LT_IOCTL_H

#include <asm/types.h>
/*
 * User-space can't include linux/mmc/core.h, so define a simple command
 * struct that can be used to populate the real struct mmc_command
 */

#define SM2683LT_IOC_MAGIC   's'

struct sd_command {
	__u32		opcode;
	__u32		arg;
	__u32		resp[4];
	unsigned int	flags;	/* expected response type */

#ifndef __KERNEL__
#define MMC_RSP_PRESENT (1 << 0)
#define MMC_RSP_136     (1 << 1)                /* 136 bit response */
#define MMC_RSP_CRC     (1 << 2)                /* expect valid crc */
#define MMC_RSP_BUSY    (1 << 3)                /* card may send busy */
#define MMC_RSP_OPCODE  (1 << 4)                /* response contains opcode */

#define MMC_CMD_MASK    (3 << 5)                /* non-SPI command type */
#define MMC_CMD_AC      (0 << 5)
#define MMC_CMD_ADTC    (1 << 5)
#define MMC_CMD_BC      (2 << 5)
#define MMC_CMD_BCR     (3 << 5)

#define MMC_RSP_SPI_S1  (1 << 7)                /* one status byte */
#define MMC_RSP_SPI_S2  (1 << 8)                /* second byte */
#define MMC_RSP_SPI_B4  (1 << 9)                /* four data bytes */
#define MMC_RSP_SPI_BUSY (1 << 10)              /* card may send busy */

/*
 * These are the native response types, and correspond to valid bit
 * patterns of the above flags.  One additional valid pattern
 * is all zeros, which means we don't expect a response.
 */
#define MMC_RSP_NONE    (0)
#define MMC_RSP_R1      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R1B     (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE|MMC_RSP_BUSY)
#define MMC_RSP_R2      (MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC)
#define MMC_RSP_R3      (MMC_RSP_PRESENT)
#define MMC_RSP_R4      (MMC_RSP_PRESENT)
#define MMC_RSP_R5      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R6      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R7      (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)

#define mmc_resp_type(cmd)      ((cmd)->flags & (MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC|MMC_RSP_BUSY|MMC_RSP_OPCODE))
/*
 * These are the SPI response types for MMC, SD, and SDIO cards.
 * Commands return R1, with maybe more info.  Zero is an error type;
 * callers must always provide the appropriate MMC_RSP_SPI_Rx flags.
 */
#define MMC_RSP_SPI_R1  (MMC_RSP_SPI_S1)
#define MMC_RSP_SPI_R1B (MMC_RSP_SPI_S1|MMC_RSP_SPI_BUSY)
#define MMC_RSP_SPI_R2  (MMC_RSP_SPI_S1|MMC_RSP_SPI_S2)
#define MMC_RSP_SPI_R3  (MMC_RSP_SPI_S1|MMC_RSP_SPI_B4)
#define MMC_RSP_SPI_R4  (MMC_RSP_SPI_S1|MMC_RSP_SPI_B4)
#define MMC_RSP_SPI_R5  (MMC_RSP_SPI_S1|MMC_RSP_SPI_S2)
#define MMC_RSP_SPI_R7  (MMC_RSP_SPI_S1|MMC_RSP_SPI_B4)

#define mmc_spi_resp_type(cmd)  ((cmd)->flags & \
                (MMC_RSP_SPI_S1|MMC_RSP_SPI_BUSY|MMC_RSP_SPI_S2|MMC_RSP_SPI_B4))

/*
 * These are the command types.
 */
#define mmc_cmd_type(cmd)       ((cmd)->flags & MMC_CMD_MASK)

#endif /* __KERNEL__ */

	unsigned int	retries;
	unsigned int	error;

	/* not present in mmc_command - for data xfer */
	union
	{
	void *		read_data;
	const void *	write_data;
	} data;
	int		data_len;
};

/* supported ioctls */

#define SM2683LT_SD_COMMAND	_IOWR(SM2683LT_IOC_MAGIC,  0, struct sd_command *)	// SD command pass-through
#define SM2683LT_SET_POWER	_IOW(SM2683LT_IOC_MAGIC,  1, int)	// Cut clock / tri-state bus / cut power

#endif /* SM2683LT_IOCTL_H */
