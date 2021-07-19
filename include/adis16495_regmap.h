/**
  * Copyright (c) 2021 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		adis16495_regmap.h
  * @author		Alex Nolan (alex.nolan@analog.com)
  * @brief 		Register map header for ADIS16495 IMU.
 **/


/* #define register_name [15:8] = page id, [7:0] = reg addr */

#ifndef __ADIS16495_REGMAP_H_
#define __ADIS16495_REGMAP_H_

#ifdef __cplusplus
extern "C" {
#endif

#define     REG_PAGE_ID                 0x0000
#define     REG_DATA_CNT                0x0004
#define     REG_SYS_E_FLAG              0x0008
#define     REG_DIAG_STS                0x000A
#define     REG_TEMP_OUT                0x000E
#define     REG_X_GYRO_LOW              0x0010
#define     REG_X_GYRO_OUT              0x0012
#define     REG_Y_GYRO_LOW              0x0014
#define     REG_Y_GYRO_OUT              0x0016
#define     REG_Z_GYRO_LOW              0x0018
#define     REG_Z_GYRO_OUT              0x001A
#define     REG_X_ACCL_LOW              0x001C
#define     REG_X_ACCL_OUT              0x001E
#define     REG_Y_ACCL_LOW              0x0020
#define     REG_Y_ACCL_OUT              0x0022
#define     REG_Z_ACCL_LOW              0x0024
#define     REG_Z_ACCL_OUT              0x0026
#define     REG_TIME_STAMP              0x0028
#define     REG_CRC_LWR                 0x002A
#define     REG_CRC_UPR                 0x002C
#define     REG_X_DELTANG_LOW           0x0040
#define     REG_X_DELTANG_OUT           0x0042
#define     REG_Y_DELTANG_LOW           0x0044
#define     REG_Y_DELTANG_OUT           0x0046
#define     REG_Z_DELTANG_LOW           0x0048
#define     REG_Z_DELTANG_OUT           0x004A
#define     REG_X_DELTVEL_LOW           0x004C
#define     REG_X_DELTVEL_OUT           0x004E
#define     REG_Y_DELTVEL_LOW           0x0050
#define     REG_Y_DELTVEL_OUT           0x0052
#define     REG_Z_DELTVEL_LOW           0x0054
#define     REG_Z_DELTVEL_OUT           0x0056
#define     REG_BURST_CMD               0x007C
#define     REG_PROD_ID                 0x007E
#define     REG_X_GYRO_SCALE            0x0204
#define     REG_Y_GYRO_SCALE            0x0206
#define     REG_Z_GYRO_SCALE            0x0208
#define     REG_X_ACCL_SCALE            0x020A
#define     REG_Y_ACCL_SCALE            0x020C
#define     REG_Z_ACCL_SCALE            0x020E
#define     REG_XG_BIAS_LOW             0x0210
#define     REG_XG_BIAS_HIGH            0x0212
#define     REG_YG_BIAS_LOW             0x0214
#define     REG_YG_BIAS_HIGH            0x0216
#define     REG_ZG_BIAS_LOW             0x0218
#define     REG_ZG_BIAS_HIGH            0x021A
#define     REG_XA_BIAS_LOW             0x021C
#define     REG_XA_BIAS_HIGH            0x021E
#define     REG_YA_BIAS_LOW             0x0220
#define     REG_YA_BIAS_HIGH            0x0222
#define     REG_ZA_BIAS_LOW             0x0224
#define     REG_ZA_BIAS_HIGH            0x0226
#define     REG_USER_SCR_1              0x0274
#define     REG_USER_SCR_2              0x0276
#define     REG_USER_SCR_3              0x0278
#define     REG_USER_SCR_4              0x027A
#define     REG_FLSHCNT_LOW             0x027C
#define     REG_FLSHCNT_HIGH            0x027E
#define     REG_GLOB_CMD                0x0302
#define     REG_FNCTIO_CTRL             0x0306
#define     REG_GPIO_CTRL               0x0308
#define     REG_CONFIG                  0x030A
#define     REG_DEC_RATE                0x030C
#define     REG_NULL_CNFG               0x030E
#define     REG_SYNC_SCALE              0x0310
#define     REG_RANG_MDL                0x0312
#define     REG_FILTR_BNK_0             0x0316
#define     REG_FILTR_BNK_1             0x0318
#define     REG_FIRM_REV                0x0378
#define     REG_FIRM_DM                 0x037A
#define     REG_FIRM_Y                  0x037C
#define     REG_BOOT_REV                0x037E
#define     REG_CAL_SIGTR_LWR           0x0404
#define     REG_CAL_SIGTR_UPR           0x0406
#define     REG_CAL_DRVTN_LWR           0x0408
#define     REG_CAL_DRVTN_UPR           0x040A
#define     REG_CODE_SIGTR_LWR          0x040C
#define     REG_CODE_SIGTR_UPR          0x040E
#define     REG_CODE_DRVTN_LWR          0x0410
#define     REG_CODE_DRVTN_UPR          0x0412
#define     REG_SERIAL_NUM              0x0420

/* Resolution */
#define     IMU_RES_TEMP_46x          (0.1) // 1.0/10
#define     IMU_OFFSET_TEMP_46x       (0)
#define     IMU_RES_ACCL16_465        (0.00025)
#define     IMU_RES_ACCL16_467        (0.00125)
#define     IMU_RES_GYRO16_46x1       (0.00625)
#define     IMU_RES_GYRO16_46x2       (0.025)
#define     IMU_RES_GYRO16_46x3       (0.1)
#define     IMU_RES_ACCL32_465        (0.00025/(1<<16))
#define     IMU_RES_ACCL32_467        (0.00125/(1<<16))
#define     IMU_RES_GYRO32_46x1       (0.00625/(1<<16))
#define     IMU_RES_GYRO32_46x2       (0.025/(1<<16))
#define     IMU_RES_GYRO32_46x3       (0.1/(1<<16))

#define     IMU_RES_TEMP_47x          (0.1) // 1.0/10
#define     IMU_OFFSET_TEMP_47x       (0)
#define     IMU_RES_ACCL16_475        (0.00025)
#define     IMU_RES_ACCL16_477        (0.00125)
#define     IMU_RES_GYRO16_47x1       (0.00625)
#define     IMU_RES_GYRO16_47x2       (0.025)
#define     IMU_RES_GYRO16_47x3       (0.1)
#define     IMU_RES_ACCL32_475        (0.00025/(1<<16))
#define     IMU_RES_ACCL32_477        (0.00125/(1<<16))
#define     IMU_RES_GYRO32_47x1       (0.00625/(1<<16))
#define     IMU_RES_GYRO32_47x2       (0.025/(1<<16))
#define     IMU_RES_GYRO32_47x3       (0.1/(1<<16))

#define     IMU_RES_TEMP_49x          (0.0125) // 1.0/80
#define     IMU_OFFSET_TEMP_49x       (25)
#define     IMU_RES_ACCL16_495        (0.00025)
#define     IMU_RES_ACCL16_497        (0.00125)
#define     IMU_RES_GYRO16_49x1       (0.00625)
#define     IMU_RES_GYRO16_49x2       (0.025)
#define     IMU_RES_GYRO16_49x3       (0.1)
#define     IMU_RES_ACCL32_495        (0.00025/(1<<16))
#define     IMU_RES_ACCL32_497        (0.00125/(1<<16))
#define     IMU_RES_GYRO32_49x1       (0.00625/(1<<16))
#define     IMU_RES_GYRO32_49x2       (0.025/(1<<16))
#define     IMU_RES_GYRO32_49x3       (0.1/(1<<16))

#define     IMU_RES_TEMP_50x          (0.1) // 1.0/10
#define     IMU_OFFSET_TEMP_50x       (0)
#define     IMU_RES_ACCL16_505        (0.00025)
#define     IMU_RES_ACCL16_507        (0.00125)
#define     IMU_RES_GYRO16_50x1       (0.00625)
#define     IMU_RES_GYRO16_50x2       (0.025)
#define     IMU_RES_GYRO16_50x3       (0.1)
#define     IMU_RES_ACCL32_505        (0.00025/(1<<16))
#define     IMU_RES_ACCL32_507        (0.00125/(1<<16))
#define     IMU_RES_GYRO32_50x1       (0.00625/(1<<16))
#define     IMU_RES_GYRO32_50x2       (0.025/(1<<16))
#define     IMU_RES_GYRO32_50x3       (0.1/(1<<16))

#define     IMU_RES_TEMP_54x          (1.0/140) //1.0/140
#define     IMU_OFFSET_TEMP_54x       (25)
#define     IMU_RES_ACCL16_545        (0.00025)
#define     IMU_RES_ACCL16_547        (0.00125)
#define     IMU_RES_GYRO16_54x1       (0.00625)
#define     IMU_RES_GYRO16_54x2       (0.025)
#define     IMU_RES_GYRO16_54x3       (0.1)
#define     IMU_RES_ACCL32_545        (0.00025/(1<<16))
#define     IMU_RES_ACCL32_547        (0.00125/(1<<16))
#define     IMU_RES_GYRO32_54x1       (0.00625/(1<<16))
#define     IMU_RES_GYRO32_54x2       (0.025/(1<<16))
#define     IMU_RES_GYRO32_54x3       (0.1/(1<<16))

/* REG STALL TIMES */

#define     IMU_STALL_US_FNCTIO       (340)
#define     IMU_STALL_US_FILTBNK0     (65)
#define     IMU_STALL_US_FILTBNK1     (65)
#define     IMU_STALL_US_NULLCFG      (71)
#define     IMU_STALL_US_SYNC_SCALE   (340)
#define     IMU_STALL_US_DEC_RATE     (340)
#define     IMU_STALL_US_GPIO_CTRL    (45)
#define     IMU_STALL_US_CONFIG       (45)
#define     IMU_STALL_US_GLOB_CMD     (1120)

/* BURST READ FRAME LENGTH 
  BRF Data Format (fSCLK > 3.6 MHz) = 20 * 2 = 40 bytes
  */
#define     MAX_BRF_LEN_BYTES         (40)

#define     IMU_MIN_PAGE_ID           (0)
#define     IMU_MAX_PAGE_ID           (3)

/* REG_BOOT_REV */
#define     IMU_BOOT_REV_MAJOR(val)     ( ((val) >> 8 ) & 0xFF )
#define     IMU_BOOT_REV_MINOR(val)     ( (val) & 0xFF )

/* REG_FIRM */
#define     IMU_FIRM_REV_UPPER(val)     ( ((val) >> 8 ) & 0xFF )
#define     IMU_FIRM_REV_LOWER(val)     ( (val) & 0xFF )
#define     IMU_FIRM_DAY(val)           ( (val) & 0xFF )
#define     IMU_FIRM_MONTH(val)         ( ((val) >> 8 ) & 0xFF )

/* REG_RANG_MDL */
#define     IMU_RANG_MDL(id)            ((id == 0x3) ? "±125°/sec" : (id == 0x7) ? "±450°/sec" : (id == 0xF) ? "±2000°/sec" : "UNKNOWN")

/* REG_SYS_E_FLAG */
#define     BITM_SYS_E_FLAG_WDG_TMR             0x8000
#define     BITM_SYS_E_FLAG_SYNC_ERR            0x0100
#define     BITM_SYS_E_FLAG_PROC_OVERRUN        0x0080
#define     BITM_SYS_E_FLAG_FLSH_MEM_UPD        0x0040
#define     BITM_SYS_E_FLAG_SENSOR_TEST         0x0020
#define     BITM_SYS_E_FLAG_SPI_COMM            0x0008
#define     BITM_SYS_E_FLAG_SRAM_CRC            0x0004
#define     BITM_SYS_E_FLAG_BOOT_MEM            0x0002

/* REG_DIAG_STS */
#define     BITM_DIAG_STS_Z_ACCL                0x0020
#define     BITM_DIAG_STS_Y_ACCL                0x0010
#define     BITM_DIAG_STS_X_ACCL                0x0008
#define     BITM_DIAG_STS_Z_GYRO                0x0004
#define     BITM_DIAG_STS_Y_GYRO                0x0002
#define     BITM_DIAG_STS_X_GYRO                0x0001

/* REG_GLOB_CMG */
#define     BITP_GLOB_CMD_SOFT_RST              (7)
#define     BITP_GLOB_CMD_CLR_USR_CALIB         (6)
#define     BITP_GLOB_CMD_FLASH_MEM_UPD         (3)
#define     BITP_GLOB_CMD_SELF_TEST             (1)
#define     BITP_GLOB_CMD_BIAS_CORR_UPD         (0)
#define     BITM_GLOB_CMD_SOFT_RST              ((0x1) << BITP_GLOB_CMD_SOFT_RST)
#define     BITM_GLOB_CMD_CLR_USR_CALIB         ((0x1) << BITP_GLOB_CMD_CLR_USR_CALIB)
#define     BITM_GLOB_CMD_FLASH_MEM_UPD         ((0x1) << BITP_GLOB_CMD_FLASH_MEM_UPD)
#define     BITM_GLOB_CMD_SELF_TEST             ((0x1) << BITP_GLOB_CMD_SELF_TEST)
#define     BITM_GLOB_CMD_BIAS_CORR_UPD         ((0x1) << BITP_GLOB_CMD_BIAS_CORR_UPD)

/* REG_FNCTIO_CTRL */
#define     BITP_FNCTIO_CTRL_SYNC_CLK_MODE      (8)
#define     BITP_FNCTIO_CTRL_SYNC_CLK_EN        (7)
#define     BITP_FNCTIO_CTRL_SYNC_CLK_POL       (6)
#define     BITP_FNCTIO_CTRL_SYNC_CLK_DIO       (4)
#define     BITP_FNCTIO_CTRL_DATA_RDY_EN        (3)
#define     BITP_FNCTIO_CTRL_DATA_RDY_POL       (2)
#define     BITP_FNCTIO_CTRL_DATA_RDY_DIO       (0)
#define     BITM_FNCTIO_CTRL_SYNC_CLK_MODE      ((0x1) << BITP_FNCTIO_CTRL_SYNC_CLK_MODE)
#define     BITM_FNCTIO_CTRL_SYNC_CLK_EN        ((0x1) << BITP_FNCTIO_CTRL_SYNC_CLK_EN)
#define     BITM_FNCTIO_CTRL_SYNC_CLK_POL       ((0x1) << BITP_FNCTIO_CTRL_SYNC_CLK_POL)
#define     BITM_FNCTIO_CTRL_SYNC_CLK_DIO       ((0x3) << BITP_FNCTIO_CTRL_SYNC_CLK_DIO)
#define     BITM_FNCTIO_CTRL_DATA_RDY_EN        ((0x1) << BITP_FNCTIO_CTRL_DATA_RDY_EN)
#define     BITM_FNCTIO_CTRL_DATA_RDY_POL       ((0x1) << BITP_FNCTIO_CTRL_DATA_RDY_POL)
#define     BITM_FNCTIO_CTRL_DATA_RDY_DIO       ((0x3) << BITP_FNCTIO_CTRL_DATA_RDY_DIO)

/* REG_GPIO_CTRL */
#define     BITP_GPIO_CTRL_DIO4_DATA            (7)
#define     BITP_GPIO_CTRL_DIO3_DATA            (6)
#define     BITP_GPIO_CTRL_DIO2_DATA            (5)
#define     BITP_GPIO_CTRL_DIO1_DATA            (4)
#define     BITP_GPIO_CTRL_DIO4_DIR             (3)
#define     BITP_GPIO_CTRL_DIO3_DIR             (2)
#define     BITP_GPIO_CTRL_DIO2_DIR             (1)
#define     BITP_GPIO_CTRL_DIO1_DIR             (0)
#define     BITM_GPIO_CTRL_DIO4_DATA            ((0x1) << BITP_GPIO_CTRL_DIO4_DATA)
#define     BITM_GPIO_CTRL_DIO3_DATA            ((0x1) << BITP_GPIO_CTRL_DIO3_DATA)
#define     BITM_GPIO_CTRL_DIO2_DATA            ((0x1) << BITP_GPIO_CTRL_DIO2_DATA)
#define     BITM_GPIO_CTRL_DIO1_DATA            ((0x1) << BITP_GPIO_CTRL_DIO1_DATA)
#define     BITM_GPIO_CTRL_DIO4_DIR             ((0x1) << BITP_GPIO_CTRL_DIO4_DIR)
#define     BITM_GPIO_CTRL_DIO3_DIR             ((0x1) << BITP_GPIO_CTRL_DIO3_DIR)
#define     BITM_GPIO_CTRL_DIO2_DIR             ((0x1) << BITP_GPIO_CTRL_DIO2_DIR)
#define     BITM_GPIO_CTRL_DIO1_DIR             ((0x1) << BITP_GPIO_CTRL_DIO1_DIR)

/* REG_CONFIG */
#define     BITP_CONFIG_LIN_G_COMP              (7)
#define     BITP_CONFIG_PNT_PERC_ALIGN          (6)
#define     BITM_CONFIG_LIN_G_COMP              ((0x1) << BITP_CONFIG_LIN_G_COMP)
#define     BITM_CONFIG_PNT_PERC_ALIGN          ((0x1) << BITP_CONFIG_PNT_PERC_ALIGN)

/* NULL_CNFG */
#define     BITP_NULL_CNFG_EN_ZA                (13)
#define     BITP_NULL_CNFG_EN_YA                (12)
#define     BITP_NULL_CNFG_EN_XA                (11)
#define     BITP_NULL_CNFG_EN_ZG                (10)
#define     BITP_NULL_CNFG_EN_YG                (9)
#define     BITP_NULL_CNFG_EN_XG                (8)
#define     BITM_NULL_CNFG_EN_ZA                ((0x1) << BITP_NULL_CNFG_EN_ZA)
#define     BITM_NULL_CNFG_EN_YA                ((0x1) << BITP_NULL_CNFG_EN_YA)
#define     BITM_NULL_CNFG_EN_XA                ((0x1) << BITP_NULL_CNFG_EN_XA)
#define     BITM_NULL_CNFG_EN_ZG                ((0x1) << BITP_NULL_CNFG_EN_ZG)
#define     BITM_NULL_CNFG_EN_YG                ((0x1) << BITP_NULL_CNFG_EN_YG)
#define     BITM_NULL_CNFG_EN_XG                ((0x1) << BITP_NULL_CNFG_EN_XG)

#ifdef __cplusplus
}
#endif
#endif