/* #define register_name [15:8] = page id, [7:0] = reg addr */

#define		REG_PAGE_ID					0x0000
#define		REG_DATA_CNT				0x0004
#define		REG_SYS_E_FLAG				0x0008
#define		REG_DIAG_STS				0x000A
#define		REG_TEMP_OUT				0x000E
#define		REG_X_GYRO_LOW				0x0010
#define		REG_X_GYRO_OUT				0x0012
#define		REG_Y_GYRO_LOW				0x0014
#define		REG_Y_GYRO_OUT				0x0016
#define		REG_Z_GYRO_LOW				0x0018
#define		REG_Z_GYRO_OUT				0x001A
#define		REG_X_ACCL_LOW				0x001C
#define		REG_X_ACCL_OUT				0x001E
#define		REG_Y_ACCL_LOW				0x0020
#define		REG_Y_ACCL_OUT				0x0022
#define		REG_Z_ACCL_LOW				0x0024
#define		REG_Z_ACCL_OUT				0x0026
#define		REG_TIME_STAMP				0x0028
#define		REG_CRC_LWR					0x002A
#define		REG_CRC_UPR					0x002C
#define		REG_X_DELTANG_LOW			0x0040
#define		REG_X_DELTANG_OUT			0x0042
#define		REG_Y_DELTANG_LOW			0x0044
#define		REG_Y_DELTANG_OUT			0x0046
#define		REG_Z_DELTANG_LOW			0x0048
#define		REG_Z_DELTANG_OUT			0x004A
#define		REG_X_DELTVEL_LOW			0x004C
#define		REG_X_DELTVEL_OUT			0x004E
#define		REG_Y_DELTVEL_LOW			0x0050
#define		REG_Y_DELTVEL_OUT			0x0052
#define		REG_Z_DELTVEL_LOW			0x0054
#define		REG_Z_DELTVEL_OUT			0x0056
#define		REG_BURST_CMD				0x007C
#define		REG_PROD_ID					0x007E
#define		REG_X_GYRO_SCALE			0x0204
#define		REG_Y_GYRO_SCALE			0x0206
#define		REG_Z_GYRO_SCALE			0x0208
#define		REG_X_ACCL_SCALE			0x020A
#define		REG_Y_ACCL_SCALE			0x020C
#define		REG_Z_ACCL_SCALE			0x020E
#define		REG_XG_BIAS_LOW				0x0210
#define		REG_XG_BIAS_HIGH			0x0212
#define		REG_YG_BIAS_LOW				0x0214
#define		REG_YG_BIAS_HIGH			0x0216
#define		REG_ZG_BIAS_LOW				0x0218
#define		REG_ZG_BIAS_HIGH			0x021A
#define		REG_XA_BIAS_LOW				0x021C
#define		REG_XA_BIAS_HIGH			0x021E
#define		REG_YA_BIAS_LOW				0x0220
#define		REG_YA_BIAS_HIGH			0x0222
#define		REG_ZA_BIAS_LOW				0x0224
#define		REG_ZA_BIAS_HIGH			0x0226
#define		REG_USER_SCR_1				0x0274
#define		REG_USER_SCR_2				0x0276
#define		REG_USER_SCR_3				0x0278
#define		REG_USER_SCR_4				0x027A
#define		REG_FLSHCNT_LOW				0x027C
#define		REG_FLSHCNT_HIGH			0x027E
#define		REG_GLOB_CMD				0x0302
#define		REG_FNCTIO_CTRL				0x0306
#define		REG_GPIO_CTRL				0x0308
#define		REG_CONFIG					0x030A
#define		REG_DEC_RATE				0x030C
#define		REG_NULL_CNFG				0x030E
#define		REG_SYNC_SCALE				0x0310
#define		REG_RANG_MDL				0x0312
#define		REG_FILTR_BNK_0				0x0316
#define		REG_FILTR_BNK_1				0x0318
#define		REG_FIRM_REV				0x0378
#define		REG_FIRM_DM					0x037A
#define		REG_FIRM_Y					0x037C
#define		REG_BOOT_REV				0x037E
#define		REG_CAL_SIGTR_LWR			0x0404
#define		REG_CAL_SIGTR_UPR			0x0406
#define		REG_CAL_DRVTN_LWR			0x0408
#define		REG_CAL_DRVTN_UPR			0x040A
#define		REG_CODE_SIGTR_LWR			0x040C
#define		REG_CODE_SIGTR_UPR			0x040E
#define		REG_CODE_DRVTN_LWR			0x0410
#define		REG_CODE_DRVTN_UPR			0x0412
#define		REG_SERIAL_NUM				0x0420

/* REG_BOOT_REV */
#define 	IMU_BOOT_REV_MAJOR(val)     ( ((val) >> 8 ) & 0xFF )
#define 	IMU_BOOT_REV_MINOR(val)     ( (val) & 0xFF )

/* REG_FIRM */
#define 	IMU_FIRM_REV_UPPER(val)       ( ((val) >> 8 ) & 0xFF )
#define 	IMU_FIRM_REV_LOWER(val)       ( (val) & 0xFF )
#define 	IMU_FIRM_DAY(val)             ( (val) & 0xFF )
#define 	IMU_FIRM_MONTH(val)           ( ((val) >> 8 ) & 0xFF )

/* REG_RANG_MDL */
#define 	IMU_RANG_MDL(id)         ((id == 0x3) ? "±125°/sec" : (id == 0x7) ? "±450°/sec" : (id == 0xF) ? "±2000°/sec" : "UNKNOWN")
