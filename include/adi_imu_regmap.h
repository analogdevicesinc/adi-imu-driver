/**
  * Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
  * This software is proprietary to Analog Devices, Inc. and its licensors.
  *
  * Use of this file is governed by the license agreement
  * included in this repository.
  *
  * @file		adi_imu_regmap.h
  * @author		Sundar Palani (sundar.palani@analog.com)
  * @brief 		Register map header for ADIS16xxx IMU.
 **/

#ifndef __ADI_IMU_REGMAP_H_
#define __ADI_IMU_REGMAP_H_

/* IMU Register Map */
/* #define register_name [15:8] = page id, [7:0] = reg addr */

/* Page Identifier
  * For 16495, PAGE_ID register
  * For 16470/16500, this is a Reserved register; FW guards any read/write access to this
  **/
#define     REG_PAGE_ID                 0x0000

/** New Data Counter
  * For 16495, DATA_CNT register
  * For 16470/16500, DATA_CNTR register
  **/
#define     REG_DATA_CNTR_47x               0x0022
#define     REG_DATA_CNT_49x                0x0004
#define     REG_DATA_CNTR_50x               0x0022

/** System Error flags and Self Test Error flags
  * For 16495, SYS_E_FLAG register for System Error Flags, DIAG_STS for Self Test Error Flags
  * For 16470/16500, equivalent to DIAG_STAT register
 **/
#define     REG_SYS_E_FLAG_49x              0x0008
#define     REG_DIAG_STS_49x                0x000A
#define     REG_DIAG_STAT_47x               0x0002
#define     REG_DIAG_STAT_50x               0x0002
#define     REG_DIAG_STAT(val) ((val==1647) ? REG_DIAG_STAT_47x : (val==1650) ? REG_DIAG_STAT_50x : 0x0000)

/* Temperature = TEMP register */
#define     REG_TEMP_OUT_47x                0x001C
#define     REG_TEMP_OUT_49x                0x000E
#define     REG_TEMP_OUT_50x                0x001C
#define     REG_TEMP_OUT(val) ((val==1647) ? REG_TEMP_OUT_47x : (val==1649) ? REG_TEMP_OUT_49x : (val==1650) ? REG_TEMP_OUT_50x : 0x0000)

/* X-axis Gyroscope, low word = X_GYRO_LOW register */
#define     REG_X_GYRO_LOW_47x              0x0004
#define     REG_X_GYRO_LOW_49x              0x0010
#define     REG_X_GYRO_LOW_50x              0x0004
#define     REG_X_GYRO_LOW(val) ((val==1647) ? REG_X_GYRO_LOW_47x : (val==1649) ? REG_X_GYRO_LOW_49x : (val==1650) ? REG_X_GYRO_LOW_50x : 0x0000)

/* X-axis Gyroscope, high word = X_GYRO_OUT register */
#define    REG_X_GYRO_OUT_47x              0x0006
#define     REG_X_GYRO_OUT_49x              0x0012
#define     REG_X_GYRO_OUT_50x              0x0006
#define     REG_X_GYRO_OUT(val) ((val==1647) ? REG_X_GYRO_OUT_47x : (val==1649) ? REG_X_GYRO_OUT_49x : (val==1650) ? REG_X_GYRO_OUT_50x : 0x0000)

/* Y-axis Gyroscope, low word = Y_GYRO_LOW register */
#define     REG_Y_GYRO_LOW_47x              0x0008
#define     REG_Y_GYRO_LOW_49x              0x0014
#define     REG_Y_GYRO_LOW_50x              0x0008
#define     REG_Y_GYRO_LOW(val) ((val==1647) ? REG_Y_GYRO_LOW_47x : (val==1649) ? REG_Y_GYRO_LOW_49x : (val==1650) ? REG_Y_GYRO_LOW_50x : 0x0000)

/* Y-axis Gyroscope, high word = Y_GYRO_OUT register */
#define     REG_Y_GYRO_OUT_47x              0x000A
#define     REG_Y_GYRO_OUT_49x              0x0016
#define     REG_Y_GYRO_OUT_50x              0x000A
#define     REG_Y_GYRO_OUT(val) ((val==1647) ? REG_Y_GYRO_OUT_47x : (val==1649) ? REG_Y_GYRO_OUT_49x : (val==1650) ? REG_Y_GYRO_OUT_50x : 0x0000)

/* Z-axis Gyroscope, low word = Z_GYRO_LOW register */
#define     REG_Z_GYRO_LOW_47x              0x000C
#define     REG_Z_GYRO_LOW_49x              0x0018
#define     REG_Z_GYRO_LOW_50x              0x000C
#define     REG_Z_GYRO_LOW(val) ((val==1647) ? REG_Z_GYRO_LOW_47x : (val==1649) ? REG_Z_GYRO_LOW_49x : (val==1650) ? REG_Z_GYRO_LOW_50x : 0x0000)

/* Z-axis Gyroscope, high word = Z_GYRO_OUT register */
#define     REG_Z_GYRO_OUT_47x              0x000E
#define     REG_Z_GYRO_OUT_49x              0x001A
#define     REG_Z_GYRO_OUT_50x              0x000E
#define     REG_Z_GYRO_OUT(val) ((val==1647) ? REG_Z_GYRO_OUT_47x : (val==1649) ? REG_Z_GYRO_OUT_49x : (val==1650) ? REG_Z_GYRO_OUT_50x : 0x0000)

/* X-axis Accelerometer, low word = X_ACCL_LOW register */
#define     REG_X_ACCL_LOW_47x              0x0010
#define     REG_X_ACCL_LOW_49x              0x001C
#define     REG_X_ACCL_LOW_50x              0x0010
#define     REG_X_ACCL_LOW(val) ((val==1647) ? REG_X_ACCL_LOW_47x : (val==1649) ? REG_X_ACCL_LOW_49x : (val==1650) ? REG_X_ACCL_LOW_50x : 0x0000)

/* X-axis Accelerometer, high word = X_ACCL_OUT register */
#define     REG_X_ACCL_OUT_47x              0x0012
#define     REG_X_ACCL_OUT_49x              0x001E
#define     REG_X_ACCL_OUT_50x              0x0012
#define     REG_X_ACCL_OUT(val) ((val==1647) ? REG_X_ACCL_OUT_47x : (val==1649) ? REG_X_ACCL_OUT_49x : (val==1650) ? REG_X_ACCL_OUT_50x : 0x0000)

/* Y-axis Accelerometer, low word = Y_ACCL_LOW register */
#define     REG_Y_ACCL_LOW_47x              0x0014
#define     REG_Y_ACCL_LOW_49x              0x0020
#define     REG_Y_ACCL_LOW_50x              0x0014
#define     REG_Y_ACCL_LOW(val) ((val==1647) ? REG_Y_ACCL_LOW_47x : (val==1649) ? REG_Y_ACCL_LOW_49x : (val==1650) ? REG_Y_ACCL_LOW_50x : 0x0000)

/* Y-axis Accelerometer, high word = Y_ACCL_OUT register */
#define     REG_Y_ACCL_OUT_47x              0x0016
#define     REG_Y_ACCL_OUT_49x              0x0022
#define     REG_Y_ACCL_OUT_50x              0x0016
#define     REG_Y_ACCL_OUT(val) ((val==1647) ? REG_Y_ACCL_OUT_47x : (val==1649) ? REG_Y_ACCL_OUT_49x : (val==1650) ? REG_Y_ACCL_OUT_50x : 0x0000)

/* Z-axis Accelerometer, low word = Z_ACCL_LOW register */
#define     REG_Z_ACCL_LOW_47x              0x0018
#define     REG_Z_ACCL_LOW_49x              0x0024
#define     REG_Z_ACCL_LOW_50x              0x0018
#define     REG_Z_ACCL_LOW(val) ((val==1647) ? REG_Z_ACCL_LOW_47x : (val==1649) ? REG_Z_ACCL_LOW_49x : (val==1650) ? REG_Z_ACCL_LOW_50x : 0x0000)

/* Z-axis Accelerometer, high word = Z_ACCL_OUT register */
#define     REG_Z_ACCL_OUT_47x              0x001A
#define     REG_Z_ACCL_OUT_49x              0x0026
#define     REG_Z_ACCL_OUT_50x              0x001A
#define     REG_Z_ACCL_OUT(val) ((val==1647) ? REG_Z_ACCL_OUT_47x : (val==1649) ? REG_Z_ACCL_OUT_49x : (val==1650) ? REG_Z_ACCL_OUT_50x : 0x0000)

/* Time stamp = TIME_STAMP register */
#define     REG_TIME_STAMP_47x              0x001E
#define     REG_TIME_STAMP_49x              0x0028
#define     REG_TIME_STAMP_50x              0x001E
#define     REG_TIME_STAMP(val) ((val==1647) ? REG_TIME_STAMP_47x : (val==1649) ? REG_TIME_STAMP_49x : (val==1650) ? REG_TIME_STAMP_50x : 0x0000)

/** CRC (32-bits) lower and upper word
  * For 16470/16500, NONE
 **/
#define     REG_CRC_LWR_49x                 0x002A
#define     REG_CRC_UPR_49x                 0x002C

/* X-axis delta angle, low word = X_DELTANG_LOW register */
#define     REG_X_DELTANG_LOW_47x           0x0024
#define     REG_X_DELTANG_LOW_49x           0x0040
#define     REG_X_DELTANG_LOW_50x           0x0024
#define     REG_X_DELTANG_LOW(val) ((val==1647) ? REG_X_DELTANG_LOW_47x : (val==1649) ? REG_X_DELTANG_LOW_49x : (val==1650) ? REG_X_DELTANG_LOW_50x : 0x0000)

/* X-axis delta angle, high word = X_DELTANG_OUT register */
#define     REG_X_DELTANG_OUT_47x           0x0026
#define     REG_X_DELTANG_OUT_49x           0x0042
#define     REG_X_DELTANG_OUT_50x           0x0026
#define     REG_X_DELTANG_OUT(val) ((val==1647) ? REG_X_DELTANG_OUT_47x : (val==1649) ? REG_X_DELTANG_OUT_49x : (val==1650) ? REG_X_DELTANG_OUT_50x : 0x0000)

/* Y-axis delta angle, low word = Y_DELTANG_LOW register */
#define     REG_Y_DELTANG_LOW_47x           0x0028
#define     REG_Y_DELTANG_LOW_49x           0x0044
#define     REG_Y_DELTANG_LOW_50x           0x0028
#define     REG_Y_DELTANG_LOW(val) ((val==1647) ? REG_Y_DELTANG_LOW_47x : (val==1649) ? REG_Y_DELTANG_LOW_49x : (val==1650) ? REG_Y_DELTANG_LOW_50x : 0x0000)

/* Y-axis delta angle, high word = Y_DELTANG_OUT register */
#define     REG_Y_DELTANG_OUT_47x           0x002A
#define     REG_Y_DELTANG_OUT_49x           0x0046
#define     REG_Y_DELTANG_OUT_50x           0x002A
#define     REG_Y_DELTANG_OUT(val) ((val==1647) ? REG_Y_DELTANG_OUT_47x : (val==1649) ? REG_Y_DELTANG_OUT_49x : (val==1650) ? REG_Y_DELTANG_OUT_50x : 0x0000)

/* Z-axis delta angle, low word = Z_DELTANG_LOW register */
#define     REG_Z_DELTANG_LOW_47x           0x002C
#define     REG_Z_DELTANG_LOW_49x           0x0048
#define     REG_Z_DELTANG_LOW_50x           0x002C
#define     REG_Z_DELTANG_LOW(val) ((val==1647) ? REG_Z_DELTANG_LOW_47x : (val==1649) ? REG_Z_DELTANG_LOW_49x : (val==1650) ? REG_Z_DELTANG_LOW_50x : 0x0000)

/* Y-axis delta angle, high word = Z_DELTANG_OUT register */
#define     REG_Z_DELTANG_OUT_47x           0x002E
#define     REG_Z_DELTANG_OUT_49x           0x004A
#define     REG_Z_DELTANG_OUT_50x           0x002E
#define     REG_Z_DELTANG_OUT(val) ((val==1647) ? REG_Z_DELTANG_OUT_47x : (val==1649) ? REG_Z_DELTANG_OUT_49x : (val==1650) ? REG_Z_DELTANG_OUT_50x : 0x0000)

/* X-axis delta velocity, low word = X_DELTVEL_LOW register */
#define     REG_X_DELTVEL_LOW_47x           0x0030
#define     REG_X_DELTVEL_LOW_49x           0x004C
#define     REG_X_DELTVEL_LOW_50x           0x0030
#define     REG_X_DELTVEL_LOW(val) ((val==1647) ? REG_X_DELTVEL_LOW_47x : (val==1649) ? REG_X_DELTVEL_LOW_49x : (val==1650) ? REG_X_DELTVEL_LOW_50x : 0x0000)

/* X-axis delta velocity, high word = X_DELTVEL_OUT register */
#define     REG_X_DELTVEL_OUT_47x           0x0032
#define     REG_X_DELTVEL_OUT_49x           0x004E
#define     REG_X_DELTVEL_OUT_50x           0x0032
#define     REG_X_DELTVEL_OUT(val) ((val==1647) ? REG_X_DELTVEL_OUT_47x : (val==1649) ? REG_X_DELTVEL_OUT_49x : (val==1650) ? REG_X_DELTVEL_OUT_50x : 0x0000)

/* Y-axis delta velocity, low word = Y_DELTVEL_LOW register */
#define     REG_Y_DELTVEL_LOW_47x           0x0034
#define     REG_Y_DELTVEL_LOW_49x           0x0050
#define     REG_Y_DELTVEL_LOW_50x           0x0034
#define     REG_Y_DELTVEL_LOW(val) ((val==1647) ? REG_Y_DELTVEL_LOW_47x : (val==1649) ? REG_Y_DELTVEL_LOW_49x : (val==1650) ? REG_Y_DELTVEL_LOW_50x : 0x0000)

/* Y-axis delta velocity, high word = Y_DELTVEL_OUT register */
#define     REG_Y_DELTVEL_OUT_47x           0x0036
#define     REG_Y_DELTVEL_OUT_49x           0x0052
#define     REG_Y_DELTVEL_OUT_50x           0x0036
#define     REG_Y_DELTVEL_OUT(val) ((val==1647) ? REG_Y_DELTVEL_OUT_47x : (val==1649) ? REG_Y_DELTVEL_OUT_49x : (val==1650) ? REG_Y_DELTVEL_OUT_50x : 0x0000)

/* Z-axis delta velocity, low word = Z_DELTVEL_LOW register */
#define     REG_Z_DELTVEL_LOW_47x           0x0038
#define     REG_Z_DELTVEL_LOW_49x           0x0054
#define     REG_Z_DELTVEL_LOW_50x           0x0038
#define     REG_Z_DELTVEL_LOW(val) ((val==1647) ? REG_Z_DELTVEL_LOW_47x : (val==1649) ? REG_Z_DELTVEL_LOW_49x : (val==1650) ? REG_Z_DELTVEL_LOW_50x : 0x0000)

/* Z-axis delta velocity, high word = Z_DELTVEL_OUT register */
#define     REG_Z_DELTVEL_OUT_47x           0x003A
#define     REG_Z_DELTVEL_OUT_49x           0x0056
#define     REG_Z_DELTVEL_OUT_50x           0x003A
#define     REG_Z_DELTVEL_OUT(val) ((val==1647) ? REG_Z_DELTVEL_OUT_47x : (val==1649) ? REG_Z_DELTVEL_OUT_49x : (val==1650) ? REG_Z_DELTVEL_OUT_50x : 0x0000)

/** Burst read command
  * For 16495, BURST_CMD register
  * For 16470/16500, There is no BURST_CMD register, but 0x6800 is cmd to trigger burst read
 **/
#define     REG_BURST_CMD_47x               0x0068
#define     REG_BURST_CMD_49x               0x007C
#define     REG_BURST_CMD_50x               0x0068
#define     REG_BURST_CMD(val) ((val==1647) ? REG_BURST_CMD_47x : (val==1649) ? REG_BURST_CMD_49x : (val==1650) ? REG_BURST_CMD_50x : 0x0000)

/* Product ID = PROD_ID register */
#define     REG_PROD_ID_47x                 0x0072
#define     REG_PROD_ID_49x                 0x007E
#define     REG_PROD_ID_50x                 0x0072
#define     REG_PROD_ID(val) ((val==1647) ? REG_PROD_ID_47x : (val==1649) ? REG_PROD_ID_49x : (val==1650) ? REG_PROD_ID_50x : 0x0000)

/** Calibration, scale, x,y,z-axis Gyroscope
  * For 16495, X_GYRO_SCALE, Y_GYRO_SCALE, Z_GYRO_SCALE registers
  * For 16470/16500, NONE
 **/
#define     REG_X_GYRO_SCALE_49x            0x0204
#define     REG_Y_GYRO_SCALE_49x            0x0206
#define     REG_Z_GYRO_SCALE_49x            0x0208

/** Calibration, scale, x,y,z-axis Accelerometer
  * For 16495, X_ACCL_SCALE, Y_ACCL_SCALE, Z_ACCL_SCALE registers
  * For 16470/16500, NONE
 **/
#define     REG_X_ACCL_SCALE_49x            0x020A
#define     REG_Y_ACCL_SCALE_49x            0x020C
#define     REG_Z_ACCL_SCALE_49x            0x020E

/* Calibration, bias, gyroscope, x-axis, low word = XG_BIAS_LOW register */
#define     REG_XG_BIAS_LOW_47x             0x0040
#define     REG_XG_BIAS_LOW_49x             0x0210
#define     REG_XG_BIAS_LOW_50x             0x0040
#define     REG_XG_BIAS_LOW(val) ((val==1647) ? REG_XG_BIAS_LOW_47x : (val==1649) ? REG_XG_BIAS_LOW_49x : (val==1650) ? REG_XG_BIAS_LOW_50x : 0x0000)

/* Calibration, bias, gyroscope, x-axis, high word = XG_BIAS_HIGH register */
#define     REG_XG_BIAS_HIGH_47x            0x0042
#define     REG_XG_BIAS_HIGH_49x            0x0212
#define     REG_XG_BIAS_HIGH_50x            0x0042
#define     REG_XG_BIAS_HIGH(val) ((val==1647) ? REG_XG_BIAS_HIGH_47x : (val==1649) ? REG_XG_BIAS_HIGH_49x : (val==1650) ? REG_XG_BIAS_HIGH_50x : 0x0000)

/* Calibration, bias, gyroscope, y-axis, low word = YG_BIAS_LOW register */
#define     REG_YG_BIAS_LOW_47x             0x0044
#define     REG_YG_BIAS_LOW_49x             0x0214
#define     REG_YG_BIAS_LOW_50x             0x0044
#define     REG_YG_BIAS_LOW(val) ((val==1647) ? REG_YG_BIAS_LOW_47x : (val==1649) ? REG_YG_BIAS_LOW_49x : (val==1650) ? REG_YG_BIAS_LOW_50x : 0x0000)

/* Calibration, bias, gyroscope, y-axis, high word = YG_BIAS_HIGH register */
#define     REG_YG_BIAS_HIGH_47x            0x0046
#define     REG_YG_BIAS_HIGH_49x            0x0216
#define     REG_YG_BIAS_HIGH_50x            0x0046
#define     REG_YG_BIAS_HIGH(val) ((val==1647) ? REG_YG_BIAS_HIGH_47x : (val==1649) ? REG_YG_BIAS_HIGH_49x : (val==1650) ? REG_YG_BIAS_HIGH_50x : 0x0000)

/* Calibration, bias, gyroscope, z-axis, low word = ZG_BIAS_LOW register */
#define     REG_ZG_BIAS_LOW_47x             0x0048
#define     REG_ZG_BIAS_LOW_49x             0x0218
#define     REG_ZG_BIAS_LOW_50x             0x0048
#define     REG_ZG_BIAS_LOW(val) ((val==1647) ? REG_ZG_BIAS_LOW_47x : (val==1649) ? REG_ZG_BIAS_LOW_49x : (val==1650) ? REG_ZG_BIAS_LOW_50x : 0x0000)

/* Calibration, bias, gyroscope, z-axis, high word = ZG_BIAS_HIGH register */
#define     REG_ZG_BIAS_HIGH_47x            0x004A
#define     REG_ZG_BIAS_HIGH_49x            0x021A
#define     REG_ZG_BIAS_HIGH_50x            0x004A
#define     REG_ZG_BIAS_HIGH(val) ((val==1647) ? REG_ZG_BIAS_HIGH_47x : (val==1649) ? REG_ZG_BIAS_HIGH_49x : (val==1650) ? REG_ZG_BIAS_HIGH_50x : 0x0000)

/* Calibration, bias, accelerometer, x-axis, low word = YA_BIAS_LOW register */
#define     REG_XA_BIAS_LOW_47x             0x004C
#define     REG_XA_BIAS_LOW_49x             0x021C
#define     REG_XA_BIAS_LOW_50x             0x004C
#define     REG_XA_BIAS_LOW(val) ((val==1647) ? REG_XA_BIAS_LOW_47x : (val==1649) ? REG_XA_BIAS_LOW_49x : (val==1650) ? REG_XA_BIAS_LOW_50x : 0x0000)

/* Calibration, bias, accelerometer, x-axis, high word = XA_BIAS_HIGH register */
#define     REG_XA_BIAS_HIGH_47x            0x004E
#define     REG_XA_BIAS_HIGH_49x            0x021E
#define     REG_XA_BIAS_HIGH_50x            0x004E
#define     REG_XA_BIAS_HIGH(val) ((val==1647) ? REG_XA_BIAS_HIGH_47x : (val==1649) ? REG_XA_BIAS_HIGH_49x : (val==1650) ? REG_XA_BIAS_HIGH_50x : 0x0000)

/* Calibration, bias, accelerometer, y-axis, low word = YA_BIAS_LOW register */
#define     REG_YA_BIAS_LOW_47x             0x0050
#define     REG_YA_BIAS_LOW_49x             0x0220
#define     REG_YA_BIAS_LOW_50x             0x0050
#define     REG_YA_BIAS_LOW(val) ((val==1647) ? REG_YA_BIAS_LOW_47x : (val==1649) ? REG_YA_BIAS_LOW_49x : (val==1650) ? REG_YA_BIAS_LOW_50x : 0x0000)

/* Calibration, bias, accelerometer, y-axis, high word = YA_BIAS_HIGH register */
#define     REG_YA_BIAS_HIGH_47x            0x0052
#define     REG_YA_BIAS_HIGH_49x            0x0222
#define     REG_YA_BIAS_HIGH_50x            0x0052
#define     REG_YA_BIAS_HIGH(val) ((val==1647) ? REG_YA_BIAS_HIGH_47x : (val==1649) ? REG_YA_BIAS_HIGH_49x : (val==1650) ? REG_YA_BIAS_HIGH_50x : 0x0000)

/* Calibration, bias, accelerometer, z-axis, low word = ZA_BIAS_LOW register */
#define     REG_ZA_BIAS_LOW_47x             0x0054
#define     REG_ZA_BIAS_LOW_49x             0x0224
#define     REG_ZA_BIAS_LOW_50x             0x0054
#define     REG_ZA_BIAS_LOW(val) ((val==1647) ? REG_ZA_BIAS_LOW_47x : (val==1649) ? REG_ZA_BIAS_LOW_49x : (val==1650) ? REG_ZA_BIAS_LOW_50x : 0x0000)

/* Calibration, bias, accelerometer, z-axis, high word = ZA_BIAS_HIGH register */
#define     REG_ZA_BIAS_HIGH_47x            0x0056
#define     REG_ZA_BIAS_HIGH_49x            0x0226
#define     REG_ZA_BIAS_HIGH_50x            0x0056
#define     REG_ZA_BIAS_HIGH(val) ((val==1647) ? REG_ZA_BIAS_HIGH_47x : (val==1649) ? REG_ZA_BIAS_HIGH_49x : (val==1650) ? REG_ZA_BIAS_HIGH_50x : 0x0000)

/** User Scratch Registers
  * For 16495, USER_SCR_1, USER_SCR_2, USER_SCR_3, and USER_SCR_4 registers
  * For 16470/16500, USER_SCR_1, USER_SCR_2, and USER_SCR_3
 **/
#define     REG_USER_SCR_1_47x              0x0076
#define     REG_USER_SCR_2_47x              0x0078
#define     REG_USER_SCR_3_47x              0x007A
#define     REG_USER_SCR_1_49x              0x0274
#define     REG_USER_SCR_2_49x              0x0276
#define     REG_USER_SCR_3_49x              0x0278
#define     REG_USER_SCR_4_49x              0x027A
#define     REG_USER_SCR_1_50x              0x0076
#define     REG_USER_SCR_2_50x              0x0078
#define     REG_USER_SCR_3_50x              0x007A

/* Diagnostic, flash memory count */
#define     REG_FLSHCNT_LOW_47x             0x007C
#define     REG_FLSHCNT_HIGH_47x            0x007E
#define     REG_FLSHCNT_LOW_49x             0x027C
#define     REG_FLSHCNT_HIGH_49x            0x027E
#define     REG_FLSHCNT_LOW_50x             0x007C
#define     REG_FLSHCNT_HIGH_50x            0x007E

/* Global commands */
#define     REG_GLOB_CMD_47x                0x0068
#define     REG_GLOB_CMD_49x                0x0302
#define     REG_GLOB_CMD_50x                0x0068
#define     REG_GLOB_CMD(val) ((val==1647) ? REG_GLOB_CMD_47x : (val==1649) ? REG_GLOB_CMD_49x : (val==1650) ? REG_GLOB_CMD_50x : 0x0000)

/** Control, I/O pins, functional definitions
  * For 16495, FNCTIO_CTRL register
  * For 16470/16500, see MSC_CTRL register
 **/
#define     REG_FNCTIO_CTRL_49x             0x0306

/** Control, I/O pins, general-purpose
  * For 16495, GPIO_CTRL register
  * For 16470/16500, NONE
 **/
#define     REG_GPIO_CTRL_49x               0x0308

/** Control, clock, and miscellaneous correction
  * For 16495, CONFIG register
  * For 16470/16500, see MSC_CTRL register
 **/
#define     REG_CONFIG_49x                  0x030A

/* Control, decimation filter (output data rate) */
#define     REG_DEC_RATE_47x                0x0064
#define     REG_DEC_RATE_49x                0x030C
#define     REG_DEC_RATE_50x                0x0064
#define     REG_DEC_RATE(val) ((val==1647) ? REG_DEC_RATE_47x : (val==1649) ? REG_DEC_RATE_49x : (val==1650) ? REG_DEC_RATE_50x : 0x0000)

/** Control, bias estimation period
  * For 16470/16495, NULL_CNFG register
  * For 16500, NONE
 **/
#define     REG_NULL_CNFG_47x               0x0066
#define     REG_NULL_CNFG_49x               0x030E
#define     REG_NULL_CNFG(val) ((val==1647) ? REG_NULL_CNFG_47x : (val==1649) ? REG_NULL_CNFG_49x : 0x0000)


/** Control, input clock scaling (PPS mode)
  * For 16495, SYNC_SCALE register
  * For 16470/16500, see UP_SCALE
 **/
#define     REG_SYNC_SCALE_49x              0x0310

/** Measurement range (model specific) identifier
  * For 16495/16500, RANG_MDL register
  * For 16470, NONE
 **/
#define     REG_RANG_MDL_49x                0x0312
#define     REG_RANG_MDL_50x                0x005E
#define     REG_RANG_MDL(val) ((val==1649) ? REG_RANG_MDL_49x : (val==1650) ? REG_RANG_MDL_50x : 0x0000)


/** Filter selection
  * For 16495, FILTR_BNK_0 and FILTR_BNK_1 registers
  * For 16470/16500, NONE
 **/
#define     REG_FILTR_BNK_0_49x             0x0316
#define     REG_FILTR_BNK_1_49x             0x0318

/* Firmware Revision, Programming Date and Year */
#define     REG_FIRM_REV_47x                0x006C
#define     REG_FIRM_DM_47x                 0x006E
#define     REG_FIRM_Y_47x                  0x0070
#define     REG_FIRM_REV_49x                0x0378
#define     REG_FIRM_DM_49x                 0x037A
#define     REG_FIRM_Y_49x                  0x037C
#define     REG_FIRM_REV_50x                0x006C
#define     REG_FIRM_DM_50x                 0x006E
#define     REG_FIRM_Y_50x                  0x0070
#define     REG_FIRM_REV(val) ((val==1647) ? REG_FIRM_REV_47x : (val==1649) ? REG_FIRM_REV_49x : (val==1650) ? REG_FIRM_REV_50x : 0x0000)
#define     REG_FIRM_DM(val) ((val==1647) ? REG_FIRM_DM_47x : (val==1649) ? REG_FIRM_DM_49x : (val==1650) ? REG_FIRM_DM_50x : 0x0000)
#define     REG_FIRM_Y(val) ((val==1647) ? REG_FIRM_Y_47x : (val==1649) ? REG_FIRM_Y_49x : (val==1650) ? REG_FIRM_Y_50x : 0x0000)

/** Boot loader revision
  * For 16495, BOOT_REV register
  * For 16470/16500, NONE
 **/
#define     REG_BOOT_REV_49x                0x037E

/** Signature CRC and Real-time CRC
  * For 16495, CAL_SIGTR_* , CAL_DRVTN_*, CODE_SIGTR_*, CODE_DRVTN_* registers
  * For 16470/16500, NONE
 **/
#define     REG_CAL_SIGTR_LWR_49x           0x0404
#define     REG_CAL_SIGTR_UPR_49x           0x0406
#define     REG_CAL_DRVTN_LWR_49x           0x0408
#define     REG_CAL_DRVTN_UPR_49x           0x040A
#define     REG_CODE_SIGTR_LWR_49x          0x040C
#define     REG_CODE_SIGTR_UPR_49x          0x040E
#define     REG_CODE_DRVTN_LWR_49x          0x0410
#define     REG_CODE_DRVTN_UPR_49x          0x0412

/* Serial Number */
#define     REG_SERIAL_NUM_47x              0x0074
#define     REG_SERIAL_NUM_49x              0x0420
#define     REG_SERIAL_NUM_50x              0x0074
#define     REG_SERIAL_NUM(val) ((val==1647) ? REG_SERIAL_NUM_47x : (val==1649) ? REG_SERIAL_NUM_49x : (val==1650) ? REG_SERIAL_NUM_50x : 0x0000)


/** Control, input/output and other miscellaneous options
  * For 16495, see CONFIG or FNCTIO_CTRL
  * For 16470/16500, MSC_CTRL register
 **/
#define     REG_MSC_CTRL_47x                0x0060
#define     REG_MSC_CTRL_50x                0x0060
#define     REG_MSC_CTRL(val) ((val==1647) ? REG_MSC_CTRL_47x : (val==1650) ? REG_MSC_CTRL_50x : 0x0000)

/** Control, scale factor for input clock, pulse per second (PPS) mode
  * For 16495, see SYNC_SCALE register
  * For 16470/16500, UP_SCALE register
 **/
#define     REG_UP_SCALE_47x                0x0062
#define     REG_UP_SCALE_50x                0x0062

/* REG STALL TIMES */
/* For 16470 */
#define     IMU_STALL_US_NULLCFG_47x      (16)  // Assume minimum stall period
#define     IMU_STALL_US_DEC_RATE_47x     (16)  // Assume minimum stall period
#define     IMU_STALL_US_GLOB_CMD_47x     (193000)

/* For 16495 */
#define     IMU_STALL_US_FNCTIO_49x       (340)
#define     IMU_STALL_US_FILTBNK0_49x     (65)
#define     IMU_STALL_US_FILTBNK1_49x     (65)
#define     IMU_STALL_US_NULLCFG_49x      (71)
#define     IMU_STALL_US_SYNC_SCALE_49x   (340)
#define     IMU_STALL_US_DEC_RATE_49x     (340)
#define     IMU_STALL_US_GPIO_CTRL_49x    (45)
#define     IMU_STALL_US_CONFIG_49x       (45)
#define     IMU_STALL_US_GLOB_CMD_49x     (1120)

/* For 16500 */
#define     IMU_STALL_US_NULLCFG_50x      (16)  // Assume minimum stall period
#define     IMU_STALL_US_DEC_RATE_50x     (16)  // Assume minimum stall period
#define     IMU_STALL_US_GLOB_CMD_50x     (255000)

#define     IMU_STALL_US_DEC_RATE(val)  ((val==1647) ? IMU_STALL_US_DEC_RATE_47x : (val==1649) ? IMU_STALL_US_DEC_RATE_49x : (val==1650) ? IMU_STALL_US_DEC_RATE_50x : 0x0000)

/* BURST READ FRAME LENGTH   */
/* For 16470
  BRF Data Format (16-bit only) = 11 * 2 = 22 bytes
  */
#define     MAX_BRF16_LEN_BYTES_47x       (22)

/* For 16495
  BRF Data Format (32-bit only) (fSCLK > 3.6 MHz) = 20 * 2 = 40 bytes
 */
#define     MAX_BRF32_LEN_BYTES_49x       (40)

/* For 16500
  BRF Data Format = 11 * 2 = 22 bytes when BURST32 is 1 (i.e. 16-bit burst data)
  BRF Data Format = 17 * 2 = 34 bytes when BURST32 is 0 (i.e. 32-bit burst data)
  */
#define     MAX_BRF32_LEN_BYTES_50x       (34)
#define     MAX_BRF16_LEN_BYTES_50x       (22)

/* Page ID Min and Max */
#define     IMU_MIN_PAGE_ID_47x           (0)
#define     IMU_MAX_PAGE_ID_47x           (0)
#define     IMU_MIN_PAGE_ID_49x           (0)
#define     IMU_MAX_PAGE_ID_49x           (3)
#define     IMU_MIN_PAGE_ID_50x           (0)
#define     IMU_MAX_PAGE_ID_50x           (0)
#define     IMU_MIN_PAGE_ID(val) ((val==1647) ? IMU_MIN_PAGE_ID_47x : (val==1649) ? IMU_MIN_PAGE_ID_49x : (val==1650) ? IMU_MIN_PAGE_ID_50x : 0x0000)
#define     IMU_MAX_PAGE_ID(val) ((val==1647) ? IMU_MAX_PAGE_ID_47x : (val==1649) ? IMU_MAX_PAGE_ID_49x : (val==1650) ? IMU_MAX_PAGE_ID_50x : 0x0000)



/* REG_DIAG_STAT_47x */
#define     BITM_DIAG_STAT_CLK_ERR_47x               0x0080
#define     BITM_DIAG_STAT_MEM_FAIL_47x              0x0040
#define     BITM_DIAG_STAT_SENSOR_FAIL_47x           0x0020
#define     BITM_DIAG_STAT_STANDBY_MODE_47x          0x0010
#define     BITM_DIAG_STAT_SPI_COMM_47x              0x0008
#define     BITM_DIAG_STAT_FLSH_MEM_UPD_47x          0x0004
#define     BITM_DIAG_STAT_DATA_PATH_OVERRUN_47x     0x0002

/* REG_SYS_E_FLAG_49x */
#define     BITM_SYS_E_FLAG_WDG_TMR_49x             0x8000
#define     BITM_SYS_E_FLAG_SYNC_ERR_49x            0x0100
#define     BITM_SYS_E_FLAG_PROC_OVERRUN_49x        0x0080
#define     BITM_SYS_E_FLAG_FLSH_MEM_UPD_49x        0x0040
#define     BITM_SYS_E_FLAG_SENSOR_TEST_49x         0x0020
#define     BITM_SYS_E_FLAG_SPI_COMM_49x            0x0008
#define     BITM_SYS_E_FLAG_SRAM_CRC_49x            0x0004
#define     BITM_SYS_E_FLAG_BOOT_MEM_49x            0x0002

/* REG_DIAG_STS_49x */
#define     BITM_DIAG_STS_Z_ACCL_49x                0x0020
#define     BITM_DIAG_STS_Y_ACCL_49x                0x0010
#define     BITM_DIAG_STS_X_ACCL_49x                0x0008
#define     BITM_DIAG_STS_Z_GYRO_49x                0x0004
#define     BITM_DIAG_STS_Y_GYRO_49x                0x0002
#define     BITM_DIAG_STS_X_GYRO_49x                0x0001

/* REG_DIAG_STAT_50x */
#define     BITM_DIAG_STAT_ACCL_FAILURE_50x          0x0400
#define     BITM_DIAG_STAT_GYRO2_FAILURE_50x         0x0200
#define     BITM_DIAG_STAT_GYRO1_FAILURE_50x         0x0100
#define     BITM_DIAG_STAT_CLK_ERR_50x               0x0080
#define     BITM_DIAG_STAT_MEM_FAIL_50x              0x0040
#define     BITM_DIAG_STAT_SENSOR_FAIL_50x           0x0020
#define     BITM_DIAG_STAT_STANDBY_MODE_50x          0x0010
#define     BITM_DIAG_STAT_SPI_COMM_50x              0x0008
#define     BITM_DIAG_STAT_FLSH_MEM_UPD_50x          0x0004
#define     BITM_DIAG_STAT_DATA_PATH_OVERRUN_50x     0x0002

/* REG_GLOB_CMD_47x */
#define     BITP_GLOB_CMD_SOFT_RST_47x              (7)
#define     BITP_GLOB_CMD_FLASH_MEM_TEST_47x        (4)
#define     BITP_GLOB_CMD_FLASH_MEM_UPD_47x         (3)
#define     BITP_GLOB_CMD_SENSOR_SELF_TEST_47x      (2)
#define     BITP_GLOB_CMD_CALIB_RESTORE_47x         (1)
#define     BITP_GLOB_CMD_BIAS_CORR_UPD_47x         (0)
#define     BITM_GLOB_CMD_SOFT_RST_47x              ((0x1) << BITP_GLOB_CMD_SOFT_RST_47x)
#define     BITM_GLOB_CMD_FLASH_MEM_TEST_47x        ((0x1) << BITP_GLOB_CMD_FLASH_MEM_TEST_47x)
#define     BITM_GLOB_CMD_FLASH_MEM_UPD_47x         ((0x1) << BITP_GLOB_CMD_FLASH_MEM_UPD_47x)
#define     BITM_GLOB_CMD_SENSOR_SELF_TEST_47x      ((0x1) << BITP_GLOB_CMD_SENSOR_SELF_TEST_47x)
#define     BITM_GLOB_CMD_CALIB_RESTORE_47x         ((0x1) << BITP_GLOB_CMD_CALIB_RESTORE_47x)
#define     BITM_GLOB_CMD_BIAS_CORR_UPD_47x         ((0x1) << BITP_GLOB_CMD_BIAS_CORR_UPD_47x)

/* REG_GLOB_CMD_49x */
#define     BITP_GLOB_CMD_SOFT_RST_49x              (7)
#define     BITP_GLOB_CMD_CLR_USR_CALIB_49x         (6)
#define     BITP_GLOB_CMD_FLASH_MEM_UPD_49x         (3)
#define     BITP_GLOB_CMD_SELF_TEST_49x             (1)
#define     BITP_GLOB_CMD_BIAS_CORR_UPD_49x         (0)
#define     BITM_GLOB_CMD_SOFT_RST_49x              ((0x1) << BITP_GLOB_CMD_SOFT_RST_49x)
#define     BITM_GLOB_CMD_CLR_USR_CALIB_49x         ((0x1) << BITP_GLOB_CMD_CLR_USR_CALIB_49x)
#define     BITM_GLOB_CMD_FLASH_MEM_UPD_49x         ((0x1) << BITP_GLOB_CMD_FLASH_MEM_UPD_49x)
#define     BITM_GLOB_CMD_SELF_TEST_49x             ((0x1) << BITP_GLOB_CMD_SELF_TEST_49x)
#define     BITM_GLOB_CMD_BIAS_CORR_UPD_49x         ((0x1) << BITP_GLOB_CMD_BIAS_CORR_UPD_49x)

/* REG_GLOB_CMD_50x */
#define     BITP_GLOB_CMD_SOFT_RST_50x              (7)
#define     BITP_GLOB_CMD_FLASH_MEM_TEST_50x        (4)
#define     BITP_GLOB_CMD_FLASH_MEM_UPD_50x         (3)
#define     BITP_GLOB_CMD_SENSOR_SELF_TEST_50x      (2)
#define     BITP_GLOB_CMD_CALIB_RESTORE_50x         (1)
#define     BITM_GLOB_CMD_SOFT_RST_50x              ((0x1) << BITP_GLOB_CMD_SOFT_RST_50x)
#define     BITM_GLOB_CMD_FLASH_MEM_TEST_50x        ((0x1) << BITP_GLOB_CMD_FLASH_MEM_TEST_50x)
#define     BITM_GLOB_CMD_FLASH_MEM_UPD_50x         ((0x1) << BITP_GLOB_CMD_FLASH_MEM_UPD_50x)
#define     BITM_GLOB_CMD_SENSOR_SELF_TEST_50x      ((0x1) << BITP_GLOB_CMD_SENSOR_SELF_TEST_50x)
#define     BITM_GLOB_CMD_CALIB_RESTORE_50x         ((0x1) << BITP_GLOB_CMD_CALIB_RESTORE_50x)

/* REG_FNCTIO_CTRL_49x */
#define     BITP_FNCTIO_CTRL_SYNC_CLK_MODE_49x      (8)
#define     BITP_FNCTIO_CTRL_SYNC_CLK_EN_49x        (7)
#define     BITP_FNCTIO_CTRL_SYNC_CLK_POL_49x       (6)
#define     BITP_FNCTIO_CTRL_SYNC_CLK_DIO_49x       (4)
#define     BITP_FNCTIO_CTRL_DATA_RDY_EN_49x        (3)
#define     BITP_FNCTIO_CTRL_DATA_RDY_POL_49x       (2)
#define     BITP_FNCTIO_CTRL_DATA_RDY_DIO_49x       (0)
#define     BITM_FNCTIO_CTRL_SYNC_CLK_MODE_49x      ((0x1) << BITP_FNCTIO_CTRL_SYNC_CLK_MODE_49x)
#define     BITM_FNCTIO_CTRL_SYNC_CLK_EN_49x        ((0x1) << BITP_FNCTIO_CTRL_SYNC_CLK_EN_49x)
#define     BITM_FNCTIO_CTRL_SYNC_CLK_POL_49x       ((0x1) << BITP_FNCTIO_CTRL_SYNC_CLK_POL_49x)
#define     BITM_FNCTIO_CTRL_SYNC_CLK_DIO_49x       ((0x3) << BITP_FNCTIO_CTRL_SYNC_CLK_DIO_49x)
#define     BITM_FNCTIO_CTRL_DATA_RDY_EN_49x        ((0x1) << BITP_FNCTIO_CTRL_DATA_RDY_EN_49x)
#define     BITM_FNCTIO_CTRL_DATA_RDY_POL_49x       ((0x1) << BITP_FNCTIO_CTRL_DATA_RDY_POL_49x)
#define     BITM_FNCTIO_CTRL_DATA_RDY_DIO_49x       ((0x3) << BITP_FNCTIO_CTRL_DATA_RDY_DIO_49x)

/* REG_GPIO_CTRL_49x */
#define     BITP_GPIO_CTRL_DIO4_DATA_49x            (7)
#define     BITP_GPIO_CTRL_DIO3_DATA_49x            (6)
#define     BITP_GPIO_CTRL_DIO2_DATA_49x            (5)
#define     BITP_GPIO_CTRL_DIO1_DATA_49x            (4)
#define     BITP_GPIO_CTRL_DIO4_DIR_49x             (3)
#define     BITP_GPIO_CTRL_DIO3_DIR_49x             (2)
#define     BITP_GPIO_CTRL_DIO2_DIR_49x             (1)
#define     BITP_GPIO_CTRL_DIO1_DIR_49x             (0)
#define     BITM_GPIO_CTRL_DIO4_DATA_49x            ((0x1) << BITP_GPIO_CTRL_DIO4_DATA_49x)
#define     BITM_GPIO_CTRL_DIO3_DATA_49x            ((0x1) << BITP_GPIO_CTRL_DIO3_DATA_49x)
#define     BITM_GPIO_CTRL_DIO2_DATA_49x            ((0x1) << BITP_GPIO_CTRL_DIO2_DATA_49x)
#define     BITM_GPIO_CTRL_DIO1_DATA_49x            ((0x1) << BITP_GPIO_CTRL_DIO1_DATA_49x)
#define     BITM_GPIO_CTRL_DIO4_DIR_49x             ((0x1) << BITP_GPIO_CTRL_DIO4_DIR_49x)
#define     BITM_GPIO_CTRL_DIO3_DIR_49x             ((0x1) << BITP_GPIO_CTRL_DIO3_DIR_49x)
#define     BITM_GPIO_CTRL_DIO2_DIR_49x             ((0x1) << BITP_GPIO_CTRL_DIO2_DIR_49x)
#define     BITM_GPIO_CTRL_DIO1_DIR_49x             ((0x1) << BITP_GPIO_CTRL_DIO1_DIR_49x)

/* REG_CONFIG_49x */
#define     BITP_CONFIG_LIN_G_COMP_49x              (7)
#define     BITP_CONFIG_PNT_PERC_ALIGN_49x          (6)
#define     BITM_CONFIG_LIN_G_COMP_49x              ((0x1) << BITP_CONFIG_LIN_G_COMP_49x)
#define     BITM_CONFIG_PNT_PERC_ALIGN_49x          ((0x1) << BITP_CONFIG_PNT_PERC_ALIGN_49x)

/* NULL_CNFG_47x */
#define     BITP_NULL_CNFG_EN_ZA_47x                (13)
#define     BITP_NULL_CNFG_EN_YA_47x                (12)
#define     BITP_NULL_CNFG_EN_XA_47x                (11)
#define     BITP_NULL_CNFG_EN_ZG_47x                (10)
#define     BITP_NULL_CNFG_EN_YG_47x                (9)
#define     BITP_NULL_CNFG_EN_XG_47x                (8)
#define     BITM_NULL_CNFG_EN_ZA_47x                ((0x1) << BITP_NULL_CNFG_EN_ZA_47x)
#define     BITM_NULL_CNFG_EN_YA_47x                ((0x1) << BITP_NULL_CNFG_EN_YA_47x)
#define     BITM_NULL_CNFG_EN_XA_47x                ((0x1) << BITP_NULL_CNFG_EN_YA_47x)
#define     BITM_NULL_CNFG_EN_ZG_47x                ((0x1) << BITP_NULL_CNFG_EN_ZG_47x)
#define     BITM_NULL_CNFG_EN_YG_47x                ((0x1) << BITP_NULL_CNFG_EN_YG_47x)
#define     BITM_NULL_CNFG_EN_XG_47x                ((0x1) << BITP_NULL_CNFG_EN_XG_47x)


/* NULL_CNFG_49x */
#define     BITP_NULL_CNFG_EN_ZA_49x                (13)
#define     BITP_NULL_CNFG_EN_YA_49x                (12)
#define     BITP_NULL_CNFG_EN_XA_49x                (11)
#define     BITP_NULL_CNFG_EN_ZG_49x                (10)
#define     BITP_NULL_CNFG_EN_YG_49x                (9)
#define     BITP_NULL_CNFG_EN_XG_49x                (8)
#define     BITM_NULL_CNFG_EN_ZA_49x                ((0x1) << BITP_NULL_CNFG_EN_ZA_49x)
#define     BITM_NULL_CNFG_EN_YA_49x                ((0x1) << BITP_NULL_CNFG_EN_YA_49x)
#define     BITM_NULL_CNFG_EN_XA_49x                ((0x1) << BITP_NULL_CNFG_EN_YA_49x)
#define     BITM_NULL_CNFG_EN_ZG_49x                ((0x1) << BITP_NULL_CNFG_EN_ZG_49x)
#define     BITM_NULL_CNFG_EN_YG_49x                ((0x1) << BITP_NULL_CNFG_EN_YG_49x)
#define     BITM_NULL_CNFG_EN_XG_49x                ((0x1) << BITP_NULL_CNFG_EN_XG_49x)

/* MSC_CTRL_47x */
#define     BITP_MSC_CTRL_LIN_G_COMP_47x              (7)
#define     BITP_MSC_CTRL_PNT_PERC_ALIGN_47x          (6)
#define     BITP_MSC_CTRL_SYNC_FUNC_47x               (2)
#define     BITP_MSC_CTRL_SYNC_POL_47x                (1)
#define     BITP_MSC_CTRL_DR_POL_47x                  (0)
#define     BITM_MSC_CTRL_LIN_G_COMP_47x              ((0x01) << BITP_MSC_CTRL_LIN_G_COMP_47x)
#define     BITM_MSC_CTRL_PNT_PERC_ALIGN_47x          ((0x01) << BITP_MSC_CTRL_PNT_PERC_ALIGN_47x)
#define     BITM_MSC_CTRL_SYNC_FUNC_47x               ((0x07) << BITP_MSC_CTRL_SYNC_FUNC_47x)
#define     BITM_MSC_CTRL_SYNC_POL_47x                ((0x01) << BITP_MSC_CTRL_SYNC_POL_47x)
#define     BITM_MSC_CTRL_DR_POL_47x                  ((0x01) << BITP_MSC_CTRL_DR_POL_47x)

/* MSC_CTRL_50x*/
#define     BITP_MSC_CTRL_BURST32_50x                 (9)
#define     BITP_MSC_CTRL_BURST_SEL_50x               (8)
#define     BITP_MSC_CTRL_LIN_G_COMP_50x              (7)
#define     BITP_MSC_CTRL_PNT_PERC_ALIGN_50x          (6)
#define     BITP_MSC_CTRL_SENS_BW_50x                 (4)
#define     BITP_MSC_CTRL_SYNC_FUNC_50x               (2)
#define     BITP_MSC_CTRL_SYNC_POL_50x                (1)
#define     BITP_MSC_CTRL_DR_POL_50x                  (0)
#define     BITM_MSC_CTRL_BURST32_50x                 ((0x01) << BITP_MSC_CTRL_BURST32_50x)
#define     BITM_MSC_CTRL_BURST_SEL_50x               ((0x01) << BITP_MSC_CTRL_BURST_SEL_50x)
#define     BITM_MSC_CTRL_LIN_G_COMP_50x              ((0x01) << BITP_MSC_CTRL_LIN_G_COMP_50x)
#define     BITM_MSC_CTRL_PNT_PERC_ALIGN_50x          ((0x01) << BITP_MSC_CTRL_PNT_PERC_ALIGN_50x)
#define     BITM_MSC_CTRL_SENS_BW_50x                 ((0x01) << BITP_MSC_CTRL_SENS_BW_50x)
#define     BITM_MSC_CTRL_SYNC_FUNC_50x               ((0x03) << BITP_MSC_CTRL_SYNC_FUNC_50x)
#define     BITM_MSC_CTRL_SYNC_POL_50x                ((0x01) << BITP_MSC_CTRL_SYNC_POL_50x)
#define     BITM_MSC_CTRL_DR_POL_50x                  ((0x01) << BITP_MSC_CTRL_DR_POL_50x)


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
#define     IMU_RES_ACCL16_470        (0.00125)
#define     IMU_RES_ACCL16_475        (0.00025)
#define     IMU_RES_ACCL16_477        (0.00125)
#define     IMU_RES_GYRO16_47x1       (0.00625)
#define     IMU_RES_GYRO16_47x2       (0.025)
#define     IMU_RES_GYRO16_47x3       (0.1)
#define     IMU_RES_ACCL32_470        (0.00125/(1<<16))
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
#define     IMU_RES_ACCL16_500        (0.00125)
#define     IMU_RES_ACCL16_505        (0.00025)
#define     IMU_RES_ACCL16_507        (0.00125)
#define     IMU_RES_GYRO16_50x1       (0.00625)
#define     IMU_RES_GYRO16_50x2       (0.025)
#define     IMU_RES_GYRO16_50x3       (0.1)
#define     IMU_RES_ACCL32_500        (0.00125/(1<<16))
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

/* REG_RANG_MDL */
#define     IMU_RANG_MDL(id)            ((id == 0x3) ? "±125°/sec" : (id == 0x7) ? "±450°/sec" : (id == 0xF) ? "±2000°/sec" : "UNKNOWN")

/* REG_BOOT_REV */
#define     IMU_BOOT_REV_MAJOR(val)     ( ((val) >> 8 ) & 0xFF )
#define     IMU_BOOT_REV_MINOR(val)     ( (val) & 0xFF )

/* REG_FIRM */
#define     IMU_FIRM_REV_UPPER(val)     ( ((val) >> 8 ) & 0xFF )
#define     IMU_FIRM_REV_LOWER(val)     ( (val) & 0xFF )
#define     IMU_FIRM_DAY(val)           ( (val) & 0xFF )
#define     IMU_FIRM_MONTH(val)         ( ((val) >> 8 ) & 0xFF )

#endif
