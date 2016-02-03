/*
 * (C) Copyright 2016
 * Device Solutions
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */
#ifndef __POWER_DA90630_H__
#define __POWER_DA9063_H__

#define DA9063_I2C_ADDRESS            0x58
#define DA9063_I2C_SPEED              (100 * 1000)

#define PMIC_ALL_BITS               0xff

#define DA9063_DEVICE_ID            0x61

// DA9063 PMIC registers
#define DA9063_PAGE_CON             0x0
#define DA9063_GPIO2_3_ADDR         0x16
#define DA9063_GPIO4_5_ADDR         0x17
#define DA9063_GPIO6_7_ADDR         0x18
#define DA9063_GPIO10_11_ADDR       0x1a
#define DA9063_GPIO_MODE0_7_ADDR    0x1d
#define DA9063_GPIO_MODE8_15_ADDR   0x1e
#define DA9063_VLDO2_CONT_ADDR      0x27
#define DA9063_VLDO2_A_ADDR         0xaa
#define DA9063_VLDO2_B_ADDR         0xbb
#define DA9063_VLDO3_CONT_ADDR      0x28
#define DA9063_VLDO3_A_ADDR         0xab
#define DA9063_VLDO3_B_ADDR         0xbc
#define DA9063_VLDO4_CONT_ADDR      0x29
#define DA9063_VLDO4_A_ADDR         0xac
#define DA9063_VLDO4_B_ADDR         0xbd
#define DA9063_CONFIG_D_ADDR        0x109
#define DA9063_DEVICE_ID_ADDR       0x181
#define DA9063_VARIANT_ID_ADDR      0x182
#define DA9063_CUSTOMER_ID_ADDR     0x183
#define DA9063_CONFIG_ID_ADDR       0x184


extern int da9063_set_register( unsigned short reg, unsigned char data, unsigned char mask );
extern int da9063_get_register( unsigned short reg, unsigned char* pData );
extern int da9063_detect( void );

#endif	/* __POWER_DA9063_H__ */
