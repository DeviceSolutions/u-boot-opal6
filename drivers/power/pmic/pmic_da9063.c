/*
 * (C) Copyright 2016
 * Device Solutions 
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <power/da9063.h>

#define DA9063_I2C_PORT 0



/*


*/
int pmic_read_one_byte(int port, unsigned char device_address, unsigned char device_register, unsigned char* pData )
{
	// int result = 0;

	return i2c_read(device_address, device_register, 1, pData, 1);

//	return result;
}


/*


*/
int pmic_write_one_byte(int port, unsigned char device_address, unsigned char device_register, unsigned char data )
{
//	int result = 0;

	return i2c_write(device_address, device_register, 1, &data, 1);

//	printf("pmic_write_one_byte: address: 0x%x, register 0x%x, result = %d\n", device_address, device_register, result );

//	return result;
}



int pmic_access_page( int port, unsigned char page )
{
	//int result = -1;

	return i2c_write(DA9063_I2C_ADDRESS, DA9063_PAGE_CON, 1, &page, 1);
/*
	for (;;)
	{
		if( pmic_write_one_byte( port, DA9063_I2C_ADDRESS, DA9063_PAGE_CON, page ) != 0)
			result = 0;
		break;
	}

	return result;
*/
}


int da9063_get_register( unsigned short reg, unsigned char* pData )
{
	int result = -1;
	unsigned char page = (unsigned char)(reg/0x80);
	int port = DA9063_I2C_PORT;

	for( ;; )
	{
		if( pmic_access_page( port, page) != 0 )
		{
			printf("da9063_read_register: failed to set page access\n");
			break;
		}
		
		if( pmic_read_one_byte( port, DA9063_I2C_ADDRESS, (unsigned char)reg, pData ) != 0)
		{
			printf("da9063_read_register: failed to read data\n");
			break;
		}	

		if( pmic_access_page( port, 0) != 0 )
		{
			printf("da9063_read_register: failed to reset page access\n");
			break;
		}

		result = 0;
		break;
	}

	return result;
}

int da9063_set_register( unsigned short reg, unsigned char data, unsigned char mask )
{
	int result = -1; // success
	unsigned char new_data;
	unsigned char page = (unsigned char)(reg/0x80);
	int port = DA9063_I2C_PORT;

	for( ;; )
	{
		if( pmic_access_page( port, page) != 0 )
		{
			printf("da9063_write_register: failed to set page access\n");
			break;
		}
		
		/* For partial writes:
		   1. Read the register
		   2. Insert the new bits
  		   3. Write back the new value
		*/ 	
		if( (mask & PMIC_ALL_BITS) ^ PMIC_ALL_BITS )
		{
			if( pmic_read_one_byte( port, DA9063_I2C_ADDRESS, (unsigned char)reg, &new_data ) != 0)
			{
				printf("da9063_write_register: failed to read register\n");
				break;
			}	
			
			new_data &= ~mask;
			new_data |= data & mask;

		}
		else
			new_data = data;

		if( pmic_write_one_byte( port, DA9063_I2C_ADDRESS, (unsigned char)reg, new_data ) != 0)
		{
			printf("da9063_write_register: failed to write data\n");
			break;
		}

		if( pmic_access_page( port, 0) != 0 )
		{
			printf("da9063_write_register: failed to reset page access\n");
			break;
		}

		result = 0;
		break;
	}

	return result;
}





int da9063_detect(void)
{
	int result = 0;
	int err_count = 0;
	int retry_count = 10;
	unsigned char value;

	for(;;)
	{
		if (!i2c_probe(0x58)) 
		{
			if (i2c_read(0x59, 0x81, 1, &value, 1)) 
			{
				printf("Read DA9063 device ID error!\n");
				result = -1;
			}

			if( -1 == result ) 
			{
				printf("Device found at 0x58: but device is unable to be read\n" );
				result = 0;	
				break;
			}
			else
			{
				if( 0x61 == value ) 
				{
					printf("Device DA9063 found! deviceid=%x\n", value );
					result = 1;
					break;
				}
				else 
				{
					printf("Device found at 0x58: (Not DA9063) found a deviceid=%x\n", value );
					break;
				}
			}
		}
		else 
		{
			err_count++;
			printf("No device at 0x58: DA9063 not found (trying again %d...)\n", err_count );
			udelay(1000000);
		}

		if( err_count >= retry_count ) 
		{
			printf("No device at 0x58: DA9063 not found!\n" );
			break;
		}
	}

	return result;
}
	




