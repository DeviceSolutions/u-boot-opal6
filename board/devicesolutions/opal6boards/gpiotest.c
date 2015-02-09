/*
* Copyright (C) 2015
* Martin Welford, Device Solutions
*
* SPDX-License-Identifier:	GPL-2.0+
*/

#include <common.h>
#include <command.h>
#include <errno.h>
#include <asm/gpio.h>

static void flash_output(int gpio)
{
	int i = 0;
	for (i = 0; i < 2; i++) {
		gpio_set_value(gpio, 1); 
		mdelay(250);
		gpio_set_value(gpio, 0);
		mdelay(250);
	}
}

static void check_input(char* input_string, int gpio, int active_high)
{
	int value;
	int pass = 0;

	puts("Activate ");
	puts(input_string);
	puts(": ");

	/* wait for input */
	while (1) {
		value = gpio_get_value(gpio);
		if (value == active_high) {
			pass = 1;
			break;
		}
		mdelay(50);

		/* fall out if there is a character in the buffer */
		if (tstc()) {
			getc(); /* dump the character so it doesn't get in to the command line */
			break;
		}
	}

	if (pass) 
		puts("OK\n");
	else
		puts("FAIL\n");
}

static int do_gpiotest(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	/* Flash Outputs */
	flash_output(32); /* Output 1 */
	flash_output(33); /* Output 2 */

	/* Flash LEDs */
	flash_output(69); /* G1 */
	flash_output(65); /* G2 */
	flash_output(68); /* R1 */
	flash_output(78); /* R2 */
	
	/* Check Buttons and Inputs */
	check_input("Button 1", 95, 0);
	check_input("Button 2", 94, 0);
	check_input("Button 3", 109, 0);
	check_input("Input 1", 34, 1);
	check_input("Input 2", 35, 1);


	/* Check Buttons */
	return 0;
}

U_BOOT_CMD(gpiotest, 1, 0, do_gpiotest,
	"Flashes LEDs and Outputs & tests Inputs and Buttons",
	"");

