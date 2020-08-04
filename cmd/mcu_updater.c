// SPDX-License-Identifier: GPL-2.0+

#include <common.h>
#include <command.h>
#include <console.h>

#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/gcr.h>
#include <asm/arch/gpio.h>
#include <i2c.h>
#include <dm/pinctrl.h>
#include <dm/uclass.h>
#include <dm/device.h>

#define _DEBUG 0
#define CONFIG_MCU_VERBOSE

#if _DEBUG
#define debug_print(...) printf(__VA_ARGS__)
#define CONFIG_MCU_VERBOSE
#else
#define debug_print(...)
#endif

// show message if verbose
#ifdef CONFIG_MCU_VERBOSE
// ------- TestMsgCore  -------
#define COLOR_BLACK		0
#define COLOR_RED		1
#define COLOR_GREEN		2
#define COLOR_YELLOW	3
#define COLOR_BLUE		4
#define COLOR_MAGENTA	5
#define COLOR_CYAN		6
#define COLOR_WHITE		7
#define COLOR_DEFAULT	9
#define COLOR_FOREGROUND 30
#define COLOR_BACKGROUND 40

#define MCU_LOG_VERBOSE(...) printf(__VA_ARGS__)
void _set_color(u8 foreColor, u8 backColor)
{
	char color_code[16];

	snprintf(color_code, sizeof(color_code) - 1, "%c[%u;%um", 0x1B,
		 foreColor + COLOR_FOREGROUND, backColor + COLOR_BACKGROUND);
	printf(color_code);
}

void _log_color(u8 foreColor, u8 backColor, char *fmt, ...)
{
	va_list ap;

	_set_color(foreColor, backColor);
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	_set_color(COLOR_DEFAULT, COLOR_DEFAULT);
}

#define MCU_LOG_PASS(...)    \
    _log_color(COLOR_GREEN, COLOR_DEFAULT, __VA_ARGS__)
#define MCU_LOG_ERROR(...)   \
    _log_color(COLOR_RED, COLOR_DEFAULT, __VA_ARGS__)
#define MCU_LOG_ERROR_V(s1, s2) MCU_LOG_ERROR(s1)
#define MCU_LOG_WARNING(...) \
    _log_color(COLOR_MAGENTA, COLOR_DEFAULT, __VA_ARGS__)

void MCU_LOG_HEADER(char *message)
{
	_set_color(COLOR_CYAN, COLOR_DEFAULT);
	printf("***************************************************\n");
	printf(message);
	printf("\n***************************************************\n");
	_set_color(COLOR_DEFAULT, COLOR_DEFAULT);
}
#else // remove all logs, just keep error message
#define MCU_LOG_VERBOSE(...)
#define MCU_LOG_PASS(...)
#define MCU_LOG_ERROR(...) printf(__VA_ARGS__)
#define MCU_LOG_ERROR_V(s1, s2) printf(s2)
#define MCU_LOG_WARNING(...)
void MCU_LOG_HEADER(char *message) {}
#endif

// some information must output
#define MCU_LOG_INFO(...) printf(__VA_ARGS__)
#define MCU_LOG_FAILURE(format)	\
	(MCU_LOG_ERROR("\n***** FAILURE ***** at:\n%s(%d)(%s):\n",	\
	__FILE__, __LINE__, __func__),	\
	(MCU_LOG_ERROR format), printf("\n"))

// Lock and Fuse Bits
#define FUSE_LOW_BYTE		0
#define FUSE_HIGH_BYTE		1
#define FUSE_EXTENDED_BYTE	2
#define LOCK_BYTE			3

#define HIGH true
#define LOW false
#define DELAY 20
#define FW_MAX_SIZE 16 * 1024
#define FW_MIN_SIZE  1 * 1024
#define FW_SIGN_OFFSET 0x70
#if CONFIG_DM_I2C
#define I2C_BUS  13
#define I2C_ADDR 0x70
#define I2C_FW_INFO_OFFSET 0x100 // Implementation Guide 2.5.1
#define I2C_FW_INFO_SIZE 0x40
#endif
static const char FW_SIGN[] = "Nuvoton_RunBMC";

// Set default Program SPI pins
// Don't forget to modify MFSEL in spi_init and spi_close
static u8 SPI_CLK = 222;
static u8 SPI_MOSI = 223;
static u8 SPI_MISO = 38;
static u8 SPI_RST = 37;
#define PINCTRL_DEV_NAME "pinctrl@f0800000"
#define PIN_CONFIG_PERSIST_STATE (PIN_CONFIG_END + 1)
static struct udevice *pinctrl_dev;
static int SMB_GROUP = -1;
static const unsigned char smb_pins[] = {114, 115, 116, 117, 118, 119,
	31, 30, 29, 28, 27, 26, 171, 172, 173, 174, 128, 129, 130, 131,
	132, 133, 134, 135, 220, 221, 222, 223, 23, 22, 21, 20};

struct mcu_binary_info {
	char header[16];
	char date[16];
	char time[16];
	u8 minor_ver;
	u8 majoy_ver;
	u8 i2c_start_addr;
	u8 uart_tx;
	u8 reserved[12];
};

static u8 read_spi_miso(void)
{
	//read MISO value from GPIO register GPnDIN.
	unsigned int m = SPI_MISO / 32;
	u32 bit = SPI_MISO - m * 32;
	struct npcm750_gpio_regs *miso_addr = (struct npcm750_gpio_regs *)
		(npcm750_get_base_gpio() + m * 0x1000);
	u32 miso_val = readl(&miso_addr->gpn_din);

	return ((miso_val >> bit) & 1);
}

static inline int set_spi(u32 pin_num, bool data,
		const struct pinctrl_ops *ops)
{
	return  ops->pinconf_set(pinctrl_dev, pin_num,
				PIN_CONFIG_OUTPUT, data ? 1 : 0);
}

static int smb_pin_to_group_selector(unsigned int selector)
{
	// get smb pin group name first
	int i, mod = -1, ret;
	char group[8] = {0};

	for (i = 0; i < sizeof(smb_pins); i++) {
		if (selector == smb_pins[i]) {
			mod = i;
			break;
		}
	}
	if (mod == -1) {
		MCU_LOG_ERROR("Invalid pin selector for get group ID, %d\n", selector);
		return -EINVAL;
	}
	mod = mod >> 1;
	ret = scnprintf(group, sizeof(group), "smb%d", mod);
	if (ret < 3) {
		MCU_LOG_ERROR("Internal error in get group name, %d\n", mod);
		return -EINVAL;
	}

	// pinctrl_group_name_to_selector
	const struct pinctrl_ops *ops = pinctrl_get_ops(pinctrl_dev);
	unsigned int ngroups, g_selector;

	if (!ops->get_groups_count || !ops->get_group_name) {
		MCU_LOG_ERROR("> PINCTRL get_groups_count or get_group_name missing\n");
		return -EINVAL;
	}

	ngroups = ops->get_groups_count(pinctrl_dev);

	/* See if this pctldev has this group */
	for (g_selector = 0; g_selector < ngroups; g_selector++) {
		const char *gname = ops->get_group_name(pinctrl_dev, g_selector);

		if (!strcmp(group, gname))
			return g_selector;
	}

	return -EINVAL;
}

//-------------------------------------------------------------------------------------------------------
/*
  ATtiny1634 External Serial Programming:

  * SCK    (input, PC1)  <-->  GPIO222/SMB13SCL (output; Open-Drain to avoid I2C bus contention)
  * MOSI   (input, PB1)  <-->  GPIO223/SMB13SDA (output; Open-Drain to avoid I2C bus contention)
  * MISO   (output, PB2) <-->  GPIO38  (input)
  * RESET# (input, PC3)  <-->  GPIO37  (output; Push-Pull)

  * Enter after RESET# and CLK are held LOW.
  * CKSEL fuses must keep default to internal clock of 8MHz. In addition, CPU clock divider is by 8 so CPU clock is 1MHz.
  * SPI CLK max freq (at first-time when CPU clock is 1MHz): CPU / 4 = 250KHz;

  * data input is clocked on the rising edge of SPI CLK.
  * data output is clocked on the falling edge of SPI CLK.

*/
static int spi_init(void)
{
	MCU_LOG_VERBOSE("> Init SPI I/Os ...\n");
	const struct pinctrl_ops *ops = pinctrl_get_ops(pinctrl_dev);
	int err = 0;

	// For some device enable PERSIST_STATE, clear it first
	MCU_LOG_WARNING("> Warning: persist state would be cleared"
		" while MCU update ...\n");
	err += ops->pinconf_set(pinctrl_dev, SPI_CLK, PIN_CONFIG_PERSIST_STATE, 0);
	err += ops->pinconf_set(pinctrl_dev, SPI_MOSI, PIN_CONFIG_PERSIST_STATE, 0);
	err += ops->pinconf_set(pinctrl_dev, SPI_MISO, PIN_CONFIG_PERSIST_STATE, 0);
	err += ops->pinconf_set(pinctrl_dev, SPI_RST, PIN_CONFIG_PERSIST_STATE, 0);
	if (err) {
		MCU_LOG_ERROR("> Clear persist state ... fail, %d\n", err);
		return CMD_RET_FAILURE;
	}

	// Init I/Os Mux
	// Set GPIO222/SMB13SCL and GPIO223/SMB13SDA to GPIOs.
	// No need set MFSEL, pinctrl driver will switch pin to GPIO
	// In NPCM750R there is no MUX on GPIO37 and GPIO38.

	// Output type set to push-pull for RESET#
	err += ops->pinconf_set(pinctrl_dev, SPI_RST, PIN_CONFIG_DRIVE_PUSH_PULL, 0);
	err += ops->pinconf_set(pinctrl_dev, SPI_CLK, PIN_CONFIG_DRIVE_OPEN_DRAIN, 0);
	err += ops->pinconf_set(pinctrl_dev, SPI_MOSI, PIN_CONFIG_DRIVE_OPEN_DRAIN, 0);

	// Enable input for SPI_MISO
	err += ops->pinconf_set(pinctrl_dev, SPI_MISO, PIN_CONFIG_INPUT_ENABLE, 0);

	// Enable output for SPI_MOSI, SPI_CLK and SPI_RST
	err += ops->pinconf_set(pinctrl_dev, SPI_RST, PIN_CONFIG_OUTPUT_ENABLE, 0);
	err += ops->pinconf_set(pinctrl_dev, SPI_CLK, PIN_CONFIG_OUTPUT_ENABLE, 0);
	err += ops->pinconf_set(pinctrl_dev, SPI_MOSI, PIN_CONFIG_OUTPUT_ENABLE, 0);

	// Disable output for SPI_MISO
	// pinctrl driver no need disable output enable, set OEC instead

	// Enable Debounce for SPI_MISO
	err += ops->pinconf_set(pinctrl_dev, SPI_MISO, PIN_CONFIG_INPUT_DEBOUNCE, 0);

	// Note: We assume all other GPIO settings are at default value.
	// (e.g., Lock, Polarity, Output Slew-Rate, Output Drive Strength, Output Blink, Pull-Down, Pull-Up, Main Power)

	if (err) {
		MCU_LOG_ERROR("> Set pin configurations ... fail, %d\n", err);
		return CMD_RET_FAILURE;
	}

	// keep SCL and SDA low
	err += set_spi(SPI_CLK, LOW, ops);
	udelay(1);
	err += set_spi(SPI_MOSI, LOW, ops);

	// Enter External Serial Programming (entered by reset falling sample SCL low)
	MCU_LOG_VERBOSE("> Enter External Serial Programming ...\n");
	err += set_spi(SPI_CLK, LOW, ops);
	err += set_spi(SPI_RST, HIGH, ops);
	mdelay(10); // 700 nsec
	err += set_spi(SPI_RST, LOW, ops);
	mdelay(20); // 20 msec

	if (err) {
		MCU_LOG_ERROR("set spi pins value fail, %d\n", err);
		return CMD_RET_FAILURE;
	}
	return CMD_RET_SUCCESS;
}

void spi_close(void)
{
	MCU_LOG_VERBOSE("> Close SPI I/Os ...\n");
	const struct pinctrl_ops *ops = pinctrl_get_ops(pinctrl_dev);

	// Init I/Os Mux
	// Set GPIO222/SMB13SCL and GPIO223/SMB13SDA to SMB13.
	ops->pinmux_group_set(pinctrl_dev, SMB_GROUP, SMB_GROUP);
	// In NPCM750R there is no MUX on GPIO37 and GPIO38.

	// SCL should gos high now.

	// exit External Serial Programming
	MCU_LOG_VERBOSE("> Exit External Serial Programming ...\n");
	set_spi(SPI_RST, HIGH, ops);
}

//-----------------------------------------------------------------------
// spi_send_receive_byte :
// ATtiny1634 sample data on the rising edge. (i.e., BMC must output data on the prev falling edge)
// ATtiny1634 output data on the falling edge. (i.e., BMC may sample data on the next rising edge or just before next falling edge)
// In this setup, clock start and end at low state.
// start output from pShiftOut MSB, start input to pShiftIn MSB.
// on first test, 150KHz look stable freq. Need to check if it's related to Open-Drain I/O.
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
u8 spi_send_receive_byte(u8 dataToSend)
{
	int l_BitCount, delay;
	u8 l_MOSI, l_MISO;
	u8 dataReceived = 0;
	const struct pinctrl_ops *ops = pinctrl_get_ops(pinctrl_dev);

	for (l_BitCount = 0; l_BitCount < 8; l_BitCount++) {
		// new SPI_MOSI
		l_MOSI = (dataToSend >> (7 - l_BitCount)) & 0x01;
		if (l_MOSI == 0)
			set_spi(SPI_MOSI, LOW, ops);
		else
			set_spi(SPI_MOSI, HIGH, ops);

		// read SPI_MIDO
		l_MISO = read_spi_miso();
		dataReceived |= l_MISO << (7 - l_BitCount);

		for (delay = 0; delay < DELAY; delay++)
			l_MISO = read_spi_miso();

		set_spi(SPI_CLK, HIGH, ops);

		// NOTE:
		// The same signals are use for I2C Channel 13 therefore SDA/MOSI
		// must not be change when SCL/CLK signal is high to avoid I2C
		// start/stop condition.

		for (delay = 0; delay < DELAY; delay++)
			l_MISO = read_spi_miso();

		set_spi(SPI_CLK, LOW, ops);
	}

	return dataReceived;
}

bool mcu_is_sync(void)
{
	u8 sync;

	spi_send_receive_byte(0xAC);
	spi_send_receive_byte(0x53);
	sync = spi_send_receive_byte(0x00);
	spi_send_receive_byte(0x00);

	if (sync == 0x53)
		return (true);
	else
		return (false);
}

bool mcu_is_ready(void)
{
	u8 data;

	spi_send_receive_byte(0xF0);
	spi_send_receive_byte(0x00);
	spi_send_receive_byte(0x00);
	data = spi_send_receive_byte(0x00);
	if ((data & 0x01) == 0x01)
		return false; // Device is busy programming
	else
		return true; // Device is ready for new command
}

void mcu_chip_erase(void)
{
	spi_send_receive_byte(0xAC);
	spi_send_receive_byte(0x80);
	spi_send_receive_byte(0x00);
	spi_send_receive_byte(0x00);
	// Need to wait for RDY !!
}

u8 read_rom_table(u8 addr)
{
	u8 data;

	if ((addr & 0x01) == 0x00) {
		// Read Signature Byte a.k.a. ROM Table, Low byte.
		spi_send_receive_byte(0x30);
	} else {
		// Read Calibration Byte a.k.a. ROM Table, High byte.
		spi_send_receive_byte(0x38);
	}

	spi_send_receive_byte(0x00);
	spi_send_receive_byte(addr >> 1);
	data = spi_send_receive_byte(0x00);
	return data;
}

bool mcu_check_device_signature(void)
{
	u32 device_signature = 0;

	device_signature |= (u32)read_rom_table(0) << 0;
	device_signature |= (u32)read_rom_table(2) << 8;
	device_signature |= (u32)read_rom_table(4) << 16;
	if (device_signature == 0x0012941E) // Check ATtiny1634 DS section 22.3.1
		return true;
	else
		return false;
}

//-----------------------------------------------------------------------------------------
// Lock and Fuse Bits  API
//-----------------------------------------------------------------------------------------
u8 read_configuration(int mode)
{
	u8 data;

	assert(mode >= FUSE_LOW_BYTE && mode <= LOCK_BYTE);
	switch (mode) {
	case (FUSE_LOW_BYTE):
		// Read Fuse low bits
		spi_send_receive_byte(0x50);
		spi_send_receive_byte(0x00);
		break;

	case (FUSE_HIGH_BYTE):
		// Read Fuse High bits
		spi_send_receive_byte(0x58);
		spi_send_receive_byte(0x08);
		break;

	case (FUSE_EXTENDED_BYTE):
		// Read Fuse Extended Bits
		spi_send_receive_byte(0x50);
		spi_send_receive_byte(0x08);
		break;

	case (LOCK_BYTE):
		// Read Lock bits
		spi_send_receive_byte(0x58);
		spi_send_receive_byte(0x00);
		break;
	}
	spi_send_receive_byte(0x00);
	data = spi_send_receive_byte(0x00);

	return data;
}

//-----------------------------------------------------------------------------------------
// Flash API
//-----------------------------------------------------------------------------------------

u8 read_flash_memory_byte(u16 addr)
{
	u8 data;

	if ((addr & 0x01) == 0x00) {
		// Read Program Memory, Low byte
		spi_send_receive_byte(0x20);
	} else {
		// Read Program Memory, High byte
		spi_send_receive_byte(0x28);
	}

	spi_send_receive_byte(addr >> 9);
	spi_send_receive_byte(addr >> 1);
	data = spi_send_receive_byte(0x00);
	return data;
}

//-------------------------------------------------------------------------------------
// it took ~ 3.7 sec @ 150KHz to dump 16KB ;
//-----------------------------------------------------------------------------------------
void read_flash_memory_block(u16 addr, u16 LengthInByte, u8 *p_data)
{
	while (LengthInByte != 0) {
		*p_data = read_flash_memory_byte(addr);
		addr++;
		p_data++;
		LengthInByte--;
	}
}

//-----------------------------------------------------------------------------------------
// write_flash_memory_page:
// The Flash array is programmed one page at a time. Page size is 16 words / 32 bytes
// tWD_FLASH  = 4.5 msec;
// Need to issue chip erase before programming.
//-----------------------------------------------------------------------------------------
int write_flash_memory_page(u16 addr, u8 *pData)
{
	// address must be aline to word
	if ((addr & 0x01) != 0) {
		MCU_LOG_FAILURE(("Addr is not aline to word"));
		return CMD_RET_FAILURE;
	}

	addr &= 0xFFFE;

	// Load Flash Memory Page
	for (int index = 0; index < 16; index++) {
		// Load Program Memory Page, Low byte of word
		spi_send_receive_byte(0x40);
		spi_send_receive_byte(0x00);
		spi_send_receive_byte(index);
		spi_send_receive_byte(*pData);
		pData++;

		// Load Program Memory Page, High byte of word
		spi_send_receive_byte(0x48);
		spi_send_receive_byte(0x00);
		spi_send_receive_byte(index);
		spi_send_receive_byte(*pData);
		pData++;
	}

	// Write Program Memory Page
	spi_send_receive_byte(0x4C);
	spi_send_receive_byte(addr >> 9);
	spi_send_receive_byte(addr >> 1);
	spi_send_receive_byte(0x00);
	while (mcu_is_ready() == false)
		; // TBD: add Time-Out
	return 0;
}

//-------------------------------------------------------------------------------------
// it took ~ 6 sec @ 150KHz to program 16KB ;
// ~2.3 sec the actual program, ~ 3.7 sec is the loading to internal page.
//-------------------------------------------------------------------------------------
int write_flash_memory_block(u16 addr, u16 LengthInByte, u8 *pData)
{
	// address must be aline to word
	if ((addr & 0x01) != 0) {
		MCU_LOG_FAILURE(("Addr is not aline to word"));
		return CMD_RET_FAILURE;
	}

	// LengthInByte must be aline to Page size 16 words / 32 bytes
	if ((LengthInByte & 0x1F) != 0) {
		MCU_LOG_FAILURE(("LengthInByte is not aline to page"));
		return CMD_RET_FAILURE;
	}

	addr &= 0xFFFE;
	LengthInByte &= 0xFFE0;
	int err = CMD_RET_SUCCESS;

	while (LengthInByte != 0) {
		err = write_flash_memory_page(addr, pData);
		if (err)
			return err;
		addr += 32;
		pData += 32;
		LengthInByte -= 32;
	}
	return err;
}

#if _DEBUG
// Utility
void memory_dump_byte_compare(u32 address_reference, u32 address,
						 u32 display_address, u16 NumOfLines)
{
	u16 line;
	u8 *p_data8 = (u8 *)address;
	u8 *p_data8_reference = (u8 *)address_reference;
	u8 index;
	u8 data8, data8_reference;
	char msg[16];

	for (line = 0; line < NumOfLines; line++) {
		MCU_LOG_VERBOSE("0x%08X: ", display_address);
		display_address += 16;
		for (index = 0; index < 16; index++) {
			data8 = *p_data8++;
			data8_reference = *p_data8_reference++;
			snprintf(msg, 15, "0x%02X  ", data8);
			if (data8 != data8_reference)
				MCU_LOG_ERROR(msg);
			else
				MCU_LOG_PASS(msg);
		}
		MCU_LOG_VERBOSE("\n");
	}
}
#endif

int check_spi_sync(void)
{
	spi_init();
	MCU_LOG_INFO("> Check Sync ... ");
	if (mcu_is_sync() == true) {
		MCU_LOG_PASS(" OK\n");
	} else {
		MCU_LOG_ERROR(" Failed\n");
		return CMD_RET_FAILURE;
	}
	return CMD_RET_SUCCESS;
}

static int do_mcu_init(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	MCU_LOG_HEADER("MCU firmware programer initialize");
	int err = 0;
	err = check_spi_sync();
	if (err)
		return err;
	MCU_LOG_VERBOSE("> Check Device Signature ... ");
	if (mcu_check_device_signature() == true) {
		MCU_LOG_PASS(" OK\n");
	} else {
		MCU_LOG_ERROR_V(" Failed\n", "Check Device Signature failed\n");
		return CMD_RET_FAILURE;
	}

	// check fuse and lock bytes
	u8 data = read_configuration(LOCK_BYTE);
	MCU_LOG_VERBOSE("> Lock bits (0x%02X): ", data);
	switch (data & 0x03) {
	case (0):
		MCU_LOG_WARNING("Further reading and programming of Flash, EEPROM and Fuses is disabled in parallel and serial programming mode.\n");
		return 0; //  can't read and program -- nothing to do here.

	case (2):
		MCU_LOG_WARNING("Further programming of Flash, EEPROM and Fuses is disabled in parallel and serial programming mode. \n");
		break; // can only dump

	case (3):
		MCU_LOG_PASS("No memory lock features enabled. \n");
		break;

	default:
		MCU_LOG_ERROR_V(" Reserved \n", "Lock bits: Reserved\n");
		return CMD_RET_FAILURE;
	}

	data = read_configuration(FUSE_LOW_BYTE);
	MCU_LOG_VERBOSE("> FUSE_LOW_BYTE (0x%02X):\n", data);

	MCU_LOG_VERBOSE("\t* Clock Source (CKSEL[3:0]): ");
	if ((data & 0xF) == 2)
		MCU_LOG_PASS("0x2 (Internal 8MHz)\n");
	else
		MCU_LOG_WARNING("0x%01x \n", data & 0xF);

	MCU_LOG_VERBOSE("\t* Clock Prescaler (CLKPS[3:0]): ");
	if ((data & 0x80) == 0)
		MCU_LOG_PASS("8 (Clock Source / 8) after reset.\n");
	else
		MCU_LOG_WARNING("1 (Clock Source / 1) after reset\n");

	if ((data & 0x40) == 0)
		MCU_LOG_WARNING("\t* CKOUT=0; PC2 is used as CKOUT.\n");
	else
		MCU_LOG_PASS("\t* CKOUT=1; CKOUT is disabled.\n");

	data = read_configuration(FUSE_HIGH_BYTE);
	MCU_LOG_VERBOSE("> FUSE_HIGH_BYTE (0x%02X):\n", data);

	MCU_LOG_VERBOSE("\t* Brown-Out Detection: ");
	switch (data & 0x7) {
	case 7:
	case 6:
		MCU_LOG_WARNING("1.8V\n");
		break;

	case 5:
		MCU_LOG_PASS(" 2.7V\n");

	case 4:
		MCU_LOG_ERROR_V(" 4.3V\n", "Brown-Out Detection: 4.3V");
	default:
		MCU_LOG_ERROR_V(" Reserved\n", "Brown-Out Detection: Reserved");
	}

	if ((data & 0x8) == 0)
		MCU_LOG_WARNING("\t* EESAVE=0; Preserves EEPROM memory during Chip Erase operation.\n");
	else
		MCU_LOG_PASS("\t* EESAVE=1; EEPROM memory erased during Chip Erase operation.\n");

	if ((data & 0x10) == 0)
		MCU_LOG_WARNING("\t* WDTON=0; watchdog enable by default and permanently on.\n");
	else
		MCU_LOG_PASS("\t* WDTON=1; watchdog timer is not enable by default.\n");

	if ((data & 0x20) == 0)
		MCU_LOG_PASS("\t* SPIEN=0; serial programming is enabled.\n");
	else
		MCU_LOG_ERROR("\t* SPIEN=1; serial programming is disabled.\n");

	if ((data & 0x40) == 0)
		MCU_LOG_ERROR("\t* DWEN=0; PC3 pin is debugWIRE by default, serial programming will not work.\n");
	else
		MCU_LOG_PASS("\t* DWEN=1; PC3 pin is RESET# by default.\n");

	if ((data & 0x80) == 0)
		MCU_LOG_ERROR("\t* RSTDISBL=0; external reset pin is disabled.\n");
	else
		MCU_LOG_PASS("\t* RSTDISBL=1; external reset pin is enabled by default.\n");

	data = read_configuration(FUSE_EXTENDED_BYTE);
	MCU_LOG_VERBOSE("> FUSE_EXTENDED_BYTE 0x%02X:\n", data);

	if ((data & 0x01) == 0)
		MCU_LOG_PASS("\t* SELFPRGEN=0; CPU self-programming is enabled.\n");
	else
		MCU_LOG_WARNING("\t* SELFPRGEN=1; CPU self-programming is disabled.\n");

	if ((data & 0x06) == 0x06)
		MCU_LOG_PASS("\t* BODACT[1:0]=11; Brown-Out Detection is disabled when device is in active or idle modes.\n");
	else
		MCU_LOG_WARNING("\t* BODACT[1:0]=xx; Brown-Out Detection may enabled when device is in active or idle modes.\n");

	if ((data & 0x18) == 0x18)
		MCU_LOG_PASS("\t* BODPD[1:0]=11; Brown-Out Detection is disabled when device is in sleep mode.\n");
	else
		MCU_LOG_WARNING("\t* BODPD[1:0]=xx; Brown-Out Detection may enabled when device is in sleep mode.\n");
#ifdef CONFIG_MCU_VERBOSE
	MCU_LOG_VERBOSE("\n");
#else
	MCU_LOG_INFO("\nmcu init success.\n");
#endif
	return CMD_RET_SUCCESS;
}

int get_address_size(char *const argv[], u32 *addr, u32 *size)
{
	*addr = simple_strtoul(argv[1], NULL, 16);
	*size = simple_strtoul(argv[2], NULL, 16);
	if (*size > FW_MAX_SIZE || *size < FW_MIN_SIZE) {
		MCU_LOG_ERROR("FW size is not correct, size:%04lX\n", *size);
		return CMD_RET_FAILURE;
	}
	return CMD_RET_SUCCESS;
}

// confirm the address contains valid MCU firmware
static int check_fw_simple(u32 address)
{
#if _DEBUG
	// dump FW signature
	for (int i = 0; i < sizeof(FW_SIGN); i++) {
		MCU_LOG_VERBOSE("%02X %02X, ",
			((u8 *)address + FW_SIGN_OFFSET)[i], FW_SIGN[i]);
	}
	MCU_LOG_VERBOSE("\n");
#endif
	return memcmp(FW_SIGN, (void *)(address + FW_SIGN_OFFSET), sizeof(FW_SIGN));
}

static int mcu_flash(u32 addr, u32 size)
{
	// we have 16K memory for MCU firmware
	u8 flash_data_0[FW_MAX_SIZE] __aligned(4);
	u8 flash_data_1[FW_MAX_SIZE] __aligned(4);
	int err = 0, index;

	MCU_LOG_INFO("> Check MCU firmware valid ...");
	if (check_fw_simple(addr) != 0) {
		MCU_LOG_ERROR(" Failed\n");
		return CMD_RET_FAILURE;
	}
	MCU_LOG_PASS(" OK\n");

	err = check_spi_sync();
	if (err)
		return err;
	MCU_LOG_HEADER("Micro-Controller ATtiny1634 Serial Programming");

	for (index = 0; index < FW_MAX_SIZE; index++) {
		flash_data_0[index] = 0x00; // data dump
		flash_data_1[index] = 0xFF; // data to program
	}

	// TODO: maybe add firmware size check and basic content?
	// copy memeory
	debug_print("> flash data 0:%p, data 1:%p\n", flash_data_0, flash_data_1);
	memcpy(flash_data_1, (void *)addr, size);
	u16 flash_offset = 0;
	u32 bytes_received = ((size + 31) / 32) * 32;
	MCU_LOG_VERBOSE("> Chip Erase (EEPROM, Flash and Lock its; Fuse bits are not affected) ");
	mcu_chip_erase();
	while (mcu_is_ready() == false)
		MCU_LOG_VERBOSE(".");
	MCU_LOG_PASS(" Done\n");

	MCU_LOG_INFO("> Program Flash Memory (address: 0x%X, size: %u) ...\n", flash_offset, bytes_received);
	//MemoryDumpB ((UINT32)Flash_Data_1, flash_offset, sizeof(Flash_Data_1)/16);
	write_flash_memory_block(flash_offset, bytes_received, flash_data_1);

	MCU_LOG_INFO("> Dump and compare Flash Memory ...\n");
	read_flash_memory_block(flash_offset, bytes_received, flash_data_0);

	if (memcmp(flash_data_1, flash_data_0, bytes_received) != 0) {
#if _DEBUG
		memory_dump_byte_compare((u32)flash_data_1, (u32)flash_data_0, flash_offset, bytes_received);
#endif
		MCU_LOG_ERROR("Verify programed firmware failed.\n");
		return CMD_RET_FAILURE;
	}
	MCU_LOG_INFO("MCU firmware update success.\n");
	return CMD_RET_SUCCESS;
}

static int check_pins(u8 clk, u8 mosi, u8 miso, u8 rst)
{
	int pins_count, i;
	u8 pins[4] = {clk, mosi, miso, rst};
	int groups[2];
	const struct pinctrl_ops *ops = pinctrl_get_ops(pinctrl_dev);
	char *ptr_gname, group_name[16], pin_name[24] = {'\0'}, *ptr_name = NULL;

	// pin number boundary check
	pins_count = pinctrl_get_pins_count(pinctrl_dev);
	for (i = 0; i < 4; i++) {
		if (pins[i] >= pins_count) {
			MCU_LOG_ERROR("pin number %u: %u is not valid\n", i, pins[i]);
			return CMD_RET_FAILURE;
		}
	}
	//smb function check
	for (i = 0; i < 2; i++) {
		groups[i] = smb_pin_to_group_selector(pins[i]);
		if (groups[i] < 0) {
			MCU_LOG_ERROR("Cannot get pin group, %u\n");
			return CMD_RET_FAILURE;
		}
		ptr_gname = ops->get_group_name(pinctrl_dev, groups[i]);
		snprintf(group_name, sizeof(group_name), ptr_gname);
		pinctrl_get_pin_name(pinctrl_dev, pins[i], pin_name, sizeof(pin_name));
		MCU_LOG_VERBOSE("pin %u: %s, group: %u.%s\n",
			pins[i], pin_name, groups[i], group_name);
	}

	// pin function check
	// clk and mosi should be SMB clk and sda
	if (groups[0] != groups[1] || pins[0] == pins[2] ||
		pins[0] == pins[3] || pins[0] == pins[1] ||
		pins[2] == pins[3]) {
		MCU_LOG_ERROR("Pin setting is not correct.\n");
		return CMD_RET_FAILURE;
	}
	pinctrl_get_pin_name(pinctrl_dev, pins[0], pin_name, sizeof(pin_name));
	ptr_name = strstr(pin_name, "SMB");
	if (ptr_name == NULL) {
		MCU_LOG_ERROR("CLK and MOSI must have SMB function\n");
		return CMD_RET_FAILURE;
	}
	SMB_GROUP = groups[0];
	return CMD_RET_SUCCESS;
}

static int do_mcu_erase(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	MCU_LOG_HEADER("MCU firmware erase");
	if (check_spi_sync())
		return CMD_RET_FAILURE;
	MCU_LOG_VERBOSE("> Chip Erase (EEPROM, Flash and Lock bits; Fuse bits "
			   "are not affected)\n");
	mcu_chip_erase();
	while (mcu_is_ready() == false)
		MCU_LOG_VERBOSE(".");
	MCU_LOG_PASS(" Done\n\n");
	return CMD_RET_SUCCESS;
}

static int do_mcu_close_spi(cmd_tbl_t *cmdtp, int flag, int argc,
							char *const argv[])
{
	MCU_LOG_HEADER("MCU firmware programer SPI stop");
	spi_close();
	MCU_LOG_VERBOSE("\n");
	return CMD_RET_SUCCESS;
}

static int do_mcu_flash(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	u32 addr, size;

	if (argc < 3)
		return CMD_RET_USAGE;
	int err = get_address_size(argv, &addr, &size);

	if (err)
		return err;
	return mcu_flash(addr, size);
}

static int do_mcu_info(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
#if CONFIG_DM_I2C
	u8 data[I2C_FW_INFO_SIZE];
	struct udevice *dev;
	struct mcu_binary_info *info = (struct mcu_binary_info *)data;
	int err;

	memset(data, 0x0, I2C_FW_INFO_SIZE); // clear data
	spi_close();
	mdelay(250); // 250 msec, keep long enough time to back i2c mode
	MCU_LOG_VERBOSE("> MCU information dump ...\n");
	// If there is no I2C device at the address, MCU firmware may not
	// flashed yet, or not enable I2C EERPOM feature. And make sure I2C
	// bus and address setting is correct.
	err = i2c_get_chip_for_busnum(I2C_BUS, I2C_ADDR, 2, &dev);
	if (err) {
		MCU_LOG_ERROR("Cannot get I2C device, bus:%u, address:0x%X\n",
			I2C_BUS, I2C_ADDR);
		return CMD_RET_FAILURE;
	}
	err = dm_i2c_read(dev, I2C_FW_INFO_OFFSET, data, I2C_FW_INFO_SIZE);
	if (err) {
		MCU_LOG_ERROR("Cannot read I2C device, offset:0x%X, length:0x%X\n",
			I2C_FW_INFO_OFFSET, I2C_FW_INFO_SIZE);
		return CMD_RET_FAILURE;
	}

	MCU_LOG_INFO("\nHeader : %s\n", info->header);
	MCU_LOG_INFO("Date   : %s\n", info->date);
	MCU_LOG_INFO("Time   : %s\n", info->time);
	MCU_LOG_INFO("Version: v%u.%u\n", info->majoy_ver, info->minor_ver);
#else
	MCU_LOG_INFO("I2C module must enable\n");
#endif
	return CMD_RET_SUCCESS;
}

static int do_mcu_update(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	if (argc < 3)
		return CMD_RET_USAGE;
	u32 addr, size;
	int rc;

	rc = get_address_size(argv, &addr, &size);
	if (rc)
		return rc;
	rc = do_mcu_init(cmdtp, flag, argc, argv);
	spi_close();
	if (rc)
		return rc;
	rc = mcu_flash(addr, size);
	if (rc == 0)
		do_mcu_info(cmdtp, flag, argc, argv);
	else
		spi_close();

	return rc;
}

static int do_mcu_getpins(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	MCU_LOG_INFO("MCU SPI CLK:%u, MOSI:%u, MISO:%u, RST:%u\n",
		SPI_CLK, SPI_MOSI, SPI_MISO, SPI_RST);
	return CMD_RET_SUCCESS;
}

static int do_mcu_setpins(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	if (argc < 2) {
		MCU_LOG_INFO("At least set one pin\n");
		return CMD_RET_USAGE;
	}

	u8 clk = SPI_CLK, mosi = SPI_MOSI, miso = SPI_MISO, rst = SPI_RST;
	clk = simple_strtoul(argv[1], NULL, 10);
	if (argc > 2)
		mosi = simple_strtoul(argv[2], NULL, 10);
	if (argc > 3)
		miso = simple_strtoul(argv[3], NULL, 10);
	if (argc > 4)
		rst = simple_strtoul(argv[4], NULL, 10);

	// perform pin check, CLK and MOSI should have multi function I2C
	int ret = check_pins(clk, mosi, miso, rst);
	if (ret)
		return ret;

	// set pins to if pins is well
	SPI_CLK = clk;
	SPI_MOSI = mosi;
	SPI_MISO = miso;
	SPI_RST = rst;
	do_mcu_getpins(cmdtp, flag, argc, argv);
	return CMD_RET_SUCCESS;
}

static cmd_tbl_t cmd_mcu_sub[] = {
	U_BOOT_CMD_MKENT(init, 1, 0, do_mcu_init, "", ""),
	U_BOOT_CMD_MKENT(flash, 3, 0, do_mcu_flash, "", ""),
	U_BOOT_CMD_MKENT(update, 3, 0, do_mcu_update, "", ""),
	U_BOOT_CMD_MKENT(erase, 1, 0, do_mcu_erase, "", ""),
	U_BOOT_CMD_MKENT(close, 1, 0, do_mcu_close_spi, "", ""),
	U_BOOT_CMD_MKENT(info, 1, 0, do_mcu_info, "", ""),
	U_BOOT_CMD_MKENT(getpins, 1, 0, do_mcu_getpins, "", ""),
	U_BOOT_CMD_MKENT(setpins, 5, 0, do_mcu_setpins, "", "")
	};

static int do_mcu(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	cmd_tbl_t *c;
	int ret;

	if (argc < 2)
		return CMD_RET_USAGE;

	/* Strip off leading 'mcu' command argument */
	argc--;
	argv++;
	c = find_cmd_tbl(argv[0], &cmd_mcu_sub[0], ARRAY_SIZE(cmd_mcu_sub));

	// get pinctrl device first
	if (!pinctrl_dev) {
		ret = uclass_get_device_by_name(UCLASS_PINCTRL, PINCTRL_DEV_NAME, &pinctrl_dev);
		if (ret) {
			MCU_LOG_ERROR("Cannot get pinctrl device!\n");
			return CMD_RET_FAILURE;
		}
		MCU_LOG_VERBOSE("> pinctrl device: " PINCTRL_DEV_NAME "\n");
	}
	// set SMB function
	SMB_GROUP = smb_pin_to_group_selector(SPI_CLK);
	if (SMB_GROUP < 0) {
		MCU_LOG_ERROR("Cannot get SMB group\n");
		return CMD_RET_FAILURE;
	}

	if (c && (argc <= c->maxargs))
		return c->cmd(cmdtp, flag, argc, argv);
	else
		return CMD_RET_USAGE;
}

U_BOOT_CMD(
	mcu, 6, 0, do_mcu,
	"Nuvoton RunBMC MCU firmware update utility",
#ifdef CONFIG_MCU_VERBOSE
	"[init/erase/flash/close/info] address size.\n"
	"  init   : make MCU get into firmware upload mode\n"
	"  erase  : erase MCU firmware\n"
	"  flash  : load [address] data and flash [size] data to MCU\n"
	"  update : run init and flash continuously\n"
	"  close  : Exit SPI program mode, turn pin function back to I2C\n"
	"  info   : dump mcu information read via I2C\n"
	"  setpins: set MCU SPI pin definitions\n"
	"           [SPI CLK] [SPI MOSI] [SPI MISO] [SPI RST]\n"
	"  getpins: get current MCU SPI pin definitions"
#else
	"update [address] [size]:\n"
	"  user must load firmware to memory first, then mcu update command\n"
	"  load firmware from [address] and [size]\n"
	"info:\n"
	"  dump mcu information\n"
	"setpins [SPI CLK] [SPI MOSI] [SPI MISO] [SPI RST]:\n"
	"  set MCU SPI pin definitions\n"
	"getpins:\n"
	"  get current MCU SPI pin definitions"
#endif
);
