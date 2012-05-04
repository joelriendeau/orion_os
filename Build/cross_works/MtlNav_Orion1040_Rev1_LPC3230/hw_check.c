#include <libmem.h>
#include <libmem_loader.h>
#include <targets/LPC3230.h>

#include <string.h>

#define TEST_DDR 0
#define TEST_NOR 0

extern unsigned __IRAM_segment_used_end__, __IRAM_segment_end__;

void wait(unsigned int periph_clock, unsigned int ms)
{
    // Power timer 0
    TIMCLK_CTRL1 = 0x4;

    // Reset counter and disable it
    T0TCR = 0x2;
    T0TCR = 0x0;

    // Clear match interrupt
    T0IR = 0x1;

    // Count mode positive clock edge
    T0CTCR = 0x0;

    // No prescaler
    T0PR = 0;

    // Generate match after x ms
    T0MR0 = periph_clock / 1000 * ms;

    // Interrupt on match reg 0
    T0MCR = 0x1;

    // Enable the counter
    T0TCR = 0x1;

    // Wait for the interrupt flag (polling instead of int handling)
    while ((T0IR & 0x1) == 0);

    // Disable the timer
    T0TCR = 0x0;

    // Disable power to timer
    TIMCLK_CTRL1 = 0x0;
}

static void walking0bitsetup(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		*base = ~(1 << i);
		base++;
	}
}
static int walking0bitcheck(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		if (*base != ~(1 << i))
		{
			return 0;
		}

		base++;
	}

	return 1;
}
static void walking1bitsetup(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		*base = (1 << i);
		base++;
	}
}
static int walking1bitcheck(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		if (*base != (1 << i))
		{
			return 0;
		}

		base++;
	}

	return 1;
}
static void invaddrsetup(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		*base = ~((unsigned int) base);
		base++;
	}
}
static int invaddrcheck(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		if (*base != ~((unsigned int) base))
		{
			return 0;
		}

		base++;
	}

	return 1;
}
static void noninvaddrsetup(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		*base = ((unsigned int) base);
		base++;
	}
}
static int noninvaddrcheck(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		if (*base != ((unsigned int) base))
		{
			return 0;
		}

		base++;
	}

	return 1;
}
static void aa55setup(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		*base = 0x55aa55aa;
		base++;
	}
}
static int aa55check(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		if (*base != 0x55aa55aa)
		{
			return 0;
		}

		base++;
	}

	return 1;
}
static void _55aasetup(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		*base = 0x55aa55aa;
		base++;
	}
}
static int _55aacheck(unsigned int *base)
{
	int i;
	for (i = 0; i < 32; i++)
	{
		if (*base != 0x55aa55aa)
		{
			return 0;
		}

		base++;
	}

	return 1;
}

typedef void (*pfrvi)(unsigned int *);
typedef int (*pfrii)(unsigned int *);
struct _memtests
{
	pfrvi testsetup;
	pfrii testcheck;
};

static struct _memtests testvecs[] = {
	{walking0bitsetup, walking0bitcheck},
	{walking1bitsetup, walking1bitcheck},
	{invaddrsetup, invaddrcheck},
	{noninvaddrsetup, noninvaddrcheck},
	{aa55setup, aa55check},
	{_55aasetup, _55aacheck},
	{NULL, NULL},
};

int ddr_memtst(unsigned int seed)
{
    int testnum;
    unsigned int start = 0x80000000;
	unsigned int inc, *base = (unsigned int *) start;
    unsigned int size = (32 * 1024 * 1024);

	// Offset test areas so we don't accidently get the results
	// from a previous test
	base += (seed * 0x4000) + (seed * 4);
	inc = size / sizeof(unsigned int);
	inc = inc / 256; // 256 test sections over test range

    // The DDR test is performed on a number of sections. Sections are
    // small areas of DDR memory separated by untested areas. The
    // sections tested are spread out over the entire range of the
    // device. Testing the entire DDR would take a long time, so this
    // is a good alternative.
	while ((unsigned int) base < ((start + size) - (32 * sizeof(unsigned int))))
	{
		// Loop through each test
		testnum = 0;
		while (testvecs[testnum].testsetup != NULL)
		{
			testvecs[testnum].testsetup(base);
			if (testvecs[testnum].testcheck(base) == 0)
			{
				// Test failed
				return 0;
			}

			testnum++;
		}

		base += inc;
	}

	// Test passed
	return 1;
}

typedef struct {
	unsigned int dqs_pass[32];
} DDRTEST_ST;
DDRTEST_ST ddrst;

static const unsigned char dqs2calsen[32] =
{
	7, 5, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0
};

static void dqsin_ddr_mod(unsigned int ddl)
{
	unsigned int tmp;

	/* Adjust calibration sensitivity with DQS delay */
	tmp = SDRAMCLK_CTRL & ~((7 << 10) | (0x1F << 2));
	SDRAMCLK_CTRL = tmp | (ddl << 2) | (dqs2calsen[ddl] << 10);
}

int find_ddr_dqsin_delay()
{
	unsigned int tmp, dqsindly, dqsstart = 0xFF, dqsend = 0xFF;
	int ppass = 0, pass = 0;

	/* Most devices fail DDR in the DQSIN range of 2 or less to about
	   20+ or more, so the test will start with a DQSIN delay value of
	   1 up to 30 (max - 1) */
	dqsindly = 1;

	/* At this point, DDR is initialized and opeational with the
	   exception of the DQSIN delay value and calibrarion sensitivity.
	   While adjusting the DQSIN delay between a range of values,
	   perform spot checks on uncached DDR memory to determine if DDR
	   is working with specific DQSIN delay values. By mapping out a
	   range of working values, we can determine the optimal DQSIN
	   delay value and calibration sensitivity. */
	while (dqsindly < 31)
	{
		/* Modify the DQSIN delay and appropriate calibration sensitivity
		   before running the test */
		dqsin_ddr_mod(dqsindly);

		/* Perform some memory write-read checks of uncached DDR memory
		   to determine if the values seem to work */
		if (ddr_memtst(dqsindly) == 1)
		{
			ddrst.dqs_pass[dqsindly] = 1;

			/* Test passed */
			if (dqsstart == 0xFF)
			{
				dqsstart = dqsindly;
			}

			ppass = 1;
		}
		else
		{
			ddrst.dqs_pass[dqsindly] = 0;

			/* Test failed */
			if (ppass == 1)
			{
				dqsend = dqsindly - 1;
				pass = 1;
				ppass = 0;
			}
		}

		/* Try next value */
		dqsindly++;
	}

	/* If the test passed, the we can use the average of the min and 
	   max values to get an optimal DQSIN delay */
	if (pass == 1)
	{
		dqsindly = (dqsstart + dqsend) / 2;
	}
	else
	{
		/* A working value couldn't be found, just pick something safe
		   so the system doesn't become unstable */
		dqsindly = 0xF;
	}

	/* Set calibration sensitivity based on nominal delay */
	dqsin_ddr_mod(dqsindly);

	return pass;
}

void simple_test_ddr()
{
    unsigned int a, b;
    unsigned short c, d;

    //find_ddr_dqsin_delay();

    //int res = ddr_memtst(1);

    //       d   c
    a = 0xffffffff;
    c = 0x4321;
    while (1)
    {
        *(volatile unsigned int*)(0x80000000) = a;
        b = *(volatile unsigned int*)(0x80000000);
        int res = ddr_memtst(1);
    }
}

int main()
{
    // JTAG devices on board :
    // ARM core (GENERIC ARM9EJ-S)  0x17900F0F 4
    // ARM Embedded trace buffer    0x1B900F0F 6
    // CPLD                         0x16D8A093 2

    /* Clock constraints to take into account
    Since we have no RTC, SYS_CLK will be set to OSC_RATE
    HCLK PLL outputs in the range 26 to 266 MHz
    HCLK may not be higher than 133MHz
    ARM_CLOCK = tentatively set to 200 MHz using the HCLKPLL
    HCLK is ARM_CLOCK divided by an integer
    DDR_CLOCK must be twice HCLK

    Max DDR clock is 133MHz
    Thus, HCLK must be 66MHz max because of DDR_CLOCK*/

    // In all cases, setup the desired clocks
    unsigned int osc_rate = 12500000; // oscillator installed is at 12,5 MHz
    unsigned int sys_clock;
    unsigned int arm_clock;
    unsigned int periph_clock;
    unsigned int h_clock;
    unsigned int emc_clock;
    unsigned int ddr_clock;
    volatile unsigned short tmp;

    unsigned char buf1[] = "Hello";
    unsigned char buf2[5];
    memcpy(buf2, buf1, 5);

    PWR_CTRL        = 0x00000000;
    HCLKPLL_CTRL    = 0x00000000;
    SYSCLK_CTRL     = 0x00000140; // 0x50 in bad phase setting
    HCLKPLL_CTRL    = 0x0001401E; // M = 15, Bypass Post Divider, PLL operating
    while ((HCLKPLL_CTRL & 0x1) == 0); // wait for lock
    HCLKDIV_CTRL    = 0x000000BD; // HCLK = ARM Clock / 2, PERIPH Clock is ARM Clock / 16, DDRAM Clk = ARM Clock
    PWR_CTRL        = 0x00000004; // From Direct RUN to Normal RUN Mode
    
    sys_clock = osc_rate;
    arm_clock = (((HCLKPLL_CTRL & 0x1FE) >> 1) + 1) * sys_clock; // in RUN mode, ARM clock taken from pll output // 200 MHz
    if ((HCLKDIV_CTRL & 0x7C) == 0)
        periph_clock = arm_clock;
    else
        periph_clock = arm_clock / ( ((HCLKDIV_CTRL & 0x7C) >> 2) + 1); // 12.5 MHz
    h_clock = arm_clock / ( (HCLKDIV_CTRL & 0x3) * 2);
    ddr_clock = arm_clock / ((HCLKDIV_CTRL & 0x180) >> 7);
    emc_clock = h_clock;

    EMCAHBControl0 = 0x00000001; // Enable - AHB bus for DMA
    EMCAHBControl3 = 0x00000001; // Enable - AHB bus for Instructions
    EMCAHBControl4 = 0x00000001; // Enable - AHB bus for Data
    EMCAHBTimeOut0 = 0x00000064; // Timeout for DMA
    EMCAHBTimeOut3 = 0x00000190; // Timeout for Instructions
    EMCAHBTimeOut4 = 0x00000190; // Timeout for Data

#if TEST_DDR
    SDRAMCLK_CTRL = 0x00080000; // reset
    SDRAMCLK_CTRL = 0x0003DE3E; // DDR is used, sens = 7, CMD_DELAY = 15, DQSIN_Delay = 15, use calibrated delays

    SDRAMCLK_CTRL = 0x0003DF3E; // start a calibration
    SDRAMCLK_CTRL = 0x0003DE3E; // stop calibration
    wait(periph_clock, 1);
    DDR_LAP_NOM = DDR_LAP_COUNT; // Save the measured value as the nominal one
    
    EMCControl  = 0x00000001; // enable EMC
    EMCConfig   = 0x00000000; // little endian
    
    EMCDynamicConfig0   = 0x00001886; // 512Mb, 32Mx16, low power ddr sdram over 16-bit bus
    
    EMCDynamicRasCas0   = 0x00000302; // RAS = 2, CAS = 3
    EMCDynamicReadConfig    = 0x00000111; // command delayed by CMD_DELAY, DDR data captured on negative edge, SDR data captured on pos edge

    // Setup Percharge command delay
    // This is the number of cycles minimum needed between a PRECHARGE command and a subsequent READ access
    // tRP = 18ns - 55,555,555 in Hz
    EMCDynamictRP  = 3; // (4 + (emc_clock / 55555555)) & 0xF;

    // Setup Active to Precharge command period
    // tRAS = 42ns - 23,809,523 in Hz
    EMCDynamictRAS = 5; // (4 + (emc_clock / 23809523)) & 0xF;

    // Setup Self-refresh exit time
    // No tSREX or tXSNR in spec sheet, use tXSR instead. tXSR = 120ns - 8,333,333 in Hz
    EMCDynamictSREX = 0xD; // (4 + (emc_clock / 8333333)) & 0x7F;

    // Setup Recovery time
    // tWR = 15ns - 66,666,666 in Hz
    EMCDynamictWR = 0x2; // (4 + (emc_clock / 66666666)) & 0xF;

    // Setup Active To Active command period
    // tRC = 60ns - 16,666,666 in Hz
    EMCDynamictRC = 0x7; // (4 + (emc_clock / 16666666)) & 0x1F;

    // Setup Auto-refresh period
    // tRFC = 97.5ns - 10,256,410 in Hz
    EMCDynamictRFC = 0x8; // (4 + (emc_clock / 10256410)) & 0x1F;

    // Setup Exit self-refresh
    // tXSR = 120ns - 8,333,333 in Hz
    EMCDynamictXSR = 0xD; // (4 + (emc_clock / 8333333)) & 0xFF;

    // Setup Active bank A to Active bank B delay
    // tRRD = 12ns - 83,333,333 in Hz
    EMCDynamictRRD = 0x2; // (4 + (emc_clock / 83333333)) & 0xF;

    // Setup Load mode register to Active command time
    // tMRD = 2 tCK
    EMCDynamictMRD = 0x00000001;

    // Setup Memory last data in to Read command time
    // tCDLR not mentionned in spec. use a default value...
    EMCDynamictCDLR = 0x00000002;
    
    EMCDynamicControl = 0x00000183; // Clk + ClkEn always on, NOP
    wait(periph_clock, 1);
    EMCDynamicControl = 0x00000103; // Clk + ClkEn always on, PRECHARGE ALL
    EMCDynamicRefresh = 0x00000002; // 32 clocks between refreshes
    wait(periph_clock, 1);
    // Optimal refresh interval (1 / tREFI)
    // 1 / tREFI = 1 / 7.8us = 128205
    EMCDynamicRefresh = 0x00000030; // 32 clocks between refreshes
    EMCDynamicControl = 0x00000083; // Clk + ClkEn always on, MODE REGISTER LOAD
    // Load the standard mode register : DDR reads this value from its address bus
    // burst length = 2, CAS = 3
    tmp = *(volatile unsigned short*)(0x80000000 + 0x31);
    EMCDynamicControl = 0x00000083; // Clk + ClkEn always on, MODE REGISTER LOAD
    // Reset the extended mode register : DDR reads this value from its address bus
    tmp = *(volatile unsigned short*)(0x80000000 + (0x1 << 14));
    EMCDynamicControl = 0x00000000; // Clk + ClkEn auto controlled, NORMAL

    // recalibrate
    SDRAMCLK_CTRL = 0x0003DD3E;
    SDRAMCLK_CTRL = 0x0003DC3E;
    wait(periph_clock, 1);

    int res = find_ddr_dqsin_delay();
    SDRAMCLK_CTRL = 0x0003DE3E;
    res = find_ddr_dqsin_delay();
#endif

#if TEST_NOR
    EMCStaticConfig0 = 0x00000081;

    uint8_t *flash1_start = (uint8_t *)0xE0000000;
    size_t flash1_size;
    const int flash1_max_geometry_regions = 30;
    libmem_geometry_t flash1_geometry[flash1_max_geometry_regions];
    libmem_flash_info_t flash1_info;
    libmem_driver_handle_t flash1_handle;
    int res2;

    // Detect the type, size and geometry of the Intel FLASH.
    /*
    res2 = libmem_cfi_get_info(flash1_start,
        &flash1_size,
        flash1_geometry,
        flash1_max_geometry_regions,
        &flash1_info);
    */
        
    /* should be :
    flash1_geometry[] = // NUMONIX 64 Mbit, top config, JS28F640P30T85
    {
        63, 128*1024, // 0x3f, 0x20000
        4,  32*1024,  // 0x4,  0x8000
        x,  0
    };
    flash1_info.pairing = 0;
    flash1_info.width = 2;
    */

    // Register the FLASH LIBMEM driver
    res2 = libmem_register_cfi_driver(&flash1_handle,
                                      flash1_start,
                                      flash1_geometry,
                                      flash1_max_geometry_regions,
                                      &flash1_info);
    if (res2 != LIBMEM_STATUS_SUCCESS)
        return 0;

    int* start, *end;
    start = &__IRAM_segment_used_end__;
    end =  &__IRAM_segment_end__;

    if (res2 == LIBMEM_STATUS_SUCCESS)
    {
        libmem_rpc_loader_start(&__IRAM_segment_used_end__, &__IRAM_segment_end__ - 1);
    }

    libmem_rpc_loader_exit(res2, NULL);

    /*
     uint8_t *write_dest = flash1_start + 16; 

    uint8_t read_data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    
    read_data[0] = write_dest[0];
    read_data[1] = write_dest[1];
    read_data[2] = write_dest[2];
    read_data[3] = write_dest[3];
    read_data[4] = write_dest[4];
    read_data[5] = write_dest[5];
    read_data[6] = write_dest[6];
    read_data[7] = write_dest[7];

    
    const uint8_t write_data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };

    // Unlock the destination memory area.
    res = libmem_unlock(write_dest, sizeof(write_data));
    if (res != LIBMEM_STATUS_SUCCESS)
      return 0;

    // Erase the destination memory area.
    res = libmem_erase(write_dest, sizeof(write_data), 0, 0);
    if (res != LIBMEM_STATUS_SUCCESS)
      return 0;

    // Copy write_data to the destination memory area.
    res = libmem_write(write_dest, write_data, sizeof(write_data));
    if (res != LIBMEM_STATUS_SUCCESS)
      return 0;

    // Complete any outstanding transactions and put FLASH memory back into read mode.
    res = libmem_flush();
    if (res != LIBMEM_STATUS_SUCCESS)
      return 0;
    */
#endif

    unsigned* a = (unsigned *)0x22000000;
    *a = 10;
    *a = 10;
    *a = 10;
    *a = 10;
    *a = 10;
    *a = 10;

    while (1)
    {
    }

    return 1;
}