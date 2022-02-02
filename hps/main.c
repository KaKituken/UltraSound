/*
 *      Author: Daniel Pelikan
 *      Copyright 2016. All rights reserved
*/

    //some documentaton

	/* By default the full size of the DDR-RAM is assigned to the linux kernel.
	 * With an option in the u-boot command line a limit can be set how much memeory should be assigned to the kernel
	 * This can be done with the option mem=xxxx
	 *
	 * To be able to enter the u-boot console connect the serial cable to the serial port before booting
	 * 115200 is the baudrate
	 *
	 *
	 * https://www.altera.com/support/support-resources/knowledge-base/solutions/rd06132014_165.html
	 * setenv bootargs console=ttyS0,115200 mem=1000M
	 * saveenv
	 *
	 * It seesm that mmcboot is booted by default
	 *
	 * The default line in u-boot looks like
	 * mmcboot=setenv bootargs console=ttyS0,115200 root=${mmcroot} rw rootwait;bootz ${loadaddr} - ${fdtaddr}
	 *
	 * which needs to be changed to:
	 * mmcboot=setenv bootargs console=ttyS0,115200 mem=800M root=${mmcroot} rw rootwait;bootz ${loadaddr} - ${fdtaddr}
	 *
	 *by:
	 * setenv mmcboot 'setenv bootargs console=ttyS0,115200 mem=800M root=${mmcroot} rw rootwait;bootz ${loadaddr} - ${fdtaddr}'
	 * saveenv
	 * ----setenv bootargs console=ttyS0,115200 root=${mmcroot} rw rootwait;bootz ${loadaddr} - ${fdtaddr}
	 *
	 * More infos
	 *
	 * https://forum.rocketboards.org/t/how-to-reserve-ddr3-memory-region-for-fpga-direct-access/162/6
	 * https://rocketboards.org/foswiki/view/Documentation/GSRD131ProgrammingFPGA#GSRD_FPGA_Configuration
	 *
	 * cat /proc/meminfo
	 *
	 * cat /proc/iomem
	 *
	 * 00000000-31ffffff : System RAM
	 * rest to 1024 is not occupied
	 *
	 * 32000000-3fffffff should be available
	 *
	 * 32000000-35ffffff = 64 MB
	 *
	 */

//--------------------------------------------------------------------------------------------------------


//#define soc_cv_av

#define DEBUG

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>
#include "hwlib.h"
#include "soc_cv_av/socal/socal.h"
#include "soc_cv_av/socal/hps.h"
#include "soc_cv_av/socal/alt_gpio.h"
#include "hps_0.h"
#include <sys/time.h> 


#include "sgdma.h"

#include "dma.h"



//DMA
//#include "alt_dma.h"
//#include "alt_globaltmr.h"



/*
/home/imp/altera/16.0/ip/altera/sopc_builder_ip/altera_avalon_sgdma/inc/altera_avalon_sgdma_regs.h
/home/imp/altera/16.0/ip/altera/sopc_builder_ip/altera_avalon_sgdma/HAL/inc/altera_avalon_sgdma.h
/home/imp/altera/16.0/ip/altera/sopc_builder_ip/altera_avalon_sgdma/HAL/inc/altera_avalon_sgdma_descriptor.h

ip/altera/nios2_ip/altera_nios2/HAL/inc/
*/

//corret in the Makre file the path to:
// /home/imp/altera/16.0/embedded/ip/altera/hps/altera_hps/hwlib/include

//https://www.altera.com/hps/en_us/cyclone-v/hps.html#topic/sfo1418687413697.html
//http://www.alteraforum.com/forum/showthread.php?t=42136

//settings for the lightweight HPS-to-FPGA bridge
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 ) //64 MB with 32 bit adress space this is 256 MB
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )


//setting for the HPS2FPGA AXI Bridge
#define ALT_AXI_FPGASLVS_OFST (0xC0000000) // axi_master
#define HW_FPGA_AXI_SPAN (0x40000000) // Bridge span 1GB
#define HW_FPGA_AXI_MASK ( HW_FPGA_AXI_SPAN - 1 )


//SDRAM 32000000-35ffffff //64 MB
#define SDRAM_64_BASE 0x32000000
#define SDRAM_64_SPAN 0x3FFFFFF

//SDRAM 36000000-36ffffff //16 MB
#define SDRAM_16_BASE 0x36000000
#define SDRAM_16_SPAN 0xFFFFFF

#define FIFO_BASE 0x30000
#define FIFO_REG_BASE 0x0100	// 寄存器的base
#define WAIT {}

#define FIFO_DELAY_BASE 0x40000
#define FIFO_DELAY_REG_BASE 0x41000


#define FIFO_WRITE		     (*(h2p_FIFO_addr))	// HPS to FPGA 
#define WRITE_FIFO_FILL_LEVEL (*h2p_lw_FIFO_reg_addr)
#define WRITE_FIFO_FULL		  ((*(h2p_lw_FIFO_reg_addr+1))& 1 )
#define WRITE_FIFO_EMPTY	  ((*(h2p_lw_FIFO_reg_addr+1))& 2 )
#define FIFO_WRITE_BLOCK(a)	  {while (WRITE_FIFO_FULL){WAIT};FIFO_WRITE=a;}	// 写入单个音符


//	void *h2p_FIFO_addr; //FIFO via AXI master
//	void *h2p_lw_FIFO_reg_addr; // FIFO register via 
int CountDelay(double theta){
    return 0;
}

#define VELOCITY 340 // 声速，单位m/s
#define DESTANCE 2   // 喇叭间距，单位cm
#define NUMBER 10    // 喇叭数量
#define FREQUN 10000000
#define PI 3.14159265
// 读取频率为10M

#define FIFO_DELAY_WRITE		    (*(h2p_FIFO_DELAY_addr))	// HPS to FPGA 
#define WRITE_FIFO_DELAY_FILL_LEVEL	(*h2p_lw_FIFO_DELAY_reg_addr)
#define WRITE_FIFO_DELAY_FULL		((*(h2p_lw_FIFO_DELAY_reg_addr+1))& 1 )
#define WRITE_FIFO_DELAY_EMPTY	  	((*(h2p_lw_FIFO_DELAY_reg_addr+1))& 2 )
#define FIFO_DELAY_WRITE_BLOCK(a)	{while (WRITE_FIFO_DELAY_FULL){WAIT};FIFO_DELAY_WRITE=a;}	// 写入单个音符


int main() {
    // // calculate the delay
    // double theta;
    // scanf("%d", &theta);
    // int delay = CountDelay(theta);

	//pointer to the different address spaces
	printf( "Hello -1\n" );
	void *virtual_base;
	void *axi_virtual_base;
	int fd;


	void *h2p_lw_reg1_addr;
	void *h2p_lw_reg2_addr;
	void *h2p_lw_reg3_addr;
	void *h2p_lw_reg_testdelay_addr;
	//void *h2p_lw_myBus_addr;


	void *h2p_led_addr; //led via AXI master
	void *h2p_rom_addr; //scratch space via ax master 64kb
	void *h2p_rom2_addr;

	void *sdram_64MB_add;
	void *sdram_16MB_add;


	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span
	printf( "Hello 0\n" );
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
	printf( "Hello 1\n" );
	//lightweight HPS-to-FPGA bridge
	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
	printf( "Hello 2\n" );
	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	//HPS-to-FPGA bridge
	axi_virtual_base = mmap( NULL, HW_FPGA_AXI_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd,ALT_AXI_FPGASLVS_OFST );

	if( axi_virtual_base == MAP_FAILED ) {
		printf( "ERROR: axi mmap() failed...\n" );
		close( fd );
		return( 1 );
	}


//-----------------------------------------------------------
	//configure the LEDs of the Golden Reference design
	printf( "\n\n\n-----------Set the LEDs on-------------\n\n" );

	//LED connected to the HPS-to-FPGA bridge
	h2p_led_addr=axi_virtual_base + ( ( unsigned long  )( 0x0 + PIO_LED_BASE ) & ( unsigned long)( HW_FPGA_AXI_MASK ) );

	*(uint32_t *)h2p_led_addr = 0b10111100;

//-----------------------------------------------------------
	//Adder test: Two registers are connected to a adder and place the result in the third register
	printf( "\n\n\n-----------Add two numbers in the FPGA-------------\n\n" );

	//the address of the two input (reg1 and reg2) registers and the output register (reg3)
	h2p_lw_reg1_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_REG1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_reg2_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_REG2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_reg3_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_REG3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_reg_testdelay_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_REG3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	// DEBUG: 上面的宏定义需要编译完之后改

	//write into register to test the adder
	*(uint32_t *)h2p_lw_reg1_addr = 10;
	*(uint32_t *)h2p_lw_reg2_addr = 5;

	//read result of the adder from register 3
	printf( "Adder result:%d + %d = %d\n", *((uint32_t *)h2p_lw_reg1_addr), *((uint32_t *)h2p_lw_reg2_addr), *((uint32_t *)h2p_lw_reg3_addr) );


//-------------------------------------------------------------
	//prepare the on chip memory devices
	printf( "\n\n\n-----------write on chip RAM-------------\n\n" );

	//ONCHIP_MEMORY2_0_BASE connected via the HPS-to-FPGA bridge
	h2p_rom_addr=axi_virtual_base + ( ( unsigned long  )( ONCHIP_MEMORY2_0_BASE ) & ( unsigned long)( HW_FPGA_AXI_MASK ) );

	h2p_rom2_addr=axi_virtual_base + ( ( unsigned long  )( ONCHIP_MEMORY2_1_BASE ) & ( unsigned long)( HW_FPGA_AXI_MASK ) );


	//write some data to the scatch disk
	for (int i=0;i<16000;i++){
		*((uint32_t *)h2p_rom_addr+i)=i*1024;
		*((uint32_t *)h2p_rom2_addr+i)=i+3;
	}

	printf( "Print scratch disks:\n" );
	printf( "ROM1 \t ROM2\n");
	for (int i=0;i<10;i++){
		printf( "%d\t%d\n", *((uint32_t *)h2p_rom_addr+i),*((uint32_t *)h2p_rom2_addr+i) );
	}

//-----------------------------------------------
	//DMA

	printf( "\n\n\n-----------DMA RAM to RAM-------------\n\n" );


	//print the content of scratchdisk 1 and 2
	printf( "Print scratch disk 1 and 2:\n" );
	for (int i=0;i<10;i++){
		printf( "%d\t%d\n", *((uint32_t *)h2p_rom_addr+i),*((uint32_t *)h2p_rom2_addr+i) );
	}


	//create a pointer to the DMA controller base
	h2p_lw_dma_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DMA_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

	//configure the DMA controller for transfer
	_DMA_REG_STATUS(h2p_lw_dma_addr)=0;
	_DMA_REG_READ_ADDR(h2p_lw_dma_addr)=ONCHIP_MEMORY2_0_BASE; //read from ROM1
	_DMA_REG_WRITE_ADDR(h2p_lw_dma_addr)=ONCHIP_MEMORY2_1_BASE; //write to ROM2
	_DMA_REG_LENGTH(h2p_lw_dma_addr)=4*16000;//write 100x 4bytes since we have a 32 bit system

	//start the transfer
	_DMA_REG_CONTROL(h2p_lw_dma_addr)=_DMA_CTR_WORD | _DMA_CTR_GO | _DMA_CTR_LEEN;


	debugPrintDMARegister();

	debugPrintDMAStatus();

	//wait for DMA to be finished
	waitDMAFinish();
	stopDMA();//stop the DMA controller


	//check if data was copied
	printf( "Print scratch disk 1 and 2:\n" );
	for (int i=0;i<10;i++){
		printf( "%d\t %d\n", *((uint32_t *)h2p_rom_addr+i),*((uint32_t *)h2p_rom2_addr+i) );
	}

	// by hjx write the delay
	volatile unsigned int *h2p_FIFO_DELAY_addr;
	volatile unsigned int *h2p_lw_FIFO_DELAY_reg_addr;
	h2p_FIFO_DELAY_addr = axi_virtual_base + ((unsigned long)(0x0 + FIFO_DELAY_BASE)&(unsigned long)(HW_FPGA_AXI_MASK));
	h2p_lw_FIFO_DELAY_reg_addr = virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FIFO_DELAY_REG_BASE) & (unsigned long)(HW_REGS_MASK));
	// create a sub-process to calculate and convey the delay
	pid_t pid;
	if((pid = fork()) < 0){
		printf("Fork Error!\n");
		exit(1);
	}
	else if(pid > 0){	// in the sub-process
		while (1){
        	double theta;
        	scanf("%lf", &theta);
        	double delta_d = DESTANCE * cos(theta) / 100;
        	double delta_t = delta_d / VELOCITY;
        	// printf("### ");
			// 开始符
			FIFO_DELAY_WRITE_BLOCK(233333);
			// 延迟位置
        	if(theta > PI / 2){
        	    int gap =  -delta_t * (NUMBER - 1) * FREQUN;
        	    for (int i = NUMBER - 1; i >= 0; i--){
					int pos = (int)(delta_t * i * FREQUN) + gap;
        	        // printf("%d ", pos);
					FIFO_DELAY_WRITE_BLOCK(pos);
        	    }
        	}
        	else{
        	    for (int i = 0; i < NUMBER; i++){
					int pos = (int)(delta_t * i * FREQUN);
        	        // printf("%d ", pos);
					FIFO_DELAY_WRITE_BLOCK(pos);
        	    }
        	}
        	// printf("###\n");
			// 结束符
			FIFO_DELAY_WRITE_BLOCK(233333);
    	}
	}

	//Access FIFO
	struct timeval t1, t2;
	double elapsedTime;

	///////////////////////////////////////////////////////////////////////////
	/////////////////////////////TODO//////////////////////////////////////////
	volatile unsigned int *h2p_FIFO_addr; //FIFO via AXI master 总线地址
	volatile unsigned int *h2p_lw_FIFO_reg_addr; // FIFO register via 寄存器总线地址
	// 桥的地址加上偏移量 FIFO_BASE or FIFO_REG_BASE
	h2p_FIFO_addr = axi_virtual_base + ( ( unsigned long  )( 0x0 + FIFO_BASE ) & ( unsigned long)( HW_FPGA_AXI_MASK ) );
	h2p_lw_FIFO_reg_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FIFO_REG_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

	printf( "write LEVEL: %d \n", *(h2p_lw_FIFO_reg_addr) );
	//h2p_led_addr=axi_virtual_base + ( ( unsigned long  )( 0x0 + PIO_LED_BASE ) & ( unsigned long)( HW_FPGA_AXI_MASK ) );
	//-------------------------------------------------------------
	// clean up our memory mapping and exit

	// TODO: 写文件
	FILE *fp;
    fp = fopen("./40k_lut.txt", "r");
	int value;
	int lut[100000*5];	// 音频数据
	for(int i = 0; i < 100000*5; i++)
	{
		fscanf(fp, "%d", &value);
		lut[i] = value;
		//printf("%d ", value);
	}

	printf("reading file finished\n", value);

	gettimeofday(&t1, NULL);
	for(long int i = 0; i < 100000*5; i++)
	{
		//printf( "write LEVEL: %d \n", *(h2p_lw_FIFO_reg_addr) );
		//FIFO_WRITE_BLOCK(i % 100);
		FIFO_WRITE_BLOCK(lut[i]);	// 直接写
		//FIFO_WRITE_BLOCK(333);
		//*h2p_FIFO_addr = i;
	}
	gettimeofday(&t2, NULL);
	printf( "write LEVEL: %d \n", *(h2p_lw_FIFO_reg_addr) );
	

	elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000000.0;      // sec to us
	elapsedTime += (t2.tv_usec - t1.tv_usec) ;   // us to 
	printf("FIFO write T=%.0f uSec  n/sec=%2.3e\n\r", elapsedTime, 20*1e6/elapsedTime);
	//printf("count=%d\n\r", count) ;
		



	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	if( munmap( axi_virtual_base, HW_FPGA_AXI_SPAN ) != 0 ) {
		printf( "ERROR: axi munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );
	wait(0);
	return( 0 );
}
