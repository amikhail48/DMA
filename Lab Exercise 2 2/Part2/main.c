/******************************************************************************************
					PROJECT DESCRIPTION 
*******************************************************************************************  
* The project demonstrates how to copy data from memory to memory (flash to RAM) using DMA.
* The DMA transfers 8 bytes of data from a flash array to a RAM array on CPU request. 
* The RAM array contents are then displayed on the LCD.
    
*******************************************************************************************/
#include <project.h>
#include <device.h>

#define DMA_BURST_COUNT 4
#define DMA_REQUEST_PER_BURST 4

#define DMA_SRC_BASE (CYDEV_SRAM_BASE)
#define DMA_DST_BASE (CYDEV_SRAM_DST)

/* Variable declarations for DMA . 
 * DMA_Chan is used to store the DMA channel handle */ 
uint8 DMA_Chan;
uint8 DMA_TD[5];
uint8 srcArray[16384];
uint8 dstArray[16384];
uint16 checkTransfer, error;
int time,timerCount, i;
int timerPeriod = 16777216;

/* DMA_DISABLE_TD definition not available for PSoC 5LP inside CyDmac.h in PSoC Creator 2.1 SP1.
 * So, manually define the DMA_DISABLE_TD */
#if (CY_PSOC5LP)
	#if !defined(DMA_DISABLE_TD)
	    #define DMA_DISABLE_TD          0xFEu
	#endif
#endif  /* (CY_PSOC5LP) */

CY_ISR(transferInterrupt){
    Timer_Stop();
    timerCount = Timer_ReadCounter();
   LCD_Position(1,10);
    LCD_PrintString("IN");
    CyDelay(200);
    for(i=0; i<16384; i=i+1){
        if(dstArray[i] != srcArray[i+3]){
            error++;
        }        
        if(dstArray[i+1] != srcArray[i+2]){
            error++;
        }               
        if(dstArray[i+2] != srcArray[i+1]){
            error++;
        }       
        if(dstArray[i+3] != srcArray[i]){
            error++;
        }
    }
}
void main()
{ 
    /* Start LCD and enable all interrupts */
    LCD_Start();
	
	/* Enable Interrupts */
    CYGlobalIntEnable;
    isr_StartEx(transferInterrupt);
    
    /*Initialize Source and Destination Arrays*/
    for(i=0;i<16384;i++){
        srcArray[i] = i%256;
        dstArray[i] = 0;
    }

    /* Step1 : DmaInitialize - Initialize the DMA 
	 * Burst count (Bytes per burst) = 1 byte. Transfer data byte by byte.
	 * Upper source address = HI16(CYDEV_FLS_BASE) for PSoC 3
							= HI16(sourceArray)for PSoC 5 LP
	 * Upper destination address = HI16(CYDEV_SRAM_BASE) for PSoC 3
								 = HI16(destinationArray)for PSoC 5 LP
	 * DMA_Chan holds the channel handle returned by the ‘DmaInitialize’ function. This is 
	 * used for all further references of the channel 
	 * The parameter upper 16 bits of the source and destination address is specific to the device family.
	 * Hence the DMA Initialization will change based on PSoC3 or PSoC5.
	 * In order to make the code compatible with both PSoC3 and PSoC5, the DMA
     * initialization is done within preprocessor statement which checks for the compiler used.
     * PSoC3 uses Keil_C51 and it is defined using __C51__
     * else initialization for PSoC5 is used */
	#if (defined(__C51__))
	    DMA_Chan = DMA_DmaInitialize(1, 0, HI16(CYDEV_FLS_BASE), HI16(CYDEV_SRAM_BASE));
    
    #else
        DMA_Chan = DMA_DmaInitialize(1, 0, HI16(srcArray), HI16(dstArray) );
    #endif
    
    /* Step2 :CyDmaTdAllocate - Allocate TD */
	DMA_TD[0] = CyDmaTdAllocate();
    DMA_TD[1] = CyDmaTdAllocate();
    DMA_TD[2] = CyDmaTdAllocate();
    DMA_TD[3] = CyDmaTdAllocate();
    DMA_TD[4] = CyDmaTdAllocate();
    
	/* Step3 :CyDmaTdSetConfiguration - Configures the TD:
	 * tdHandle = DMA_TD[0] - TD handle previously returned by CyDmaTdAlloc()
     * Next Td = DMA_INVALID_TD , disable the channel after the transfer is finished
	 * TD configuration = increment source and destination addresses after each transaction */	
    // Configure the TDs to perform an endian swap of size 4 bytes before moving on to the next TD. 
    CyDmaTdSetConfiguration(DMA_TD[0], 4092, DMA_TD[1], TD_SWAP_EN | TD_SWAP_SIZE4 | TD_INC_SRC_ADR | TD_INC_DST_ADR | TD_AUTO_EXEC_NEXT);
    CyDmaTdSetConfiguration(DMA_TD[1], 4092, DMA_TD[2], TD_SWAP_EN | TD_SWAP_SIZE4 | TD_INC_SRC_ADR | TD_INC_DST_ADR | TD_AUTO_EXEC_NEXT);
    CyDmaTdSetConfiguration(DMA_TD[2], 4092, DMA_TD[3], TD_SWAP_EN | TD_SWAP_SIZE4 | TD_INC_SRC_ADR | TD_INC_DST_ADR | TD_AUTO_EXEC_NEXT);
    CyDmaTdSetConfiguration(DMA_TD[3], 4092, DMA_TD[4], TD_SWAP_EN | TD_SWAP_SIZE4 | TD_INC_SRC_ADR | TD_INC_DST_ADR | TD_AUTO_EXEC_NEXT);
    CyDmaTdSetConfiguration(DMA_TD[4], 16, CY_DMA_DISABLE_TD, TD_SWAP_EN | TD_SWAP_SIZE4 | DMA__TD_TERMOUT_EN | TD_INC_SRC_ADR | TD_INC_DST_ADR);
    
    /* Step 4 : CyDmaTdSetAddress - Configure the lower 16 bit source and destination addresses 
	 * Set the source address as sourceArray and destination address as destination Array */
	CyDmaTdSetAddress(DMA_TD[0], (uint16)((uint32)srcArray), (uint16)((uint32)dstArray) );
    CyDmaTdSetAddress(DMA_TD[1], (uint16)((uint32)srcArray)+4092, (uint16)((uint32)dstArray)+4092 );
	CyDmaTdSetAddress(DMA_TD[2], (uint16)((uint32)srcArray)+8184, (uint16)((uint32)dstArray)+8184 );
	CyDmaTdSetAddress(DMA_TD[3], (uint16)((uint32)srcArray)+12276, (uint16)((uint32)dstArray)+12276 );
	CyDmaTdSetAddress(DMA_TD[4], (uint16)((uint32)srcArray)+16368, (uint16)((uint32)dstArray)+16368 );

    /* Step 5: Map the TD to the DMA Channel */
	CyDmaChSetInitialTd(DMA_Chan, DMA_TD[0]);
	
    /* Step 6: Start Timer and Enable the DMA channel */
    Timer_Start();
	CyDmaChEnable(DMA_Chan, 1);
	
    /* Trigger DMA channel using CPU */
	CyDmaChSetRequest(DMA_Chan, CPU_REQ);	
 
    for(;;){
        /*Print out Time and Number of Errors*/
        LCD_ClearDisplay();
        LCD_PrintString("Time:");
        time = (timerPeriod-timerCount)/24; //Divide by 24 to get time in microseconds (24MHz Clock)
        LCD_PrintNumber(time);
        LCD_Position(1,0);
        LCD_PrintString("Errors:");
        LCD_PrintNumber(error);
        CyDelay(100);
	}
}
/* [] END OF FILE */