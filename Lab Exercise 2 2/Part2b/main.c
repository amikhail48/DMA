/******************************************************************************************
					PROJECT DESCRIPTION 
*******************************************************************************************  
* The project demonstrates how to copy data from memory to memory (flash to RAM) using DMA.
* The DMA transfers 8 bytes of data from a flash array to a RAM array on CPU request. 
* The RAM array contents are then displayed on the LCD.
    
*******************************************************************************************/
#include <project.h>
#include <device.h>

uint8 srcArray[16384];
uint8 dstArray[16384];
uint8 checkTransfer, error;
int time,timerCount, i;
int timerPeriod = 16777216;

void main()
{ 
    /* Start LCD and enable all interrupts */
    LCD_Start();
	
    /*Initialize Source and Destination Arrays*/
    for(i=0;i<16384;i++){
        srcArray[i] = i%256;
        dstArray[i] = 0;
    }
    Timer_Start();
    /*Transpose Array using Software loop*/
    for(i=0;i<16384;i++){
        dstArray[i] = srcArray[i+3];
        dstArray[i+1] = srcArray[i+2];
        dstArray[i+2] = srcArray[i+1];
        dstArray[i+3] = srcArray[i];
    }
    Timer_Stop();
    timerCount = Timer_ReadCounter();
    for(i=0; i<4096; i=i+4){
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
    /*Print out Time and Number of Error*/
    LCD_ClearDisplay();
    LCD_PrintString("Time:");
    time = (timerPeriod-timerCount)/24; //Divide by 24 to get time in microseconds (24MHz Clock)
    LCD_PrintNumber(time);
    LCD_Position(1,0);
    LCD_PrintString("Errors:");
    LCD_PrintNumber(error);
    CyDelay(100);
}
/* [] END OF FILE */