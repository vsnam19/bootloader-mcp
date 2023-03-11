/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "Bootloader.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/** Array store vector table of the bootloader in ram */
uint32_t g_VectorsTableRam[NUMBER_OF_INT_VECTORS] __attribute__((aligned(256u)));
/** Store information about a record*/
SREC_Infomation_Type g_srecInfo;
/** Mode run to switch running - program */
volatile eProgramMode_Type g_ProgramMode = eAppMode;
volatile Error_Type g_ErrorCode = eNope;
/*******************************************************************************
 * Code
 ******************************************************************************/

int main(void)
{
   uint8_t ErrorExist = 0u;
   uint32_t u32Timeout = 0;
   uint8_t u8TimeoutErrorCounter = 0;
   uint32_t u32AppResetHanler = 0xFFFFFFFFu;
   volatile uint8_t u8Queue_Front;

   Flash_Init();

   /**
    * Initialize button interrupt to detect user action.
    */
   Board_SwitchInit();
   PORT_ConfigInterrupt(BOARD_SW_PORT, BOARD_SW1_PIN, PORT_IRQLogicZero);
   NVIC_EnableIRQ(PORTC_PORTD_IRQn);

   /** Move Vectortble to RAM */
   CopyVectorTable(VECTORTABLE_DEFAULT_ADDRESS, g_VectorsTableRam);
   ChangeIRQHandler((uint32_t *)g_VectorsTableRam, UART0_IRQn, &UART0_IRQHandlerInRAM);
   MoveVectorTable((uint32_t)g_VectorsTableRam);

   SetupUART();
   RoutingManager();

   /******************************************************************************
    *
    *                                 BOOT MODE
    *
   *******************************************************************************/

   /** Erase old application for update */
   UART0_PutString("Erase application memory:");
   EraseFlashMemory(APP_REGION_START, APP_REGION_END, SECTION_SIZE);
   UART0_PutString("Finished!\n");
   UART0_PutString("Send new firmware to update!\n");

   while (1)
   {
      while ((g_srecInfo.u8RecordType != Termination16BitsType) &&
             (g_srecInfo.u8RecordType != Termination24BitsType) &&
             (g_srecInfo.u8RecordType != Termination32BitsType))
      { /** Get the next record */

         u8Queue_Front = queue_getFront();
         if ((u8Queue_Front < MAX_QUEUE_SIZE))
         {
            /** The timeout counter to check lost connection */
            u32Timeout = 0;
            u8TimeoutErrorCounter = 0;

            /** If the record valid, get infomation of record */
            if (SREC_GetSrecInfo(queue[u8Queue_Front].record, &g_srecInfo))
            {
               /** Remove the front of queue */
               queue_dequeue();

               /**
             * Store reset handler of application to make sure
             * the update process complete successfully.
             */
               if (g_srecInfo.u32Address == APP_REGION_START)
               {
                  u32AppResetHanler = GetResetHandler(g_srecInfo.data);
               }

               if (!ErrorExist)
               {
                  WriteRecordToFlash(g_srecInfo);
               }
            }
            else
            {
               g_ErrorCode = eSrec_Error;
            }
         }
         else
         {
            u32Timeout += 1u;

            /** Timeout verify */
            if (u32Timeout == TIMEOUT_U32)
            {
               u32Timeout = 0;
               u8TimeoutErrorCounter++;
               g_ErrorCode = eTimeout_Error;

               if (u8TimeoutErrorCounter == TIMEOUT_LIMIT_TOBACKUP_U8)
               {
                  if (EXIST_BACKUP)
                  {
                     UART0_PutString("Restore old firmware!");
                     RestoreBackupApp();
                     UART0_PutString("Restore successful.\n");
                     JumpToApplication((uint32_t *)APP_REGION_START);
                  }
               }
            }
         }

         /** Overrun UART verify */
         if (UART0_GetOverrunFlag())
         {
            g_ErrorCode = eUARTOverrun_Error;
         }

         if (g_ErrorCode != eNope)
         {
            ResetBootModeState();
            ErrorReport(g_ErrorCode);
            u32AppResetHanler = 0xFFFFFFFFu;
            g_ErrorCode = eNope;
            ErrorExist = 1u;
         }
      }
      /** If detect termination of srec file, store reset handler and jump to application */
      if (u32AppResetHanler != 0xFFFFFFFFu)
      {
         FLASH_WriteLongWord(APP_RESETHANDLER_ADDR, u32AppResetHanler);
         UART0_PutString("\nUpdate successfully! Run Application.\n");
         JumpToApplication((uint32_t *)APP_REGION_START);
      }
      else
      {
         g_srecInfo.u8RecordType = HeaderType;
         EraseFlashMemory(APP_REGION_START, APP_REGION_END, SECTION_SIZE);
         ErrorExist = 0u;
      }
   }
}

/**----------------------------------------------------------------------------
 * Function name: GetResetHandler
 * Description: Get reset handler address and
 *              clear reset handler in record to 0xFFFFFFFFu.
 *----------------------------------------------------------------------------*/
static inline uint32_t GetResetHandler(uint8_t *data)
{
   uint32_t u32ResetHanler = data[APP_RESETHANDLER_OFFSET] |
                             (data[APP_RESETHANDLER_OFFSET + 1] << 8u) |
                             (data[APP_RESETHANDLER_OFFSET + 2] << 16u) |
                             (data[APP_RESETHANDLER_OFFSET + 3] << 24u);

   /** Clear reset handler in record to 0xFFFFFFFFu */
   data[APP_RESETHANDLER_OFFSET] = 0xFFu;
   data[APP_RESETHANDLER_OFFSET + 1u] = 0xFFu;
   data[APP_RESETHANDLER_OFFSET + 2u] = 0xFFu;
   data[APP_RESETHANDLER_OFFSET + 3u] = 0xFFu;
   return u32ResetHanler;
}

/**----------------------------------------------------------------------------
 * Function name: ModeVectorTable
 * Description: Move vector table to the specific destination
 *----------------------------------------------------------------------------*/
void MoveVectorTable(uint32_t destination)
{
   __disable_irq();
   __DMB();
   SCB->VTOR = destination;
   __DSB();
   __enable_irq();
}

/**----------------------------------------------------------------------------
 * Function name: Copy vector table to specific destination
 * Description: Move vector table to the specific destination
 *----------------------------------------------------------------------------*/
void CopyVectorTable(uint32_t base, uint32_t dest[])
{
   uint8_t index;
   for (index = 0; index < NUMBER_OF_INT_VECTORS; index++)
   {
      dest[index] = *((uint32_t *)(base + index * 0x04u));
   }
}

/**----------------------------------------------------------------------------
 * Function name: ChangeIRQHandler
 * Description: Change default IRQ Handler to new handler
 *----------------------------------------------------------------------------*/
void ChangeIRQHandler(uint32_t *IntvectAddr,
                      IRQn_Type IRQn,
                      void (*newHandler)(void))
{
   IntvectAddr[(uint8_t)IRQn + 16u] = (uint32_t)newHandler;
}

/**----------------------------------------------------------------------------
 * Function name: SetupUART
 * Description: Initialize for uart
 *----------------------------------------------------------------------------*/
void SetupUART(void)
{
   UART0_Config_Type uartCfg;

   /** OSC clock mode */

   UART0_GetDefaultConfig(&uartCfg);
   /** Enable receive interrupt  */
   uartCfg.enableReceiveInterrupt = eEnable;

   UART0_Init(uartCfg);

#if UART0_CLOCK_MODE == 0u
   UART0_SelectSource(Clock_MCGFLLPLL);
   UART0_SetBaudRate(DEFAULT_SYSTEM_CLOCK,
                     UART0_BAUDRATE_SEL);
#elif UART0_CLOCK_MODE == 1u
   UART0_SelectSource(Clock_OSCERCLK);
   UART0_SetBaudRate(CPU_XTAL_CLK_HZ,
                     UART0_BAUDRATE_SEL);
#endif

   UART0_EnableTx();
   UART0_EnableRx();

   NVIC_EnableIRQ(UART0_IRQn);
}

/**----------------------------------------------------------------------------
 * Function name: ErrorReport
 * Description: Report error to serial port.
 *----------------------------------------------------------------------------*/
void ErrorReport(Error_Type errorID)
{

   switch (errorID)
   {
   case eTimeout_Error:
   {
      UART0_PutString("\nTimeout. Send firmware again to re-update.\n");
      break;
   }
   case eSrec_Error:
   {
      UART0_PutString("\nYour firmware incorrect, check it. And try again\n");
      break;
   }
   case eUARTOverrun_Error:
   {

      UART0_PutString("\nDecrease the baudrate. And send firmware again to re-update.\n");
      break;
   }
   case eQueueFull_Error:
   {

      UART0_PutString("\nDecrease the baudrate. And send firmware again to re-update.\n");
      break;
   }
   }
}

/**----------------------------------------------------------------------------
 * Function name: Write one Record to flash memory
 * Description: Initialize for uart
 *----------------------------------------------------------------------------*/
void WriteRecordToFlash(SREC_Infomation_Type info)
{
   uint8_t u8CurrentIndex = 0;

   if ((info.u8RecordType == Data16BitsType) ||
       (info.u8RecordType == Data32BitsType) ||
       (info.u8RecordType == Data24BitsType))
   {
      while (u8CurrentIndex < info.u8DataSize)
      {

         FLASH_WriteLongWord(info.u32Address + u8CurrentIndex,
                             PackingLongwords(info.data[u8CurrentIndex],
                                              info.data[u8CurrentIndex + 1u],
                                              info.data[u8CurrentIndex + 2u],
                                              info.data[u8CurrentIndex + 3u]));
         u8CurrentIndex += 4u;
      }
   }
}

/**----------------------------------------------------------------------------
 * Function name: EraseAppMemory
 * Description: Eraser the memory of the application.
 *----------------------------------------------------------------------------*/
void EraseFlashMemory(uint32_t start, uint32_t end, uint16_t sectionSize)
{

   while (start < end)
   {
      FLASH_EraseSector(start);
      start += sectionSize;
      UART0_PutChar('.');
   }
}

/**----------------------------------------------------------------------------
 * Function name: JumpToApplication
 * Description: Jump to application program.
 *----------------------------------------------------------------------------*/
void JumpToApplication(uint32_t *AppAddr)
{
   /** Clear all interrupt  */
   NVIC_DisableAllIRQ();

   /** Clear all pending interrupts */
   NVIC_ClearAllPendingIRQ();

   /** Disable peripherals */
   Board_SwitchDeinit();
   UART0_Deinit();

   MoveVectorTable((uint32_t)AppAddr);
   __JumpASM(AppAddr[0], AppAddr[1]);
}

/**----------------------------------------------------------------------------
 * Function name: __JumpASM
 * Description: Jump to application program by asm instruction.
 *----------------------------------------------------------------------------*/
__naked void __JumpASM(uint32_t SP, uint32_t ResetHandler)
{
   __asm("MSR MSP,R0");
   __asm("BX R1");
}

/**----------------------------------------------------------------------------
 * Function name: PackingLongword
 * Description: Convert 4 byte to long word form
 *----------------------------------------------------------------------------*/

static inline uint32_t PackingLongwords(uint8_t byte1, uint8_t byte2,
                                        uint8_t byte3, uint8_t byte4)
{
   return (uint32_t)((uint32_t)byte1 |
                     (uint32_t)(byte2 << 8u) |
                     (uint32_t)(byte3 << 16u) |
                     (uint32_t)(byte4 << 24u));
}

/**----------------------------------------------------------------------------
 * Function name: ResetBootModeState
 * Description: Reset state of boot program for the next update.
 *----------------------------------------------------------------------------*/
void ResetBootModeState(void)
{
   while (UART0->S1 & UART0_S1_IDLE_MASK == 0)
   {
   }

   queue_reset();
   UART0_ClearOverrunFlag();
}

/**----------------------------------------------------------------------------
 * Function name: BackupApplication
 * Description: Store application code in backup memory
 *----------------------------------------------------------------------------*/
void BackupApplication(void)
{
   uint32_t index;
   uint32_t regionSize = BACKUP_REGION_END - BACKUP_REGION_START;

   UART0_PutString("Erase backup memory:");
   EraseFlashMemory(BACKUP_REGION_START, BACKUP_REGION_END, SECTION_SIZE);
   UART0_PutString("Finished!\n");

   for (index = 0; index < regionSize; index += 4u)
   {
      FLASH_WriteLongWord(BACKUP_REGION_START + index,
                          PackingLongwords(((uint8_t *)APP_REGION_START)[index],
                                           ((uint8_t *)APP_REGION_START)[index + 1u],
                                           ((uint8_t *)APP_REGION_START)[index + 2u],
                                           ((uint8_t *)APP_REGION_START)[index + 3u]));
   }
}

/**----------------------------------------------------------------------------
 * Function name: RestoreBackupApp
 * Description: Restore application program from backup memory
 *----------------------------------------------------------------------------*/
void RestoreBackupApp(void)
{
   uint32_t index;
   uint32_t regionSize = BACKUP_REGION_END - BACKUP_REGION_START;

   UART0_PutString("Erase application memory:");
   EraseFlashMemory(APP_REGION_START, APP_REGION_END, SECTION_SIZE);
   UART0_PutString("Finished!\n");

   for (index = 0; index < regionSize; index += 4u)
   {
      FLASH_WriteLongWord(APP_REGION_START + index,
                          PackingLongwords(((uint8_t *)BACKUP_REGION_START)[index],
                                           ((uint8_t *)BACKUP_REGION_START)[index + 1u],
                                           ((uint8_t *)BACKUP_REGION_START)[index + 2u],
                                           ((uint8_t *)BACKUP_REGION_START)[index + 3u]));
   }
}

/**----------------------------------------------------------------------------
 * Function name: PORTC_PORTD_IRQHandler
 * Description: Hanlder for button interrupt to detect user action.
 *----------------------------------------------------------------------------*/
void PORTC_PORTD_IRQHandler(void)
{
   if (PORT_GetInterruptFlag(BOARD_SW_PORT, BOARD_SW1_PIN))
   {
      g_ProgramMode = eBootMode;
      PORT_ClearInterruptFlag(BOARD_SW_PORT, BOARD_SW1_PIN);
   }
}

void RoutingManager(void)
{

   if (g_ProgramMode == eAppMode)
   {
      if (EXIST_APPLICATION)
      {
         UART0_PutString("Running Application!");
         JumpToApplication((uint32_t *)APP_REGION_START);
      }
      else if (EXIST_BACKUP)
      {
         UART0_PutString("Restore old firmware!");
         RestoreBackupApp();
         UART0_PutString("Restore successful.\n");
         JumpToApplication((uint32_t *)APP_REGION_START);
      }
      else
      {
         UART0_PutString("Cant find the program!");
         g_ProgramMode = eBootMode;
      }
   }
   else
   {
      if (EXIST_APPLICATION)
      {
         UART0_PutString("Backup processing.\n");
         BackupApplication();
         UART0_PutString("Backup successful.\n");
      }
   }
}