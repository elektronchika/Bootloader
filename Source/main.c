// ----------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016 by MED-EL Elektromedizinische Geraete GmbH. All rights reserved!
// ----------------------------------------------------------------------------------------------------------------------

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "flash.h"
#include "srecord.h"

#define USER_APPLICATION_BASE_ADDRESS 0x8004000U

/* http://www.keil.com/support/docs/3913.htm */

uint8_t srec_data[16];

void __svc(1)   EnablePrivilegedMode(void );
void __SVC_1    (void){
        __disable_irq();
        __set_CONTROL((__get_CONTROL( ))& 0xFFFFFFFE);  // enter priv mode
        __enable_irq();
}

static void BootJump(uint32_t *Address)
{
  
  /* 1. Make sure, the CPU is in privileged mode. */
  if (CONTROL_nPRIV_Msk & __get_CONTROL())
  {  /* not in privileged mode */
    EnablePrivilegedMode();
  }
  
  /* 2. Disable all enabled interrupts in NVIC. */
  NVIC->ICER[0] = 0xFFFFFFFF;
  NVIC->ICER[1] = 0xFFFFFFFF;
  NVIC->ICER[2] = 0xFFFFFFFF;
  NVIC->ICER[3] = 0xFFFFFFFF;
  NVIC->ICER[4] = 0xFFFFFFFF;
  NVIC->ICER[5] = 0xFFFFFFFF;
  NVIC->ICER[6] = 0xFFFFFFFF;
  NVIC->ICER[7] = 0xFFFFFFFF;
  
  /* 3. Disable all enabled peripherals which might generate interrupt requests,
        and clear all pending interrupt flags in those peripherals. Because this
        is device-specific, refer to the device datasheet for the proper way to
        clear these peripheral interrupts.  */
  
  
  /* 4. Clear all pending interrupt requests in NVIC. */
  NVIC->ICPR[0] = 0xFFFFFFFF;
  NVIC->ICPR[1] = 0xFFFFFFFF;
  NVIC->ICPR[2] = 0xFFFFFFFF;
  NVIC->ICPR[3] = 0xFFFFFFFF;
  NVIC->ICPR[4] = 0xFFFFFFFF;
  NVIC->ICPR[5] = 0xFFFFFFFF;
  NVIC->ICPR[6] = 0xFFFFFFFF;
  NVIC->ICPR[7] = 0xFFFFFFFF;
  
  /* 5. Disable SysTick and clear its exception pending bit, if it is used in the bootloader, e. g. by the RTX.  */
  SysTick->CTRL = 0;
  SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;
  
  /* 6. Disable individual fault handlers if the bootloader used them. */
  SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_MEMFAULTENA_Msk);
  
  /* 7. Activate the MSP, if the core is found to currently run with the PSP. */
  if( CONTROL_SPSEL_Msk & __get_CONTROL())
  {  /* MSP is not active */
    __set_CONTROL(__get_CONTROL() & ~CONTROL_SPSEL_Msk);
  }
  
  /*8. Load the vector table address of the user application into SCB->VTOR register.
       Make sure the address meets the alignment requirements of the register. */
  SCB->VTOR = (uint32_t)Address;
  
  /* 9. Set the MSP to the value found in the user application vector table. */
  __set_MSP( Address[0]);

  /* 10. Set the PC to the reset vector value of the user application via a function call. */
  ((void(*)(void))Address[1])();
  
}

/* UART related  */
void uart_init_read(void)
{
    
    /* Enable GPIOA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    /* Enable Tx and Rx pins alternate functions */
    //GPIOA->MODER |= ((2 << 2*10) + (2 << 2*9));
    //GPIOA->AFR[1] |= (7 << 8) + (7 << 4);
    GPIOA->MODER |= (2 << 2*2);
    GPIOA->AFR[0] |= (7 << 8);
    GPIOA->OSPEEDR |= (3 << 4);
    //GPIOA->PUPDR |= (1 << 2*15);
    
    /* Enable USART1 clock */
    //RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    
    /* Reset USART2 */
    RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
    
    /* Enable USART2 clock */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    
    /* RM0383 page 504 */
    /* 1. Enable the USART by writing the UE bit in USART_CR1 register to 1. */
    USART2->CR1 |= USART_CR1_UE;
    
    /* 2. Program the M bit in USART_CR1 to define the word length. 8 bits */
    USART2->CR1 &= ~(USART_CR1_M);
    
    /* 3. Program the number of stop bits in USART_CR2. 1 stop bit */
    USART2->CR2 &= ~(USART_CR2_STOP_0 + USART_CR2_STOP_1);
    
    /* 4. Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to take
          place. Configure the DMA register as explained in multibuffer communication. */
    
    /* 5. Select the desired baud rate using the USART_BRR register. 115200 */
    //USART1->BRR |= (138 << 4) + (14);
    USART2->BRR |= (8 << 4) + (11);
    
    /* 6. Set the TE bit in USART_CR1 to send an idle frame as first transmission. */
    USART2->CR1 |= USART_CR1_TE;
    
    /* 7. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
          for each data to be transmitted in case of single buffer. */
    while(!(USART2->SR & USART_SR_TC));
    USART2->DR = 0x30;
    while(!(USART2->SR & USART_SR_TC));
    USART2->DR = 0x31;
    while(!(USART2->SR & USART_SR_TC));
    USART2->DR = 0x32;
    while(!(USART2->SR & USART_SR_TC));
    USART2->DR = 0x33;
    
    
    /* 8. After writing the last data into the USART_DR register, wait until TC=1. This indicates
          that the transmission of the last frame is complete. This is required for instance when
          the USART is disabled or enters the Halt mode to avoid corrupting the last
          transmission. */
    while(!(USART2->SR & USART_SR_TC));
    
}

void uart_init(void)
{
    
    /* Enable GPIOA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    /* Enable Tx and Rx pins alternate functions */
    /* PA2 is TX, PA3 is RX */
    GPIOA->MODER |= (2 << 2*2) + (2 << 2*3);
    GPIOA->AFR[0] |= (7 << 8) + (7 << 12);
    GPIOA->OSPEEDR |= (3 << 4) + (3 << 6);
    
    /* Enable USART2 clock */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    
    /* RM0383 page 504 */
    /* 1. Enable the USART by writing the UE bit in USART_CR1 register to 1. */
    USART2->CR1 |= USART_CR1_UE;
    
    /* 2. Program the M bit in USART_CR1 to define the word length. 8 bits */
    USART2->CR1 &= ~(USART_CR1_M);
    
    /* 3. Program the number of stop bits in USART_CR2. 1 stop bit */
    USART2->CR2 &= ~(USART_CR2_STOP_0 + USART_CR2_STOP_1);
    
    /* 4. Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to take
          place. Configure the DMA register as explained in multibuffer communication. */
    
    /* 5. Select the desired baud rate using the USART_BRR register. 115200 */
    //USART1->BRR |= (138 << 4) + (14);
    USART2->BRR |= (8 << 4) + (11); /* 115200 */
    //USART2->BRR |= (104 << 4) + (3); /* 9600 */
    
    /* 6. Set the TE bit in USART_CR1 to send an idle frame as first transmission. */
    USART2->CR1 |= USART_CR1_TE;
    
    /* 7. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
          for each data to be transmitted in case of single buffer. */
//    while(!(USART2->SR & USART_SR_TC));
//    USART2->DR = 'T';
//    while(!(USART2->SR & USART_SR_TC));
//    USART2->DR = 'x';
//    while(!(USART2->SR & USART_SR_TC));
//    USART2->DR = 'D';
//    while(!(USART2->SR & USART_SR_TC));
//    USART2->DR = 0x0d;
//    while(!(USART2->SR & USART_SR_TC));
//    USART2->DR = 0x0a;
    
    /* 8. After writing the last data into the USART_DR register, wait until TC=1. This indicates
          that the transmission of the last frame is complete. This is required for instance when
          the USART is disabled or enters the Halt mode to avoid corrupting the last
          transmission. */
    while(!(USART2->SR & USART_SR_TC));
    
    /* Enable receiver */
    USART2->CR1 |= USART_CR1_RE;
    
}

uint8_t uart_receive(void)
{
    
    uint8_t uart_rxd;
//    uint16_t timeout_count = 0;
    
    while(!(USART2->SR & USART_SR_RXNE));
    
//    while(!(USART2->SR & USART_SR_RXNE) & (timeout_count <= 10000))
//    {
//        timeout_count++;
//    }
    
//    if(timeout_count < 10000)
//    {
//        uart_rxd = USART2->DR;
//    }
//    else
//    {
//        uart_rxd = 0x00;
//    }
    
    uart_rxd = USART2->DR;
    
    return uart_rxd;
    
}

void uart_transmit(uint8_t data)
{
    
    USART2->DR = data;
    while(!(USART2->SR & USART_SR_TC));
    
}

void blue_button_init(void)
{
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  /* Enable port a clock */
    GPIOA->MODER &= ~0x01;                /* PA0 is input */
    
}

uint8_t blue_button_read(void)
{
    
    uint8_t blue_button_data = 0;
    
    if (GPIOA->IDR & 0x00000001)
    {
        blue_button_data = 1;
    }
    else
    {
        blue_button_data = 0;
    }
    
    return blue_button_data;
    
}

uint8_t ascii_to_number(uint8_t ascii_input)
{
    
    uint8_t number_output = 0;
    
    switch (ascii_input)
    {
      
      case 'S':
      {
        break;
      }
      case '0':
      {
        number_output = 0x00;
        break;
      }
      case '1':
      {
        number_output = 0x01;
        break;
      }
      case '2':
      {
        number_output = 0x02;
        break;
      }
      case '3':
      {
        number_output = 0x03;
        break;
      }
      case '4':
      {
        number_output = 0x04;
        break;
      }
      case '5':
      {
        number_output = 0x05;
        break;
      }
      case '6':
      {
        number_output = 0x06;
        break;
      }
      case '7':
      {
        number_output = 0x07;
        break;
      }
      case '8':
      {
        number_output = 0x08;
        break;
      }
      case '9':
      {
        number_output = 0x09;
        break;
      }
      case 'A':
      {
        number_output = 0x0A;
        break;
      }
      case 'B':
      {
        number_output = 0x0B;
        break;
      }
      case 'C':
      {
        number_output = 0x0C;
        break;
      }
      case 'D':
      {
        number_output = 0x0D;
        break;
      }
      case 'E':
      {
        number_output = 0x0E;
        break;
      }
      case 'F':
      {
        number_output = 0x0F;
        break;
      }
      case 'a':
      {
        number_output = 0x0A;
        break;
      }
      case 'b':
      {
        number_output = 0x0B;
        break;
      }
      case 'c':
      {
        number_output = 0x0C;
        break;
      }
      case 'd':
      {
        number_output = 0x0D;
        break;
      }
      case 'e':
      {
        number_output = 0x0E;
        break;
      }
      case 'f':
      {
        number_output = 0x0F;
        break;
      }
      default:
      {
        break;
      }
  }
    
  return number_output;
    
}

int main(void) {
  
  //Process_Srec(&test_srec[0], srec_data);

  //Flash_Init();
  //Flash_Erase_Sector(2);
  //Flash_Write_x32(0x08008000, 0xDEADBEEF);
  //Flash_Deinit();


#if 1

    blue_button_init();
        
    if (blue_button_read())
        {
    
        uart_init();
        
        uart_transmit('B');
        uart_transmit('o');
        uart_transmit('o');
        uart_transmit('t');
        uart_transmit(' ');
        uart_transmit('M');
        uart_transmit('o');
        uart_transmit('d');
        uart_transmit('e');
        uart_transmit('!');
        uart_transmit(0x0d);
        uart_transmit(0x0a);
        
        uint8_t uart_receive_buffer[100];
        uint8_t uart_rxd_old = 0x00;
        uint8_t uart_rxd = 0x00;
        int i = 0;
        int j = 0;
        uint32_t flash_address = 0;
        uint32_t srec_data_count = 0;
        uint32_t data_count = 0;
        
        Flash_Init();
        Flash_Erase_Sector(1);
    
        /* Receive SREC while "S7" is reached */
        while((uart_receive_buffer[1] != '7') & (uart_receive_buffer[1] != '0'))
        {
            i = 0;
            uart_rxd_old = 0x00;
            uart_rxd = 0x00;
            /* Receive over UART while CR, LF is sent */
            do
            {
                uart_rxd_old = uart_rxd;
                uart_rxd = uart_receive();
                if((uart_rxd != 0x0d) & (uart_rxd != 0x0a))
                {
                    uart_receive_buffer[i] = uart_rxd;
                }
                //uart_transmit(uart_rxd);
                i++;
            }
            while(((uart_rxd_old != 0x0d) & (uart_rxd != 0x0a)));
            
            /* Writting SREC data to FLASH */
            if ((uart_receive_buffer[0] == 'S') | (uart_receive_buffer[0] == 's'))    /* A valid SREC data */
            {
                if (uart_receive_buffer[1] == 0x33)              /* A 32 bit addressing */
                {
                    srec_data_count = ((uart_receive_buffer[2] - 0x30) << 4) + (uart_receive_buffer[3] - 0x30);
                    data_count = srec_data_count - 5;
                    flash_address = (ascii_to_number(uart_receive_buffer[4])  << 28) +
                                    (ascii_to_number(uart_receive_buffer[5])  << 24) +
                                    (ascii_to_number(uart_receive_buffer[6])  << 20) +
                                    (ascii_to_number(uart_receive_buffer[7])  << 16) +
                                    (ascii_to_number(uart_receive_buffer[8])  << 12) +
                                    (ascii_to_number(uart_receive_buffer[9])  <<  8) +
                                    (ascii_to_number(uart_receive_buffer[10])  << 4) +
                                    (ascii_to_number(uart_receive_buffer[11])      );
    //                for(j = 0; j < data_count/4; j++)
    //                {
    //                    Flash_Write_x32(flash_address + j, ((uart_receive_buffer[12 + j*8] - 0x30) << 28) +
    //                                                       ((uart_receive_buffer[13 + j*8] - 0x30) << 24) +
    //                                                       ((uart_receive_buffer[14 + j*8] - 0x30) << 20) +
    //                                                       ((uart_receive_buffer[15 + j*8] - 0x30) << 16) +
    //                                                       ((uart_receive_buffer[16 + j*8] - 0x30) << 12) +
    //                                                       ((uart_receive_buffer[17 + j*8] - 0x30) <<  8) +
    //                                                       ((uart_receive_buffer[18 + j*8] - 0x30) <<  4) +
    //                                                       ((uart_receive_buffer[19 + j*8] - 0x30)      ));
    //                }
                        Flash_Write_x32(flash_address,        (ascii_to_number(uart_receive_buffer[12]) <<  4) +
                                                              (ascii_to_number(uart_receive_buffer[13]) <<  0) +
                                                              (ascii_to_number(uart_receive_buffer[14]) << 12) +
                                                              (ascii_to_number(uart_receive_buffer[15]) <<  8) +
                                                              (ascii_to_number(uart_receive_buffer[16]) << 20) +
                                                              (ascii_to_number(uart_receive_buffer[17]) << 16) +
                                                              (ascii_to_number(uart_receive_buffer[18]) << 28) +
                                                              (ascii_to_number(uart_receive_buffer[19]) << 24));
                        Flash_Write_x32(flash_address + 0x04, (ascii_to_number(uart_receive_buffer[20]) <<  4) +
                                                              (ascii_to_number(uart_receive_buffer[21]) <<  0) +
                                                              (ascii_to_number(uart_receive_buffer[22]) << 12) +
                                                              (ascii_to_number(uart_receive_buffer[23]) <<  8) +
                                                              (ascii_to_number(uart_receive_buffer[24]) << 20) +
                                                              (ascii_to_number(uart_receive_buffer[25]) << 16) +
                                                              (ascii_to_number(uart_receive_buffer[26]) << 28) +
                                                              (ascii_to_number(uart_receive_buffer[27]) << 24));
                        Flash_Write_x32(flash_address + 0x08, (ascii_to_number(uart_receive_buffer[28]) <<  4) +
                                                              (ascii_to_number(uart_receive_buffer[29]) <<  0) +
                                                              (ascii_to_number(uart_receive_buffer[30]) << 12) +
                                                              (ascii_to_number(uart_receive_buffer[31]) <<  8) +
                                                              (ascii_to_number(uart_receive_buffer[32]) << 20) +
                                                              (ascii_to_number(uart_receive_buffer[33]) << 16) +
                                                              (ascii_to_number(uart_receive_buffer[34]) << 28) +
                                                              (ascii_to_number(uart_receive_buffer[35]) << 24));
                        Flash_Write_x32(flash_address + 0x0C, (ascii_to_number(uart_receive_buffer[36]) <<  4) +
                                                              (ascii_to_number(uart_receive_buffer[37]) <<  0) +
                                                              (ascii_to_number(uart_receive_buffer[38]) << 12) +
                                                              (ascii_to_number(uart_receive_buffer[39]) <<  8) +
                                                              (ascii_to_number(uart_receive_buffer[40]) << 20) +
                                                              (ascii_to_number(uart_receive_buffer[41]) << 16) +
                                                              (ascii_to_number(uart_receive_buffer[42]) << 28) +
                                                              (ascii_to_number(uart_receive_buffer[43]) << 24));
                }
            }
        }
        
        uart_transmit('F');
        uart_transmit('l');
        uart_transmit('a');
        uart_transmit('s');
        uart_transmit('h');
        uart_transmit(' ');
        uart_transmit('W');
        uart_transmit('r');
        uart_transmit('i');
        uart_transmit('t');
        uart_transmit('e');
        uart_transmit(' ');
        uart_transmit('D');
        uart_transmit('o');
        uart_transmit('n');
        uart_transmit('e');
        uart_transmit('!');
        uart_transmit(0x0d);
        uart_transmit(0x0a);
            
        Flash_Deinit();
        
        uart_transmit('F');
        uart_transmit('l');
        uart_transmit('a');
        uart_transmit('s');
        uart_transmit('h');
        uart_transmit(' ');
        uart_transmit('D');
        uart_transmit('e');
        uart_transmit('i');
        uart_transmit('n');
        uart_transmit('i');
        uart_transmit('t');
        uart_transmit(' ');
        uart_transmit('D');
        uart_transmit('o');
        uart_transmit('n');
        uart_transmit('e');
        uart_transmit('!');
        uart_transmit(0x0d);
        uart_transmit(0x0a);
        
    }
        
    

#endif    
  
//  uncomment to enable firmware flashing
#if 0
  Flash_Init();
  Flash_Erase_Sector(1);
  for (int records_number = 0; records_number < 42; records_number++)
  {
    
    //Process_Srec(&app_srec_red[0 + (records_number * 48)], srec_data);
    Process_Srec(&app_srec_blue[0 + (records_number * 48)], srec_data);
    
    for (int i = 0; i < 4; i++)
    {
      Flash_Write_x32(0x08004000 + (i * 4) + (0x10 * records_number), (srec_data[3 + (i * 4)] << 24) +
                                                                      (srec_data[2 + (i * 4)] << 16) +
                                                                      (srec_data[1 + (i * 4)] <<  8) +
                                                                      (srec_data[0 + (i * 4)]      ));
    }
    
  }
  
  Flash_Deinit();
#endif
  
  BootJump((uint32_t*)USER_APPLICATION_BASE_ADDRESS);
  
}
