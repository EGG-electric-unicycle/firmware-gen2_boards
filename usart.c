/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f10x_usart.h"

char tx_buffer[TX_LEN];
char rx_buffer[RX_LEN];
unsigned int tx_i = 0; // start at first byte
unsigned int tx_len = 0;
unsigned int rx_i = 0; // start at first byte
unsigned int rx_len = 0;

void usart1_bluetooth_init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


  GPIO_InitTypeDef GPIO_InitStructure;

  // USART for bluetooth module pins
  GPIO_InitStructure.GPIO_Pin = USART_RX__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART_RX__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = USART_TX__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USART_TX__PORT, &GPIO_InitStructure);

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

  USART_DeInit(USART1);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  /* Enable USART1 Receive and Transmit interrupts */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  USART_ClearITPendingBit(USART1, USART_IT_TXE);

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);
}

// USART1 Tx and Rx interrupt handler.
void USART1_IRQHandler()
{
  // The interrupt may be from Tx, Rx, or both.
  if (USART_GetITStatus(USART1, USART_IT_ORE) == SET)
  {
    USART_ReceiveData(USART1); // get ride of this interrupt flag
    return;
  }
  else if (USART_GetITStatus(USART1, USART_IT_TXE) == SET)
  {
    // If the Tx index is less than the Tx packet length...
    if(tx_i < (tx_len - 1))
    {
      // ...increment and transmit the next byte.
      tx_i++;
      USART_SendData (USART1, tx_buffer[tx_i]);
    }
    else
    {
      // ...disable Tx interrupt until next packet.
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
      tx_i = 0; // reset TX buffer index
    }
  }
  else if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
  {
    // Triggers EVERY time a byte is received on the UART
    unsigned char rx_byte;
    rx_byte = USART1->DR;    // read byte

    if (rx_byte == '\r') return;

    // buffer should be full or there is data on the buffer waiting to be processed by _read()
    // so discard the received data and return - we should not get to this condition
    if ((rx_i >= RX_LEN) || (rx_len > 0))
    {
      if (rx_i >= RX_LEN) // buffer is full and didn't received \n, so reset the buffer and start again
      {
	rx_len = 0;
	rx_i = 0;
      }

      return;
    }

    rx_buffer[rx_i] = rx_byte;
    if (rx_buffer[rx_i] == '\n') // end of line/data command
    {
      rx_len = rx_i + 1; // scanf() / _read() should now read the data from rx_buffer[] and empty it
    }
    rx_i++;

    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}

void usart1_send_data ()
{
  while (tx_i) ;
  // Start transmission.
  USART_SendData (USART1, tx_buffer[tx_i]);
  // Enable Tx interrupt.
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

unsigned char usart1_send_char (unsigned char c)
{
  while (USART_GetFlagStatus (USART1, USART_FLAG_TXE) == RESET) ;

  USART_SendData (USART1, c);

  return (c);
}

void usart1_send_str (unsigned char *data)
{
  unsigned char i = 0, r;

  while ((r = data[i++]))
    usart1_send_char (r);
}

unsigned char usart1_receive_char (void)
{
  while (USART_GetFlagStatus (USART1, USART_FLAG_RXNE) == RESET) ;

  return (USART_ReceiveData (USART1));
}
