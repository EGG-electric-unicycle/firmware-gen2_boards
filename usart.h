#ifndef _USART_H_
#define _USART_H_

unsigned char usart1_send_char (unsigned char c);
void usart1_send_str (unsigned char *data);
unsigned char usart1_receive_char (void);

#endif
