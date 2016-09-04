/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _USART_H_
#define _USART_H_

void usart1_bluetooth_init(void);
unsigned char usart1_send_char (unsigned char c);
void usart1_send_str (unsigned char *data);
unsigned char usart1_receive_char (void);

#endif
