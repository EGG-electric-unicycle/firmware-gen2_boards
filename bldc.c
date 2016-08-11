/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "gpio.h"
#include "bldc.h"
#include "pwm.h"

extern GPIO_InitTypeDef GPIO_InitStructure;

#define HALL_SENSORS_MASK ((1 << 0) | (1 << 1) | (1 << 2))

unsigned int bldc_machine_state = BLDC_NORMAL;

static unsigned int _direction = RIGHT;

struct Bldc_phase_state bldc_phase_state;

void phase_a_h_off (void)
{
  GPIO_InitStructure.GPIO_Pin = BRIDGE_A_HIGH;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, BRIDGE_A_HIGH);
}

void phase_a_l_off (void)
{
  GPIO_InitStructure.GPIO_Pin = BRIDGE_A_LOW;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, BRIDGE_A_LOW);
}

void phase_b_h_off (void)
{
  GPIO_InitStructure.GPIO_Pin = BRIDGE_B_HIGH;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, BRIDGE_B_HIGH);
}

void phase_b_l_off (void)
{
  GPIO_InitStructure.GPIO_Pin = BRIDGE_B_LOW;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, BRIDGE_B_LOW);
}

void phase_c_h_off (void)
{
  GPIO_InitStructure.GPIO_Pin = BRIDGE_C_HIGH;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, BRIDGE_C_HIGH);
}

void phase_c_l_off (void)
{
  GPIO_InitStructure.GPIO_Pin = BRIDGE_C_LOW;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, BRIDGE_C_LOW);
}

// update duty-cycle controller with phase new states
void apply_duty_cycle (void)
{
  TIM_UpdateDisableConfig (TIM1, ENABLE); // disable update event
  pwm_update_duty_cycle ();
  TIM_GenerateEvent(TIM1, TIM_EventSource_Update); // generate update event to update shadow registers / duty-cycle
  TIM_UpdateDisableConfig (TIM1, DISABLE); // enable update event
}

void commutation_AB (void)
{
  bldc_phase_state.a = NORMAL;
  bldc_phase_state.b = INVERTED;
  bldc_phase_state.c = OFF;

  apply_duty_cycle (); // update duty-cycle controller with phase new states
}

void commutation_AC (void)
{
  bldc_phase_state.a = NORMAL;
  bldc_phase_state.b = OFF;
  bldc_phase_state.c = INVERTED;

  apply_duty_cycle ();
}

void commutation_BC (void)
{
  bldc_phase_state.a = OFF;
  bldc_phase_state.b = NORMAL;
  bldc_phase_state.c = INVERTED;

  apply_duty_cycle ();
}

void commutation_BA (void)
{
  bldc_phase_state.a = INVERTED;
  bldc_phase_state.b = NORMAL;
  bldc_phase_state.c = OFF;

  apply_duty_cycle ();
}

void commutation_CA (void)
{
  bldc_phase_state.a = INVERTED;
  bldc_phase_state.b = OFF;
  bldc_phase_state.c = NORMAL;

  apply_duty_cycle ();
}

void commutation_CB (void)
{
  bldc_phase_state.a = OFF;
  bldc_phase_state.b = INVERTED;
  bldc_phase_state.c = NORMAL;

  apply_duty_cycle ();
}

void commutation_disable (void)
{
  TIM_CtrlPWMOutputs (TIM1, DISABLE); // PWM Output Disable
}

unsigned int get_current_sector (void)
{
  static unsigned int hall_sensors = 0;
  unsigned int sector;
  unsigned int i;

  hall_sensors = (GPIO_ReadInputData (GPIOA) & (HALL_SENSORS_MASK)); // mask other pins

  // IDENTIFY the sector from hall sensors signals
  //
  //       cba
  //  00000001 == 1
  //  00000011 == 3
  //  00000010 == 2
  //  00000110 == 6
  //  00000100 == 4
  //  00000101 == 5

  //Halls sequence: 1, 3, 2, 6, 4, 5
  switch (hall_sensors)
  {
    case 4:
    sector = 2;
    break;

    case 5:
    sector = 3;
    break;

    case 1:
    sector = 4;
    break;

    case 3:
    sector = 5;
    break;

    case 2:
    sector = 6;
    break;

    case 6:
    sector = 1;
    break;

    default:
    break;
  }

  if (_direction == RIGHT)
  {
    sector = increment_sector(sector);
  }
  else if (_direction == LEFT)
  {
    sector = decrement_sector(sector);
  }

  return sector;
}

void commutate (void)
{
  volatile unsigned int sector;

  sector = get_current_sector ();

  //Coils: AB, AC, BC, BA, CA, CB
  switch (sector)
  {
    case 1:
    commutation_AB ();
    break;

    case 2:
    commutation_AC ();
    break;

    case 3:
    commutation_BC ();
    break;

    case 4:
    commutation_BA ();
    break;

    case 5:
    commutation_CA ();
    break;

    case 6:
    commutation_CB ();
    break;

    default:
    commutation_disable ();
    break;
  }
}

void commutate_sector (void)
{
  static unsigned int sector = 1;

  sector = increment_sector (sector);

  //Coils: AB, AC, BC, BA, CA, CB
  switch (sector)
  {
    case 1:
    commutation_AB ();
    break;

    case 2:
    commutation_AC ();
    break;

    case 3:
    commutation_BC ();
    break;

    case 4:
    commutation_BA ();
    break;

    case 5:
    commutation_CA ();
    break;

    case 6:
    commutation_CB ();
    break;

    default:
    commutation_disable ();
    break;
  }
}

unsigned int increment_sector (unsigned int sector)
{
  if (sector < 6)
  {
    sector++;
  }
  else // sector = 6
  {
    sector = 1;
  }

  return sector;
}

unsigned int decrement_sector (unsigned int sector)
{
  if (sector > 1)
  {
    sector--;
  }
  else // sector = 1
  {
    sector = 6;
  }

  return sector;
}

void set_direction (unsigned int direction)
{
  _direction = direction;
}

unsigned int get_direction (void)
{
  return _direction;
}

void bldc_set_state (unsigned int state)
{
  bldc_machine_state = state;
}

unsigned int bldc_get_state (void)
{
  return bldc_machine_state;
}
