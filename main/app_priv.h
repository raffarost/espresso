/*
   Include file only

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once
#include <stdint.h>
#include <stdbool.h>

extern void pidInit(void);
extern float pidUpdate(float setpoint, float measurement);
void dimInit(void);
void gpioConfig(void);

extern float indexRatio(float vector[], int size, float input);
extern float interp1D(float vector[], int size, float ir);
extern unsigned long long getAbsTime1us(void);

/** Boiler MAX6675 path: true after several consecutive good samples; false on any SPI/open/range fault. */
bool boiler_temp_is_trusted(void);
/** Latched fault bits (TEMP_DIAG_*); cleared when read becomes fully trusted again. */
uint32_t boiler_temp_fault_bits(void);
