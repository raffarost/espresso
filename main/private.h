/*
   Espresso — internal shared declarations
*/
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_rmaker_core.h"

extern void pidInit(void);
extern float pidUpdate(float setpoint, float measurement);
void dimInit(void);
void gpioConfig(void);

extern float indexRatio(float vector[], int size, float input);
extern float interp1D(float vector[], int size, float ir);
extern unsigned long long getAbsTime1us(void);

void rainmaker_init(esp_rmaker_node_t *node);

bool boiler_temp_is_trusted(void);
uint32_t boiler_temp_fault_bits(void);
