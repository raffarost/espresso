/*
   Espresso — NVS persistence API
*/
#pragma once

#include "control.h"    /* for ADAPTIVE_WARMUP_ENABLE */

void nvsRead(void);
void nvsWrite(void);

#if ADAPTIVE_WARMUP_ENABLE
void nvs_persist_dither_step(void);
#endif
