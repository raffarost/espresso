/*
   Espresso — NVS persistence API
*/
#pragma once

#include "control.h"    /* for OVERSHOOT_DETECT_ENABLE */

void nvsRead(void);
void nvsWrite(void);

#if OVERSHOOT_DETECT_ENABLE
void nvs_persist_overshoot_trim(void);
#endif
