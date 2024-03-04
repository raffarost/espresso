#include <stdio.h>
#include <stdlib.h>

#include "PID.h"

/* Controller parameters */
#define PID_KP  4.0f
#define PID_KI  2.0f
#define PID_KD  1.0f

#define PID_TAU 0.02f

#define PID_LIM_MIN    0.0f
#define PID_LIM_MAX  100.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.5f

/* PID controller constants init */
PIDController pid = { PID_KP, PID_KI, PID_KD,
                        PID_TAU,
                        PID_LIM_MIN, PID_LIM_MAX,
                        PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                        SAMPLE_TIME_S };

void pidInit(void)
{
    PIDController_Init(&pid);
}

float pidUpdate(float setpoint, float measurement)
{
    /* Compute new control signal */
    PIDController_Update(&pid, setpoint, measurement);

    return (pid.out);
}
