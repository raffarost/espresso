#include <stdio.h>
#include <stdlib.h>

float indexRatio(float vector[], int size, float input)
{
    int i;
    float out;

    for (i=0; i < size; i++)
    {
        /* search for first element greater than input */
        if (vector[i] > input)
        {
            break;
        }
    }

    if (i > 0)
    {
        /* we have two points for linear interpolation */
        out = ((i - 1) + ((input - vector[i-1]) / (vector[i] - vector[i-1])));    /* index + fraction */
    }
    else
    {
        out = 0;    /* first element is already bigger and we won't extrapolate */
    }

    return out;
}

float interp1D(float vector[], int size, float ir)
{
    int index = (int)ir;
    float ratio = (ir - index);
    float out;

    if (index < (size-2))
    {
        out = ((vector[index+1] - vector[index]) * ratio) + vector[index];
    }
    else
    {
        out = vector[size-1];   /* saturate and return last cell */
    }

    return out;
}
