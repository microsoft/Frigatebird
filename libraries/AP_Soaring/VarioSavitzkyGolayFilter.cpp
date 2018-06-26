// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#include "VarioSavitzkyGolayFilter.h"

void
VarioSavitzkyGolayFilter::prediction(float dt, const float **ekf_buffer, unsigned ekf_buffer_size, unsigned ekf_buffer_ptr, float *Edot, float *Edotdot)
{
    float z2 = _c_z*_c_z;
    float z3 = z2 * _c_z;
    int k = ekf_buffer_ptr - _c_m;
    if (k < 0) k += ekf_buffer_size;
    *Edot = 0;
    *Edotdot = 0;
    float p1 = 0;
    float p2 = 0;
    float p3 = 0;
    float p4 = 0;

    for (int i = 0; i < _c_m; i++)
    {
        p1 += SG_FILTER_C[0][i] * ekf_buffer[k][0];
        p2 += SG_FILTER_C[1][i] * ekf_buffer[k][0];
        p3 += SG_FILTER_C[2][i] * ekf_buffer[k][0];
        p4 += SG_FILTER_C[3][i] * ekf_buffer[k][0];
        k = (k + 1) % ekf_buffer_size;
    }

    *Edot = p1 + p2 * _c_z + p3 * z2 + p4 * z3;
    *Edotdot = p2 + 2 * p3 * _c_z + 3 * p4 * z2;
    *Edotdot = *Edotdot / dt;
}
