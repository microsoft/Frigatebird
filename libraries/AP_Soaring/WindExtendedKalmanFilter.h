// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

/*
* Wind Extended Kalman Filter class based on the EKF class by Sam Tabor, 2013.
* Based on EKF wind estimate described by J. Edwards
*/

#pragma once

#include <AP_Math/matrixN.h>

#define WEKF_N 3


class WindExtendedKalmanFilter {
public:
    WindExtendedKalmanFilter(void) {}
    
    VectorN<float, WEKF_N> X;
    MatrixN<float, WEKF_N> P;
    MatrixN<float, WEKF_N> Q;
    float R;
    void reset(const VectorN<float, WEKF_N> &x, const MatrixN<float, WEKF_N> &p, const MatrixN<float, WEKF_N> &q, float r);
    void update(float VN, float VE, float aspd_senso);
    float prediction();

private:
    float _z;
    float measurementpredandjacobian(VectorN<float, WEKF_N> &A);
};
