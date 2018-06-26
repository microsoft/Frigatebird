// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#include "WindExtendedKalmanFilter.h"
#include "AP_Math/matrixN.h"

float
WindExtendedKalmanFilter::measurementpredandjacobian(VectorN<float,WEKF_N> &A)
{
	return 0;
}


float
WindExtendedKalmanFilter::prediction()
{
	return _z;
}


void
WindExtendedKalmanFilter::reset(const VectorN<float, WEKF_N> &x, const MatrixN<float, WEKF_N> &p, const MatrixN<float, WEKF_N> &q, float r)
{
    P = p;
    X = x;
    Q = q;
    R = r;
}


void
WindExtendedKalmanFilter::update(float VN, float VE, float aspd_sensor)
{
    MatrixN<float, WEKF_N> tempM;
    VectorN<float, WEKF_N> H;
    VectorN<float, WEKF_N> P12;
    VectorN<float, WEKF_N> K;
    
	float N_err = VN - X[1]; // X[1] = WN
	float E_err = VE - X[2]; // X[2] = WE
	float Va = sqrtf(N_err * N_err + E_err * E_err);
	float aspd_estimate = Va - X[0]; // X[0] = aspd_sensor_bias
	float err = aspd_sensor - aspd_estimate;
	_z = aspd_estimate;
	P(0, 0) += Q(0, 0);
	P(1, 1) += Q(1, 1);
	P(2, 2) += Q(2, 2);
	P(3, 3) += Q(3, 3);

	if (Va > 0)
    {
		H[0] = -1;
		H[1] = -N_err / Va;
		H[2] = -E_err / Va;
	}

    // cross covariance 
    P12.mult(P, H);
    
    // Calculate the KALMAN GAIN
    K = P12 * 1.0 / (H * P12 + R);

    // Correct the state estimate using the measurement residual.
    X += K * err;

    // Correct the covariance too.
    tempM.mult(K, P12);
    P -= tempM;
    
    P.force_symmetry();
}
