#include "ExtendedKalmanFilter.h"
#include "AP_Math/matrixN.h"
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <fenv.h>
#endif
#define EKF_FAST_MATH

#ifdef EKF_FAST_MATH

static union
{
	float d;
	int i;
} _eco;
#define EXP_A 12102203 /* int(1<<23/math.log(2)) */
#define EXP_C 0 /* see text for choice of c values */

// Adapted from Schraudolph, "A Fast, Compact Approximation if the Exponential Function",
// Tech Report UDSIA-07-98
#define fastexp(y) (_eco.i = EXP_A*(y)+(1065353216 - EXP_C), _eco.d)
/*
in the above fastexp macro:
values of x around -88 to -89 can result in NaN,
values below about -89 are not valid:
x                                    hex value                                 hex value
-88.0    exp(x) = 6.0546014852e-39   41edc4      fastexp(x) = 5.0357061614e-40     57bc0
-88.5    exp(x) = 3.6723016101e-39   27fce2      fastexp(x) = NaN               ffa92680
-89.0    exp(x) = 2.2273639090e-39   1840fc      fastexp(x) = -2.7225029733e+38 ff4cd180
so we check that x is than 88 to avoid this.
(Note we also assume here that x is always negative, which is the case when used in a gaussian)
*/

#define EXP(x) ( (x) > -88.0f ? fastexp(x) : 0.0 )
#else
#define EXP(x) expf(x)
#endif

extern const AP_HAL::HAL& hal;

float ExtendedKalmanFilter::measurementpredandjacobian(VectorN<float,EKF_N> &A)
{
    // This function computes the Jacobian using equations from
    // analytical derivation of Gaussian updraft distribution
    // This expression gets used lots
	float x1_2 = X[1] * X[1];
	float x2_2 = X[2] * X[2];
	float x3_2 = X[3] * X[3];
    float expon = EXP(- (x2_2 + x3_2) / x1_2);
    // Expected measurement
    float w = X[0] * expon;

    // Elements of the Jacobian
    A[0] = expon;
    A[1] = 2 * X[0] * ((x2_2 + x3_2) / (x1_2 * X[1])) * expon;
    A[2] = -2 * (X[0] * X[2] / x1_2) * expon;
    A[3] = -2 * (X[0] * X[3] / x1_2) * expon; //A[2] * X[3] / X[2];
    return w;
}

float ExtendedKalmanFilter::prediction() {
    _z = X[0] * EXP(- (powf(X[2], 2) + powf(X[3], 2)) / powf(X[1], 2));
    return _z;
}

void ExtendedKalmanFilter::reset(const VectorN<float,EKF_N> &x, const MatrixN<float,EKF_N> &p, const MatrixN<float,EKF_N> q, float r)
{
    P = p;
    X = x;
    Q = q;
    R = r;
}


void ExtendedKalmanFilter::update(float z, float Vx, float Vy)
{
    MatrixN<float, EKF_N> tempM;
    VectorN<float, EKF_N> H;
    VectorN<float, EKF_N> P12;
    VectorN<float, EKF_N> K;
    
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
	int old = fedisableexcept(FE_OVERFLOW);
	if (old < 0) {
		hal.console->printf("ExtendedKalmanFilter::update(): warning: error on disabling FE_OVERFLOW floating point exception\n");
	}
#endif

    // LINE 28
    // Estimate new state from old.
    X[2] -= Vx;
    X[3] -= Vy;

    // LINE 33
    // Update the covariance matrix
    // P = A*ekf.P*A'+ekf.Q;
    // We know A is identity so
    // P = ekf.P+ekf.Q;
    //P += Q;
	P(0, 0) += Q(0, 0);
	P(1, 1) += Q(1, 1);
	P(2, 2) += Q(2, 2);
	P(3, 3) += Q(3, 3);
    // What measurement do we expect to receive in the estimated
    // state
    // LINE 37
    // [z1,H] = ekf.jacobian_h(x1);
    float z1 = measurementpredandjacobian(H);

    // LINE 40
    // P12 = P * H';
    P12.mult(P, H); //cross covariance 
    
    // LINE 41
    // Calculate the KALMAN GAIN
    // K = P12 * inv(H*P12 + ekf.R);                     %Kalman filter gain
    K = P12 * 1.0 / (H * P12 + R);

    // Correct the state estimate using the measurement residual.
    // LINE 44
    // X = x1 + K * (z - z1);
    X += K * (z - z1);

    // Correct the covariance too.
    // LINE 46
    // NB should be altered to reflect Stengel
    // P = P_predict - K * P12';
    tempM.mult(K, P12);
    P -= tempM;
    
    P.force_symmetry();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
	if (old >= 0 && feenableexcept(old) < 0) {
		hal.console->printf("ExtendedKalmanFilter::update(): warning: error on restoring floating exception mask\n");
	}
#endif

}
