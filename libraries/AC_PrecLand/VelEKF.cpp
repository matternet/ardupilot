#include "VelEKF.h"
#include <math.h>
#include <string.h>

#define VELEKF_POS_CALC_NIS(__P, __R, __X, __Z, __RET_NIS) \
__RET_NIS = ((-__X[0] + __Z)*(-__X[0] + __Z))/(__P[0] + __R);

#define VELEKF_POS_CALC_STATE(__P, __R, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __P[0]*(-__X[0] + __Z)/(__P[0] + __R) + __X[0]; __RET_STATE[1] = __P[1]*(-__X[0] + \
__Z)/(__P[0] + __R) + __X[1];

#define VELEKF_POS_CALC_COV(__P, __R, __X, __Z, __RET_COV) \
__RET_COV[0] = ((__P[0])*(__P[0]))*__R/((__P[0] + __R)*(__P[0] + __R)) + __P[0]*((-__P[0]/(__P[0] + \
__R) + 1)*(-__P[0]/(__P[0] + __R) + 1)); __RET_COV[1] = __P[0]*__P[1]*__R/((__P[0] + __R)*(__P[0] + \
__R)) - __P[0]*__P[1]*(-__P[0]/(__P[0] + __R) + 1)/(__P[0] + __R) + __P[1]*(-__P[0]/(__P[0] + __R) + \
1); __RET_COV[2] = ((__P[1])*(__P[1]))*__R/((__P[0] + __R)*(__P[0] + __R)) - \
((__P[1])*(__P[1]))/(__P[0] + __R) - __P[1]*(-__P[0]*__P[1]/(__P[0] + __R) + __P[1])/(__P[0] + __R) + \
__P[2];

#define VELEKF_PREDICTION_CALC_STATE(__P, __DT, __DV, __DV_NOISE, __X, __RET_STATE) \
__RET_STATE[0] = __DT*__X[1] + __X[0]; __RET_STATE[1] = __DV + __X[1];

#define VELEKF_PREDICTION_CALC_COV(__P, __DT, __DV, __DV_NOISE, __X, __RET_COV) \
__RET_COV[0] = __DT*__P[1] + __DT*(__DT*__P[2] + __P[1]) + __P[0]; __RET_COV[1] = __DT*__P[2] + \
__P[1]; __RET_COV[2] = ((__DV_NOISE)*(__DV_NOISE)) + __P[2];

#define VELEKF_VEL_CALC_NIS(__P, __R, __X, __Z, __RET_NIS) \
__RET_NIS = ((-__X[1] + __Z)*(-__X[1] + __Z))/(__P[2] + __R);

#define VELEKF_VEL_CALC_STATE(__P, __R, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __P[1]*(-__X[1] + __Z)/(__P[2] + __R) + __X[0]; __RET_STATE[1] = __P[2]*(-__X[1] + \
__Z)/(__P[2] + __R) + __X[1];

#define VELEKF_VEL_CALC_COV(__P, __R, __X, __Z, __RET_COV) \
__RET_COV[0] = __P[0] + ((__P[1])*(__P[1]))*__R/((__P[2] + __R)*(__P[2] + __R)) - \
((__P[1])*(__P[1]))/(__P[2] + __R) - __P[1]*(-__P[1]*__P[2]/(__P[2] + __R) + __P[1])/(__P[2] + __R); \
__RET_COV[1] = __P[1]*__P[2]*__R/((__P[2] + __R)*(__P[2] + __R)) + (-__P[2]/(__P[2] + __R) + \
1)*(-__P[1]*__P[2]/(__P[2] + __R) + __P[1]); __RET_COV[2] = ((__P[2])*(__P[2]))*__R/((__P[2] + \
__R)*(__P[2] + __R)) + __P[2]*((-__P[2]/(__P[2] + __R) + 1)*(-__P[2]/(__P[2] + __R) + 1));

void VelEKF::init(float pos, float posVar, float vel, float velVar)
{
    _state[0] = pos;
    _state[1] = pos;
    _cov[0] = posVar;
    _cov[1] = 0.0f;
    _cov[2] = velVar;
}

void VelEKF::predict(float dt, float dVel, float dVelNoise)
{
    float newState[2];
    float newCov[3];

    VELEKF_PREDICTION_CALC_STATE(_cov, dt, dVel, dVelNoise, _state, newState)
    VELEKF_PREDICTION_CALC_COV(_cov, dt, dVel, dVelNoise, _state, newCov)

    memcpy(_state,newState,sizeof(_state));
    memcpy(_cov,newCov,sizeof(_cov));
}

void VelEKF::fusePos(float pos, float posVar)
{
    float newState[2];
    float newCov[3];

    VELEKF_POS_CALC_STATE(_cov, posVar, _state, pos, newState)
    VELEKF_POS_CALC_COV(_cov, posVar, _state, pos, newCov)

    memcpy(_state,newState,sizeof(_state));
    memcpy(_cov,newCov,sizeof(_cov));
}

void VelEKF::fuseVel(float vel, float velVar)
{
    float newState[2];
    float newCov[3];

    VELEKF_VEL_CALC_STATE(_cov, velVar, _state, vel, newState)
    VELEKF_VEL_CALC_COV(_cov, velVar, _state, vel, newCov)

    memcpy(_state,newState,sizeof(_state));
    memcpy(_cov,newCov,sizeof(_cov));
}
