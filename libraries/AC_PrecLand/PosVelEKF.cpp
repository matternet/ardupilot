#include "PosVelEKF.h"
#include <math.h>
#include <string.h>

#define POSVELEKF_POS_CALC_NIS(__P, __R, __X, __Z, __RET_NIS) \
__RET_NIS = ((-__X[0] + __Z)*(-__X[0] + __Z))/(__P[0] + __R);

#define POSVELEKF_POS_CALC_STATE(__P, __R, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __P[0]*(-__X[0] + __Z)/(__P[0] + __R) + __X[0]; __RET_STATE[1] = __P[1]*(-__X[0] + \
__Z)/(__P[0] + __R) + __X[1]; __RET_STATE[2] = __P[2]*(-__X[0] + __Z)/(__P[0] + __R) + __X[2];

#define POSVELEKF_POS_CALC_COV(__P, __R, __X, __Z, __RET_COV) \
__RET_COV[0] = ((__P[0])*(__P[0]))*__R/((__P[0] + __R)*(__P[0] + __R)) + __P[0]*((-__P[0]/(__P[0] + \
__R) + 1)*(-__P[0]/(__P[0] + __R) + 1)); __RET_COV[1] = __P[0]*__P[1]*__R/((__P[0] + __R)*(__P[0] + \
__R)) - __P[0]*__P[1]*(-__P[0]/(__P[0] + __R) + 1)/(__P[0] + __R) + __P[1]*(-__P[0]/(__P[0] + __R) + \
1); __RET_COV[2] = __P[0]*__P[2]*__R/((__P[0] + __R)*(__P[0] + __R)) - __P[0]*__P[2]*(-__P[0]/(__P[0] \
+ __R) + 1)/(__P[0] + __R) + __P[2]*(-__P[0]/(__P[0] + __R) + 1); __RET_COV[3] = \
((__P[1])*(__P[1]))*__R/((__P[0] + __R)*(__P[0] + __R)) - ((__P[1])*(__P[1]))/(__P[0] + __R) - \
__P[1]*(-__P[0]*__P[1]/(__P[0] + __R) + __P[1])/(__P[0] + __R) + __P[3]; __RET_COV[4] = \
__P[1]*__P[2]*__R/((__P[0] + __R)*(__P[0] + __R)) - __P[1]*__P[2]/(__P[0] + __R) - \
__P[2]*(-__P[0]*__P[1]/(__P[0] + __R) + __P[1])/(__P[0] + __R) + __P[4]; __RET_COV[5] = \
((__P[2])*(__P[2]))*__R/((__P[0] + __R)*(__P[0] + __R)) - ((__P[2])*(__P[2]))/(__P[0] + __R) - \
__P[2]*(-__P[0]*__P[2]/(__P[0] + __R) + __P[2])/(__P[0] + __R) + __P[5];

#define POSVELEKF_PREDICTION_CALC_STATE(__P, __ABIAS_PNOISE, __DT, __DV, __DV_NOISE, __X, __RET_STATE) \
__RET_STATE[0] = __DT*__X[1] + __X[0]; __RET_STATE[1] = -__DT*__X[2] + __DV + __X[1]; __RET_STATE[2] \
= __X[2];

#define POSVELEKF_PREDICTION_CALC_COV(__P, __ABIAS_PNOISE, __DT, __DV, __DV_NOISE, __X, __RET_COV) \
__RET_COV[0] = __DT*__P[1] + __DT*(__DT*__P[3] + __P[1]) + __P[0]; __RET_COV[1] = __DT*__P[3] - \
__DT*(__DT*__P[4] + __P[2]) + __P[1]; __RET_COV[2] = __DT*__P[4] + __P[2]; __RET_COV[3] = \
-__DT*__P[4] - __DT*(-__DT*__P[5] + __P[4]) + ((__DV_NOISE)*(__DV_NOISE)) + __P[3]; __RET_COV[4] = \
-__DT*__P[5] + __P[4]; __RET_COV[5] = ((__ABIAS_PNOISE)*(__ABIAS_PNOISE))*((__DT)*(__DT)) + __P[5];

#define POSVELEKF_VEL_CALC_NIS(__P, __R, __X, __Z, __RET_NIS) \
__RET_NIS = ((-__X[1] + __Z)*(-__X[1] + __Z))/(__P[3] + __R);

#define POSVELEKF_VEL_CALC_STATE(__P, __R, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __P[1]*(-__X[1] + __Z)/(__P[3] + __R) + __X[0]; __RET_STATE[1] = __P[3]*(-__X[1] + \
__Z)/(__P[3] + __R) + __X[1]; __RET_STATE[2] = __P[4]*(-__X[1] + __Z)/(__P[3] + __R) + __X[2];

#define POSVELEKF_VEL_CALC_COV(__P, __R, __X, __Z, __RET_COV) \
__RET_COV[0] = __P[0] + ((__P[1])*(__P[1]))*__R/((__P[3] + __R)*(__P[3] + __R)) - \
((__P[1])*(__P[1]))/(__P[3] + __R) - __P[1]*(-__P[1]*__P[3]/(__P[3] + __R) + __P[1])/(__P[3] + __R); \
__RET_COV[1] = __P[1]*__P[3]*__R/((__P[3] + __R)*(__P[3] + __R)) + (-__P[3]/(__P[3] + __R) + \
1)*(-__P[1]*__P[3]/(__P[3] + __R) + __P[1]); __RET_COV[2] = __P[1]*__P[4]*__R/((__P[3] + __R)*(__P[3] \
+ __R)) - __P[1]*__P[4]/(__P[3] + __R) + __P[2] - __P[4]*(-__P[1]*__P[3]/(__P[3] + __R) + \
__P[1])/(__P[3] + __R); __RET_COV[3] = ((__P[3])*(__P[3]))*__R/((__P[3] + __R)*(__P[3] + __R)) + \
__P[3]*((-__P[3]/(__P[3] + __R) + 1)*(-__P[3]/(__P[3] + __R) + 1)); __RET_COV[4] = \
__P[3]*__P[4]*__R/((__P[3] + __R)*(__P[3] + __R)) - __P[3]*__P[4]*(-__P[3]/(__P[3] + __R) + \
1)/(__P[3] + __R) + __P[4]*(-__P[3]/(__P[3] + __R) + 1); __RET_COV[5] = \
((__P[4])*(__P[4]))*__R/((__P[3] + __R)*(__P[3] + __R)) - ((__P[4])*(__P[4]))/(__P[3] + __R) - \
__P[4]*(-__P[3]*__P[4]/(__P[3] + __R) + __P[4])/(__P[3] + __R) + __P[5];

void PosVelEKF::init(float pos, float posVar, float vel, float velVar)
{
    uint8_t next_state_idx = (_state_idx+1)%2;

    _state[next_state_idx].x[0] = pos;
    _state[next_state_idx].x[1] = vel;
    _state[next_state_idx].x[2] = 0;
    _state[next_state_idx].P[0] = posVar;
    _state[next_state_idx].P[1] = 0.0f;
    _state[next_state_idx].P[2] = 0.0f;
    _state[next_state_idx].P[3] = velVar;
    _state[next_state_idx].P[4] = 0.0f;
    _state[next_state_idx].P[5] = 0.05f*0.05f;

    _state_idx = next_state_idx;
}

void PosVelEKF::predict(float dt, float dVel, float dVelNoise)
{
    uint8_t next_state_idx = (_state_idx+1)%2;
    const float abias_pnoise = 0.005f;
    POSVELEKF_PREDICTION_CALC_STATE(_state[_state_idx].P, abias_pnoise, dt, dVel, dVelNoise, _state[_state_idx].x, _state[next_state_idx].x)
    POSVELEKF_PREDICTION_CALC_COV(_state[_state_idx].P, abias_pnoise, dt, dVel, dVelNoise, _state[_state_idx].x, _state[next_state_idx].P)

    _state_idx = next_state_idx;
}

void PosVelEKF::fusePos(float pos, float posVar)
{
    uint8_t next_state_idx = (_state_idx+1)%2;

    POSVELEKF_POS_CALC_STATE(_state[_state_idx].P, posVar, _state[_state_idx].x, pos, _state[next_state_idx].x)
    POSVELEKF_POS_CALC_COV(_state[_state_idx].P, posVar, _state[_state_idx].x, pos, _state[next_state_idx].P)

    _state_idx = next_state_idx;
}

void PosVelEKF::fuseVel(float vel, float velVar)
{
    uint8_t next_state_idx = (_state_idx+1)%2;

    POSVELEKF_VEL_CALC_STATE(_state[_state_idx].P, velVar, _state[_state_idx].x, vel, _state[next_state_idx].x)
    POSVELEKF_VEL_CALC_COV(_state[_state_idx].P, velVar, _state[_state_idx].x, vel, _state[next_state_idx].P)

    _state_idx = next_state_idx;
}

float PosVelEKF::getPosNIS(float pos, float posVar)
{
    float ret;

    POSVELEKF_POS_CALC_NIS(_state[_state_idx].P, posVar, _state[_state_idx].x, pos, ret)

    return ret;
}
