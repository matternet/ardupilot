#pragma once

#include <stdint.h>

class PosVelEKF {
public:
    void init(float pos, float posVar, float vel, float velVar);
    void predict(float dt, float dVel, float dVelNoise);
    void fusePos(float pos, float posVar);
    void fuseVel(float vel, float velVar);

    float getPos() const { return _state[_state_idx].x[0]; }
    float getVel() const { return _state[_state_idx].x[1]; }
    float getABias() const { return _state[_state_idx].x[2]; }

    float getPosNIS(float pos, float posVar);

private:
    struct {
        float x[3];
        float P[6];
    } _state[2];
    uint8_t _state_idx;
};
