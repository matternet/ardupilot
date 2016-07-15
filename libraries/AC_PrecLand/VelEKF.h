/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

class VelEKF {
public:
    void init(float pos, float posVar, float vel, float velVar);
    void predict(float dt, float dVel, float dVelNoise);
    void fusePos(float pos, float posVar);
    void fuseVel(float vel, float velVar);

    float getPos() { return _state[0]; }
    float getVel() { return _state[1]; }

private:
    float _state[2];
    float _cov[3];
};
