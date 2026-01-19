#include "ekf.h"

void ekf_init(ekf_state_t *ekf, double h0, double v0, double dt)
{
    ekf->h = h0;
    ekf->v = v0;
    ekf->dt = dt;
    
    ekf->P[0] = 1.0;
    ekf->P[1] = 0.0;
    ekf->P[2] = 0.0;
    ekf->P[3] = 1.0;

    ekf->q_h = 0.01;
    ekf->q_v = 0.1;
    ekf->r_baro = 1.0;
}

void ekf_set_tuning(ekf_state_t *ekf, double q_h, double q_v, double r_baro)
{
    ekf->q_h = q_h;
    ekf->q_v = q_v;
    ekf->r_baro = r_baro;
}

void ekf_predict(ekf_state_t *ekf, double acc_z)
{
    double dt = ekf->dt;

    ekf->h = ekf->h + ekf->v * dt + 0.5 * acc_z * dt * dt;
    ekf->v = ekf->v + acc_z * dt;

    double p00 = ekf->P[0];
    double p01 = ekf->P[1];
    double p10 = ekf->P[2];
    double p11 = ekf->P[3];

    double fp00 = p00 + dt * p10;
    double fp01 = p01 + dt * p11;

    ekf->P[0] = fp00 + dt * fp01 + ekf->q_h;
    ekf->P[1] = fp01;
    ekf->P[2] = p10 + dt * p11;
    ekf->P[3] = p11 + ekf->q_v;
}

void ekf_update(ekf_state_t *ekf, double baro_h)
{
    double y = baro_h - ekf->h;
    double S = ekf->P[0] + ekf->r_baro;

    double k0 = ekf->P[0] / S;
    double k1 = ekf->P[2] / S;

    ekf->h = ekf->h + k0 * y;
    ekf->v = ekf->v + k1 * y;

    double p00 = ekf->P[0];
    double p01 = ekf->P[1];

    ekf->P[0] = (1.0 - k0) * p00;
    ekf->P[1] = (1.0 - k0) * p01;
    ekf->P[2] = ekf->P[2] - k1 * p00;
    ekf->P[3] = ekf->P[3] - k1 * p01;
}
