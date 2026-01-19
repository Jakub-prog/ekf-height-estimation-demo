#ifndef EKF_H
#define EKF_H

typedef struct {
    double h;
    double v;
    double P[4];    /* row-major 2x2 */
    double q_h;
    double q_v;
    double r_baro;
    double dt;
} ekf_state_t;

void ekf_init(ekf_state_t *ekf, double h0, double v0, double dt);
void ekf_set_tuning(ekf_state_t *ekf, double q_h, double q_v, double r_baro);
void ekf_predict(ekf_state_t *ekf, double acc_z);
void ekf_update(ekf_state_t *ekf, double baro_h);

#endif /* EKF_H */
