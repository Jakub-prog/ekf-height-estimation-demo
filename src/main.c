#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "ekf.h"

#define SIM_DURATION    30.0
#define IMU_HZ          1000
#define BARO_HZ         100
#define DT_IMU          (1.0 / IMU_HZ)
#define BARO_DIVIDER    (IMU_HZ / BARO_HZ)
#define N_SAMPLES       ((int)(SIM_DURATION * IMU_HZ))

static double randn(void)
{
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
    double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

int main(void)
{
    ekf_state_t ekf;
    FILE *csv;
    int i;
    
    double true_h = 100.0;
    double true_v = 0.0;
    double acc_noise_std = 0.2;
    double baro_noise_std = 1.5;
    double acc_var = acc_noise_std * acc_noise_std;
    double baro_var = baro_noise_std * baro_noise_std;
    
    srand((unsigned int)time(NULL));
    
    ekf_init(&ekf, 95.0, 0.0, DT_IMU);          /* h0, v0, dt */
    ekf_set_tuning(&ekf,
                   acc_var * DT_IMU * DT_IMU,
                   acc_var,
                   baro_var);

    csv = fopen("../data/simulated_baro.csv", "w");
    if (!csv) {
        fprintf(stderr, "Failed to open output file\n");
        return 1;
    }
    
    fprintf(csv, "time,true_h,true_v,acc_cmd,acc_meas,baro_meas,ekf_h,ekf_v\n");
    
    for (i = 0; i < N_SAMPLES; i++) {
        double t = i * DT_IMU;
        double acc_cmd, acc_meas;

        if (t < 5.0) {
            acc_cmd = 2.0;
        } else if (t < 10.0) {
            acc_cmd = 0.0;
        } else if (t < 15.0) {
            acc_cmd = -1.5;
        } else if (t < 20.0) {
            acc_cmd = 0.0;
        } else {
            acc_cmd = 0.8 * sin(0.5 * t);
        }

        true_v += acc_cmd * DT_IMU;
        true_h += true_v * DT_IMU;

        acc_meas = acc_cmd + acc_noise_std * randn();
        ekf_predict(&ekf, acc_meas);

        if (i % BARO_DIVIDER == 0) {
            double baro_meas = true_h + baro_noise_std * randn();
            ekf_update(&ekf, baro_meas);

            fprintf(csv, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    t, true_h, true_v, acc_cmd, acc_meas, baro_meas,
                    ekf.h, ekf.v);
        }
    }
    
    fclose(csv);
    
    printf("Simulation complete.\n");
    printf("Final true height:  %.2f m\n", true_h);
    printf("Final EKF height:   %.2f m\n", ekf.h);
    printf("Final error:        %.2f m\n", fabs(true_h - ekf.h));
    printf("\nOutput written to data/simulated_baro.csv\n");
    
    return 0;
}
