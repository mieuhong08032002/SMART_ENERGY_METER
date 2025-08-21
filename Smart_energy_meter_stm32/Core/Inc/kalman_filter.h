// kalman_filter.h
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct
{
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float x; // Value
    float p; // Estimation error covariance
    float k; // Kalman gain
} Kalman_t;

void Kalman_Init(Kalman_t *kalman, float q, float r, float initial_value);
float Kalman_Update(Kalman_t *kalman, float measurement);

extern Kalman_t current_filter;
extern Kalman_t voltage_filter;

void Kalman_Filters_Init(void);
float Get_Filtered_Current(float raw_current);
float Get_Filtered_Voltage(float raw_voltage);

#endif // KALMAN_FILTER_H
