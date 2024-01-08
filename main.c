#include <stdio.h>

#define FILTER_LENGTH 10
#define MAX_DELAY 500
#define MAX_WEIGHT 2000
#define MAX_VIBRATION_ERROR 2
#define SAMPLING_RATE 1000 // Sampling rate in Hz

typedef struct {
    double weights[FILTER_LENGTH];
    double P[FILTER_LENGTH][FILTER_LENGTH];
    double lambda;
} AdaptiveFilter;

AdaptiveFilter weightFilter;

void initFilter(AdaptiveFilter *filter) {
    // Initialize filter parameters
    for (int i = 0; i < FILTER_LENGTH; ++i) {
        filter->weights[i] = 0.0;
        for (int j = 0; j < FILTER_LENGTH; ++j) {
            filter->P[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    filter->lambda = 0.99; // Forgetting factor
}

double updateFilter(AdaptiveFilter *filter, double measurement) {
    // RLS Algorithm
    double x[FILTER_LENGTH];
    for (int i = 0; i < FILTER_LENGTH - 1; ++i) {
        x[i + 1] = filter->weights[i];
    }
    x[0] = measurement;

    double error = MAX_WEIGHT - measurement;
    double P_x[FILTER_LENGTH];
    double alpha;

    // Calculate P * x
    for (int i = 0; i < FILTER_LENGTH; ++i) {
        P_x[i] = 0.0;
        for (int j = 0; j < FILTER_LENGTH; ++j) {
            P_x[i] += filter->P[i][j] * x[j];
        }
    }

    // Calculate alpha
    alpha = 1.0 / (1.0 + filter->lambda * x[0] * P_x[0]);

    // Update weights
    for (int i = 0; i < FILTER_LENGTH; ++i) {
        filter->weights[i] += alpha * error * P_x[i];
    }

    // Update P
    for (int i = 0; i < FILTER_LENGTH; ++i) {
        for (int j = 0; j < FILTER_LENGTH; ++j) {
            filter->P[i][j] = filter->lambda * (filter->P[i][j] - alpha * P_x[i] * P_x[j]);
        }
    }

    return measurement + filter->weights[0];
}

int main() {
    initFilter(&weightFilter);

    int time;
    for (time = 0; time <= MAX_DELAY; time += 1000 / SAMPLING_RATE) {
        double measurement = 2000.0 + 6.0 * sin(2 * 3.14159 * 0.1 * time); // Simulating vibration
        double filteredWeight = updateFilter(&weightFilter, measurement);

        printf("Time: %d ms, Measured Weight: %.2f grams, Filtered Weight: %.2f grams\n", time, measurement, filteredWeight);
    }

    return 0;
}
