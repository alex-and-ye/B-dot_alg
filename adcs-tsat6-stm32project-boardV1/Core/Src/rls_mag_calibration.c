#include <stdio.h>
#include <math.h>
// #include "./Inc/rls_mag_calibration.h"


// Matrix size (3x3 for 3D magnetometer)
#define N 3

// RLS parameters - ADJUST THESE BASED ON YOUR REQUIREMENTS
#define LAMBDA 0.995  // Forgetting factor (0.9 to 1.0)
#define DELTA 1000.0  // Initial covariance matrix diagonal value

// TODO: REQUIRES ADJUSTMENT BASED ON HARDWARE
#define INITIAL_SCALE 1.0
#define INITIAL_OFFSET 0.0

// Magnetometer calibration parameters
typedef struct {
    double scale[N][N];
    double offset[N];
} MagCalibration;

// RLS state
typedef struct {
    double P[N*2][N*2];  // Covariance matrix
    MagCalibration params;
} RLSState;

// Initialize RLS state
void init_rls(RLSState *state) {
    int i, j;
    
    // Initialize covariance matrix
    for (i = 0; i < N*2; i++) {
        for (j = 0; j < N*2; j++) {
            state->P[i][j] = (i == j) ? DELTA : 0.0;
        }
    }
    
    // Initialize calibration parameters
    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            state->params.scale[i][j] = (i == j) ? INITIAL_SCALE : 0.0;
        }
        state->params.offset[i] = INITIAL_OFFSET;
    }
}

// Update RLS state with new measurement
void update_rls(RLSState *state, const double raw[N], const double reference[N]) {
    double phi[N*2];
    double k[N*2];
    double y, e;
    int i, j;
    
    // Compute phi (measurement vector)
    for (i = 0; i < N; i++) {
        phi[i] = raw[i];
        phi[i+N] = 1.0;
    }
    
    // Compute Kalman gain
    for (i = 0; i < N*2; i++) {
        k[i] = 0.0;
        for (j = 0; j < N*2; j++) {
            k[i] += state->P[i][j] * phi[j];
        }
    }
    
    y = 0.0;
    for (i = 0; i < N*2; i++) {
        y += phi[i] * k[i];
    }
    
    for (i = 0; i < N*2; i++) {
        k[i] /= (LAMBDA + y);
    }
    
    // Update parameters
    for (i = 0; i < N; i++) {
        e = reference[i];
        for (j = 0; j < N; j++) {
            e -= state->params.scale[i][j] * raw[j];
        }
        e -= state->params.offset[i];
        
        for (j = 0; j < N; j++) {
            state->params.scale[i][j] += k[j] * e;
        }
        state->params.offset[i] += k[i+N] * e;
    }
    
    // Update covariance matrix
    double temp[N*2][N*2];
    for (i = 0; i < N*2; i++) {
        for (j = 0; j < N*2; j++) {
            temp[i][j] = -k[i] * phi[j];
            if (i == j) temp[i][j] += 1.0;
        }
    }
    
    for (i = 0; i < N*2; i++) {
        for (j = 0; j < N*2; j++) {
            state->P[i][j] = 0.0;
            for (int k = 0; k < N*2; k++) {
                state->P[i][j] += temp[i][k] * state->P[k][j];
            }
            state->P[i][j] /= LAMBDA;
        }
    }
}

// Apply calibration to raw magnetometer data
void apply_calibration(const MagCalibration *cal, const double raw[N], double calibrated[N]) {
    int i, j;
    for (i = 0; i < N; i++) {
        calibrated[i] = 0.0;
        for (j = 0; j < N; j++) {
            calibrated[i] += cal->scale[i][j] * raw[j];
        }
        calibrated[i] += cal->offset[i];
    }
}

// Example usage
int main() {
    RLSState rls_state;
    init_rls(&rls_state);
    
    // CAN BE REPLACED WITH ACTUAL MAGNETOMETER READINGS AND REFERENCE VALUES
    double raw_data[N] = {100.0, -50.0, 200.0};
    double reference_data[N] = {30.0, -15.0, 60.0};
    
    // Update RLS with new data
    update_rls(&rls_state, raw_data, reference_data);
    
    // Apply calibration to new raw data
    double calibrated_data[N];
    apply_calibration(&rls_state.params, raw_data, calibrated_data);
    
    // Print results
    printf("Calibrated data: %.2f, %.2f, %.2f\n", 
           calibrated_data[0], calibrated_data[1], calibrated_data[2]);
    
    return 0;
}