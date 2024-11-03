#ifndef RLS_MAG_CALIBRATION_H
#define RLS_MAG_CALIBRATION_H

// Define matrix size (3x3 for 3D magnetometer)
#define N 3

// TODO: NEED ADJUSTMENTS BASED ON REQUIREMENTS
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

// Function declarations

/**
 * Initialize RLS state
 * @param state Pointer to RLSState structure to initialize
 */
void init_rls(RLSState *state);

/**
 * Update RLS state with new measurement
 * @param state Pointer to RLSState structure
 * @param raw Array of raw magnetometer readings
 * @param reference Array of reference (true) magnetic field values
 */
void update_rls(RLSState *state, const double raw[N], const double reference[N]);

/**
 * Apply calibration to raw magnetometer data
 * @param cal Pointer to MagCalibration structure
 * @param raw Array of raw magnetometer readings
 * @param calibrated Array to store calibrated magnetometer readings
 */
void apply_calibration(const MagCalibration *cal, const double raw[N], double calibrated[N]);

#endif // RLS_MAG_CALIBRATION_H