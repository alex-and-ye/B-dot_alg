#include "Magnetorquers_driver.h"
#include "GYRO_A3G4250DTR_driver.h"
#include "magnetometer_driver.h"


// Calculating magnetic field data
Vector3 mag_raw, gyro_raw, mag_calibrated, orientation;
Matrix3x3 rotation_matrix;

while (1) {
    // Read sensor data
    mag_raw = read_magnetometer();
    gyro_raw = read_gyroscope();

    // Calibrate magnetometer data
    mag_calibrated = calibrate_magnetometer(mag_raw);

    // Update orientation using sensor fusion
    orientation = update_orientation(orientation, mag_calibrated, gyro_raw);

    // Calculate rotation matrix from orientation
    rotation_matrix = calculate_rotation_matrix(orientation);

    // Rotate magnetic field vector to Earth frame
    Vector3 mag_earth = rotate_vector(rotation_matrix, mag_calibrated);

    // Apply tilt compensation
    Vector3 mag_compensated = tilt_compensate(mag_earth, orientation);

    // mag_compensated is now the magnetic field vector in the Earth frame
    output_magnetic_field_vector(mag_compensated);

    // Wait for next sample
    delay(SAMPLE_PERIOD);
}



// b-dot algorithm given Magnetic field data

#define K 1000000  // Control gain
#define DT 1.0     // Time step (1 second)

Vector3 prev_B = {0, 0, 0};
Vector3 current_B = {0, 0, 0};
Vector3 B_dot = {0, 0, 0};
Vector3 M = {0, 0, 0};

while (1) {
    // Read magnetometer
    current_B = read_magnetometer();

    // Calculate B-dot
    B_dot.x = (current_B.x - prev_B.x) / DT;
    B_dot.y = (current_B.y - prev_B.y) / DT;
    B_dot.z = (current_B.z - prev_B.z) / DT;

    // Calculate control command
    M.x = -K * B_dot.x;
    M.y = -K * B_dot.y;
    M.z = -K * B_dot.z;

    // Apply control to magnetorquers
    apply_magnetorquer_command(M);

    // Update previous B
    prev_B = current_B;

    // Wait for next control cycle
    sleep(DT);
}