// #include "Magnetorquers_driver.h"
// #include "GYRO_A3G4250DTR_driver.h"
// #include "magnetometer_driver.h"


#include <stdio.h>
#include <stdint.h>
#include <math.h>
// #include "rls_mag_calibration.c"
// Constants
#define Kd 1e-4  // B-dot gain (requires adjustment)
#define DT 0.1   // Time step between measurements (in seconds)

// Magnetometer calibration parameters (adjust based on your sensor)
#define MAG_SCALE_X 1.0
#define MAG_SCALE_Y 1.0
#define MAG_SCALE_Z 1.0
#define MAG_OFFSET_X 0.0
#define MAG_OFFSET_Y 0.0
#define MAG_OFFSET_Z 0.0

// Magnetorquer parameters (adjust based on your hardware)
#define MAX_VOLTAGE 5.0
#define TORQUER_CONSTANT 1.0  // Nm/A or Nm/V depending on the system

// Function to convert raw magnetometer data to magnetic field values
void convert_magnetometer_data(int16_t raw_x, int16_t raw_y, int16_t raw_z, 
                               float *B_x, float *B_y, float *B_z) {
    *B_x = (float)raw_x * MAG_SCALE_X + MAG_OFFSET_X;
    *B_y = (float)raw_y * MAG_SCALE_Y + MAG_OFFSET_Y;
    *B_z = (float)raw_z * MAG_SCALE_Z + MAG_OFFSET_Z;
}

// Function to implement B-dot algorithm
void b_dot_algorithm(float B_x, float B_y, float B_z,
                     float prev_B_x, float prev_B_y, float prev_B_z,
                     float *m_x, float *m_y, float *m_z) {
    float dB_x = (B_x - prev_B_x) / DT;
    float dB_y = (B_y - prev_B_y) / DT;
    float dB_z = (B_z - prev_B_z) / DT;
    
    *m_x = -Kd * dB_x;
    *m_y = -Kd * dB_y;
    *m_z = -Kd * dB_z;
}

// Function to convert magnetic moment to voltages for magnetorquers
void convert_to_voltages(float m_x, float m_y, float m_z,
                         float *V_x, float *V_y, float *V_z) {
    *V_x = fmin(fmax(m_x / TORQUER_CONSTANT, -MAX_VOLTAGE), MAX_VOLTAGE);
    *V_y = fmin(fmax(m_y / TORQUER_CONSTANT, -MAX_VOLTAGE), MAX_VOLTAGE);
    *V_z = fmin(fmax(m_z / TORQUER_CONSTANT, -MAX_VOLTAGE), MAX_VOLTAGE);
}

// Main control loop
void control_loop(int16_t mag_raw_x, int16_t mag_raw_y, int16_t mag_raw_z,
                  float *V_x, float *V_y, float *V_z) {
    static float prev_B_x = 0, prev_B_y = 0, prev_B_z = 0;
    float B_x, B_y, B_z;
    float m_x, m_y, m_z;
    
    // Convert raw magnetometer data to magnetic field values
    convert_magnetometer_data(mag_raw_x, mag_raw_y, mag_raw_z, &B_x, &B_y, &B_z);
    
    // Apply B-dot algorithm
    b_dot_algorithm(B_x, B_y, B_z, prev_B_x, prev_B_y, prev_B_z, &m_x, &m_y, &m_z);
    
    // Convert magnetic moment to voltages for magnetorquers
    convert_to_voltages(m_x, m_y, m_z, V_x, V_y, V_z);
    
    // Update previous magnetic field values
    prev_B_x = B_x;
    prev_B_y = B_y;
    prev_B_z = B_z;
}

// Example usage
int main() {
    // Simulated raw magnetometer data (replaced with actual sensor readings)
    int16_t mag_raw_x = 100, mag_raw_y = -50, mag_raw_z = 200;
    float V_x, V_y, V_z;
    
    control_loop(mag_raw_x, mag_raw_y, mag_raw_z, &V_x, &V_y, &V_z);
    
    printf("V_x: %f, V_y: %f, V_z: %f\n", V_x, V_y, V_z);
    
    return 0;
}