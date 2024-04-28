// examples/derivative_smoothing.rs

// This example demonstrates the use of a PID controller with derivative smoothing.
// It utilizes built-in functions to integrate gyroscopic data into the derivative
// term, highlighting a PID control scenario designed for educational purposes.
// The example is intended to illustrate derivative smoothing techniques and does not
// represent a realistic real-world implementation.

use piddiy::PidController;

// Define a struct to hold control data
//  This includes sensor outputs and a factor for derivative smoothing.
#[derive(Clone, Copy)]
struct ControlData {
    imu: f64,   // IMU sensor output, e.g., orientation angle
    gyro: f64,  // Gyro sensor output, e.g., angular velocity
    alpha: f64, // Smoothing factor for derivative calculation
}

// Custom compute function that calculates PID terms using sensor data.
// Note that the derivative is smoothed and calculated with the gyro
// instead of the error.
fn control_system_compute(
    pid: &mut PidController<f64, ControlData>,
    data: ControlData,
) -> (f64, f64, f64) {
    let error = pid.calculate_error(data.imu); // Error is the deviation from the setpoint.
    let integral = pid.calculate_integral(error); // Integral of error over time.
    let derivative = pid.calculate_derivative_smooth(data.gyro, data.alpha, 1.0 - data.alpha);
    (error, integral, derivative)
}

fn main() {
    let set_point = 0.0; // Desired orientation angle (degrees), assuming stable position is 0 degrees
    let mut pid = PidController::<f64, ControlData>::new();
    pid.compute_fn(control_system_compute)
        .set_point(set_point)
        .kp(1.2) // Proportional gain.
        .ki(0.4) // Integral gain.
        .kd(-0.3); // Derivative gain (negative to counteract increasing error).

    // Simulate a control loop
    let mut control_data = ControlData {
        imu: 10.0,  // Initial angle from IMU
        gyro: 0.0,  // Initial angular velocity from gyro
        alpha: 0.6, // Smoothing factor
    };

    println!("Target: {:.3} degrees", set_point);
    println!(
        "Initial State: IMU = {:.3} degrees, Gyro = {:.3} degrees/s",
        control_data.imu, control_data.gyro
    );
    for t in 0..10 {
        let control_output = pid.compute(control_data);

        // Update system state based on control output (simplified)
        control_data.imu += control_output; // Adjust the angle based on the control output
        control_data.gyro = -control_output; // Assume the gyro changes inversely with the control adjustment

        // Simulate new readings.
        control_data.imu *= 0.95; // Simulate some natural damping

        println!(
            "t = {:2}, IMU = {:+.3} degrees, Gyro = {:+.3} degrees/s",
            t, control_data.imu, control_data.gyro
        );
    }
}
