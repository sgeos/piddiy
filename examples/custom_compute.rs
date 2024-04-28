// examples/custom_compute.rs

// This Rust example showcases a PID (Proportional-Integral-Derivative)
// controller simulation employing custom computations for error, integral, and
// derivative components within the control_system_compute function. These
// computations are contrived and intentionally not possible with the built-in
// library functions. This example does not reflect real-world flight control
// system design or flight dynamics.

use piddiy::PidController;

// Define a struct to hold control data for the PID controller.
#[derive(Clone, Copy)]
struct ControlData {
    altitude: f64,          // Current altitude of the drone or object being controlled.
    altitude_previous: f64, // Altitude from the previous update cycle, used to calculate altitude change rate.
    gyro: f64, // Current gyro reading, indicating angular velocity in degrees per second.
    gyro_previous: f64, // Gyro reading from the previous cycle, for calculating angular acceleration.
    integral_decay: f64, // Factor to reduce the impact of accumulated error over time in the integral term.
    alpha: f64,          // Smoothing factor for blending current and previous derivatives.
    dt: f64, // Time interval between control updates, crucial for derivative and integral calculations.
}

// Define the function that computes PID outputs based on control data.
fn control_system_compute(
    pid: &mut PidController<f64, ControlData>,
    data: ControlData,
) -> (f64, f64, f64) {
    // Calculate the rate of altitude change by comparing current and previous altitudes.
    let altitude_change = (data.altitude - data.altitude_previous) / data.dt;

    // Compute the error as the difference from setpoint, adjusted for rate of altitude change.
    let error = pid.set_point - data.altitude + 0.1 * altitude_change;

    // Calculate the integral of the error, applying a decay factor to reduce its growth over time.
    let integral = pid.integral * data.integral_decay + error * data.dt;

    // Determine the angular acceleration from changes in gyro readings.
    let angular_acceleration = (data.gyro - data.gyro_previous) / data.dt;

    // Calculate the derivative of the error, blending it with the rate of angular acceleration.
    let derivative = data.alpha * error / data.dt + (1.0 - data.alpha) * angular_acceleration;

    // Return the computed error, integral, and derivative values.
    (error, integral, derivative)
}

fn main() {
    // Define the target altitude for the PID controller.
    let set_point = 100.0;

    // Initialize the PID controller with specific control parameters.
    let mut pid = PidController::<f64, ControlData>::new();
    pid.compute_fn(control_system_compute)
        .set_point(set_point)
        .kp(0.3) // Proportional gain, affecting response to current error.
        .ki(0.05) // Integral gain, affecting response based on cumulative error.
        .kd(0.65); // Derivative gain, affecting response based on rate of error change.

    // Initialize control data with starting values.
    let mut control_data = ControlData {
        altitude: 65.0,          // Initial altitude is set below the target.
        altitude_previous: 50.0, // Set a hypothetical previous altitude.
        gyro: 15.0,              // Initial gyro reading.
        gyro_previous: 15.0,     // Initial previous gyro reading.
        integral_decay: 0.85,    // Set an integral decay factor.
        alpha: 0.7,              // Set the smoothing factor for derivative calculation.
        dt: 0.1,                 // Time step of 0.1 seconds per cycle.
    };

    println!("Target Altitude: {:.2}m", set_point);
    println!("Initial Altitude: {:.2}m", control_data.altitude);

    // Loop to simulate the PID control over several cycles.
    let mut control_output;
    for i in 0..10 {
        // Calculate the control output based on current data.
        control_output = pid.compute(control_data); // .clamp(-100.0, 100.0);

        // Crude simulation of the effect of the control output on altitude and gyro readings.
        control_data.altitude += control_output * 0.1; // Simulate altitude change.
        control_data.gyro = control_output * 0.05; // Simulate gyro change due to control action.

        // Update the historical data for the next cycle.
        control_data.altitude_previous = control_data.altitude;
        control_data.gyro_previous = control_data.gyro;

        // Output the results of this cycle.
        println!(
            "Cycle {:2}: Altitude = {:.2}m, Gyro = {:.2}deg/s, Error = {:.2}, Integral = {:.2}, Derivative = {:.2}",
            i, control_data.altitude, control_data.gyro, pid.error, pid.integral, pid.derivative
        );
    }
}
