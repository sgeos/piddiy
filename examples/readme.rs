// examples/readme.rs

// This example code is found in the README.md and demonstrates a custom PID control
// setup for maintaining a desired temperature using the piddiy library.

use piddiy::PidController;

// Custom control data structure for managing temperature.
#[derive(Clone, Copy)]
struct ControlData {
    temperature: f64, // Current temperature in degrees Celsius.
    dt: f64,          // Time step in seconds between temperature measurements.
}

// Custom compute function that defines the PID computation specific to temperature control.
fn temperature_control_compute(
    pid: &mut PidController<f64, ControlData>,
    data: ControlData,
) -> (f64, f64, f64) {
    // Calculate error, integral, and derivative using the PID library functions.
    let error = pid.calculate_error(data.temperature);
    let integral = pid.calculate_integral_dt(error, data.dt);
    let derivative = pid.calculate_derivative_backward(error, data.dt);
    (error, integral, derivative)
}

fn main() {
    // Setup the PID controller with specific gains and set point.
    let set_point = 23.0; // Desired temperature in degrees Celsius.
    let mut pid = PidController::<f64, ControlData>::new();
    pid.compute_fn(temperature_control_compute)
        .set_point(set_point)
        .kp(0.1)  // Proportional gain.
        .ki(0.05) // Integral gain.
        .kd(0.01); // Derivative gain.

    // Simulated initial conditions.
    let control_data = ControlData {
        temperature: 20.0, // Initial temperature in degrees Celsius.
        dt: 0.1,           // Time step in seconds.
    };

    // Execute the PID control computation.
    let control_output = pid.compute(control_data);

    // Output the results.
    println!("Target Temperature:   {: >7.3} °C", set_point);
    println!("Current Temperature:  {: >7.3} °C", control_data.temperature);
    println!("Measurement Time:     {: >7.3} Seconds", control_data.dt);
    println!("Control Action:       {: >7.3} (Dimensionless)", control_output);
}

