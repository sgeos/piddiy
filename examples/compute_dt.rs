// examples/compute_td.rs

// This example demonstrates the use of `compute_dt` in a PID controller
// to handle variable timing intervals. This is an alternative to using
// dt to calculate the error, integral, and derivative in the user supplied
// compute function.  This toy example is not an accurate representation
// hydraultic control.

use piddiy::PidController;

// Define a struct to hold control data, including the current pressure and the time delta.
#[derive(Clone, Copy)]
struct ControlData {
    pressure: f64,
    dt: f64, // Time step in seconds
}

// Define the function to compute the PID output using simple proportional control.
fn hydraulic_pressure_compute(
    pid: &mut PidController<f64, ControlData>,
    data: ControlData,
) -> (f64, f64, f64) {
    // Calculate the basic PID components.
    let (error, mut integral, derivative) = pid.default_compute(data.pressure);

    // If the error sign changes, reset the integral to zero to prevent windup.
    if error * pid.error < 0.0 {
      integral = 0.0;
    }

    (error, integral, derivative)
}

fn main() {
    let set_point = 147.0; // Target hydraulic pressure (in psi)
    let mut pid = PidController::<f64, ControlData>::new();
    pid.compute_fn(hydraulic_pressure_compute)
        .set_point(set_point)
        .kp(0.8) // Proportional gain
        .ki(0.4) // Integral gain
        .kd(0.1); // Derivative gain

    // Initialize the control data with the starting conditions.
    let mut control_data = ControlData {
        pressure: 14.7, // Initial pressure is below the set point.
        dt: 0.0,        // Initial delta time.
    };

    println!("Initial Pressure: {:.2} psi", control_data.pressure);
    println!("Target Pressure: {:.2} psi", set_point);

    // Array of time deltas to simulate variable processing intervals.
    let time_steps = [0.05, 0.1, 0.2, 0.15, 0.1, 0.05, 0.08, 0.12, 0.2, 0.15];

    // Simulate the control process over time with variable steps.
    let mut t = 0.0;
    for dt in time_steps {
        t += dt;
        control_data.dt = dt;
        let control_output = pid.compute_dt(control_data, dt);
        let (error, integral, derivative) = (pid.error, pid.integral, pid.derivative);

        // Simulate pressure change in response to the control action.
        control_data.pressure += control_output * dt;

        println!(
            "t = {:.2}s, Dt = {:.2}s: Pressure = {:.2} psi, Control Output = {:.2}, Error = {:.2}, Integral = {:.2}, Derivative = {:.2}",
            t, dt, control_data.pressure, control_output, error, integral, derivative
        );
    }
}
