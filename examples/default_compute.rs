// examples/default_compute_example.rs

// This example demonstrates the use of `PidController::default_compute`
// for simple PID control scenarios where the basic data type T and user
// data type U are the same. f64 is used in this example.

use piddiy::PidController;

// No ControlData struct, no user compute function.

fn main() {
    let mut intensity = 80.0; // Initial measurement
    let set_point = 100.0; // Target light intensity
    let mut pid = PidController::<f64, f64>::new(); // T and U are both f64
    pid.compute_fn(PidController::default_compute) // Use the default compute function
       .set_point(set_point)
       .kp(0.8) // Proportional gain
       .ki(0.2) // Integral gain
       .kd(0.2); // Derivative gain

    println!("Target Intensity: {:.2} lux", set_point);
    println!("Initial Intensity: {:.2} lux", intensity);

    // Simulate a simple control loop over several cycles.
    for t in 0..10 {
        let control_output = pid.compute(intensity);

        // Simulate the adjustment of the light intensity based on the control output.
        intensity += control_output;

        println!(
            "t = {:2}s, Intensity = {:.2} lux, Control Output = {:.2}, Error = {:.2}, Integral = {:.2}, Derivative = {:.2}",
            t, intensity, control_output, pid.error, pid.integral, pid.derivative
        );
    }
}

