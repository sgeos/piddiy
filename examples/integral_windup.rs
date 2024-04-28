// examples/integral_windup.rs

// This is a conceptual example demonstrating the management of integral
// windup in a PID controller. It is intended as a toy example for educational
// purposes and not as a realistic simulation of refrigeration or any specific
// temperature management system. The focus is on illustrating a crude method
// for handling integral windup.

use piddiy::PidController;

#[derive(Clone, Copy)]
struct ControlData {
    temperature: f64,
    integral_limit: f64,
    integral_reset: bool,
}

fn control_system_compute(
    pid: &mut PidController<f64, ControlData>,
    data: ControlData,
) -> (f64, f64, f64) {
    if data.integral_reset {
        pid.reset_integral(); // Resets the integral if required
    }
    let error = pid.calculate_error(data.temperature); // Calculates the deviation from setpoint
    let integral = pid
        .calculate_integral(error)
        .clamp(-data.integral_limit, data.integral_limit); // Clamps the integral
    let derivative = pid.calculate_derivative(error);

    (error, integral, derivative)
}

fn main() {
    let set_point = -2.0; // Desired temperature in Celsius
    let mut pid = PidController::<f64, ControlData>::new();
    pid.compute_fn(control_system_compute)
        .set_point(set_point)
        .kp(0.5) // Proportional gain
        .ki(0.1) // Integral gain
        .kd(0.05); // Derivative gain

    let mut control_data = ControlData {
        temperature: 30.0,     // Initial temperature
        integral_limit: 10.0,  // Limit for the integral component
        integral_reset: false, // No integral reset initially
    };

    println!("Target Temperature: {:.2}°C", set_point);
    println!("Initial Temperature: {:.2}°C", control_data.temperature);
    for i in 0..10 {
        let (error, integral, derivative) = (pid.error, pid.integral, pid.derivative);

        // Set flag based on anti-windup condition.
        // Comment out the following line to see the effects on the system
        // without integral windup management.
        // Note the integral values and the final temperature.
        control_data.integral_reset = control_data.integral_limit - 1.0 <= integral.abs();

        println!("Cycle {}: Temp = {:.2}°C, Error = {:.2}, Integral = {:.2}, Derivative = {:.2}, Windup = {}", 
                 i, control_data.temperature, error, integral, derivative, control_data.integral_reset);

        // Compute the control output directly using the PID compute function
        let control_output = pid.compute(control_data);

        // Update environment
        control_data.temperature += control_output; // Adjust temperature based on control output
    }
}
