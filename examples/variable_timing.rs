// examples/variable_timing.rs

// This example demonstrates a PID controller simulating temperature control
// with variable timing intervals, explicitly using the `calculate_integral_dt`
// and `calculate_derivative_backward` library functions to handle the respective
// integral and derivative calculations. This examplie illustrates a simple way
// for controllers to accomode irregular timing. This example is intended for
// educational purposes and does not represent a real-world temperature control
// solution.

use piddiy::PidController;

// Define a struct to hold control data
#[derive(Clone, Copy)]
struct ControlData {
    temperature: f64,
    dt: f64, // time step in seconds
}

// Define a custom compute function for our temperature control
fn temperature_control_compute(
    pid: &mut PidController<f64, ControlData>,
    data: ControlData,
) -> (f64, f64, f64) {
    let error = pid.calculate_error(data.temperature);
    let integral = pid.calculate_integral_dt(error, data.dt);
    let derivative = pid.calculate_derivative_backward(error, data.dt);
    (error, integral, derivative)
}

fn main() {
    let set_point = 23.0; // desired temperature (Celsius)
    let mut pid = PidController::<f64, ControlData>::new();
    pid.compute_fn(temperature_control_compute)
        .set_point(set_point)
        .kp(0.1) // Proportional gain
        .ki(0.05) // Integral gain
        .kd(0.01); // Derivative gain

    // Simulate a control loop with variable timing intervals
    let mut control_data = ControlData {
        temperature: 20.0,
        dt: 0.1, // 0.1 second time step
    };

    // Predefined array of time steps to simulate variable timing
    let timing = [
        0.000, 0.101, 0.028, 0.212, 0.495, 0.099, 0.067, 0.146, 0.111, 0.093, 0.183,
    ];
    let mut t = 0.0; // Initialize time
    println!("Target : {:.3} Degrees C", set_point);
    println!(
        "Initial Temperature : {:.3} Degrees C",
        control_data.temperature
    );
    for dt in timing {
        control_data.dt = dt;
        let control_output = pid.compute(control_data);
        // Simulate environment response and update temperature
        t += dt;
        control_data.temperature +=
            (control_output - (control_data.temperature - set_point) * 0.1) * dt;
        println!(
            "Time = {:.3} s : {:.3} degrees C",
            t, control_data.temperature
        );
    }
}
