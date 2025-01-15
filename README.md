<!-- README.md -->

# PIDDIY

PIDDIY (Proportional-Integral-Derivative Do It Yourself) is a lightweight and flexible library designed for building custom PID controllers in `no_std` Rust environments. This library offers foundational components for Proportional, Integral, and Derivative controls, enabling users to fine-tune and deploy PID algorithms tailored to their specific needs.

## Philosophy

PIDDIY empowers developers to bring their own data and compute functions, achieving precision control through customized strategies for the primary components of PID systems:

- **Proportional (P)**: Modulates the controller output in proportion to the current error.
- **Integral (I)**: Integrates past errors, providing a cumulative correction to minimize steady-state error.
- **Derivative (D)**: Computes the rate of change of the error, aiding in reducing overshoot and system oscillations.

## Features

- **no_std Compatibility**: Optimized for use in environments without the standard library, suitable for embedded and resource-constrained applications.
- **Support for Fixed-Point Numbers**: Facilitates precise PID control on systems lacking floating-point units (FPUs), crucial for embedded system performance.
- **Custom Data Type Support**: Accommodates a variety of data types, from simple scalars to complex structured types, ensuring flexibility across different control scenarios.
- **Customizable Compute Functions**: Includes default functions for typical PID tasks while providing the flexibility to implement specialized algorithms tailored to unique requirements.

## Quick Start

To use PIDDIY in your project, add the following to your `Cargo.toml`:

```toml
[dependencies]
piddiy = { version = "0.1.0", default-features = false }
```

Example Usage

This example demonstrates setting up a PID controller for temperature regulation:

```rust
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
```

Run the example using:

```sh
cargo run --example readme
```

## History and Inspiration

Originally developed for aviation flight stabilization, PIDDIY allows rapid adaptation to various aircraft by modifying compute functions to fit specific control surfaces and engine configurations. While tailored for UAVs initially, PIDDIY's flexible architecture now accommodates a broad range of applications requiring custom PID solutions.

## License

PIDDIY is licensed under the BSD Zero Clause License (0BSD), allowing for nearly unrestricted use, modification, and distribution of the library in both open and closed source projects.

For more details, see the [LICENSE](LICENSE) file included with the library.

