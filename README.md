<!-- README.md -->


# PIDDIY

PIDDIY (Proportional-Integral-Derivative Do It Yourself) is a lightweight and flexible library designed for building custom PID controllers in `no_std` Rust environments. This library provides the foundational components for Proportional, Integral, and Derivative controls, allowing users to fine-tune and deploy PID algorithms tailored to their specific application needs.

## Philosophy

The core philosophy behind PIDDIY is to offer a simple yet powerful set of tools that enable developers to implement custom PID control strategies without the overhead of unnecessary features. By providing direct control over the three primary components of PID systems:

- **Proportional (P)**: Adjusts the controller output in proportion to the current error value.
- **Integral (I)**: Summates past errors, providing a correction based on the accumulated offset to reduce steady-state error.
- **Derivative (D)**: Predicts future error by considering the rate of change of the error, helping to minimize overshoot and dampen the system response.

PIDDIY encourages a hands-on approach, allowing for deep integration into systems where precise control and customization are required.

## Features

- **Flexible Control Strategies**: Customize every aspect of your PID controller, from gain settings to the computation of the PID terms.
- **no_std Compatibility**: Designed to work in resource-constrained environments without the standard library.
- **Customizable Compute Functions**: Implement your own logic for error computation, integral accumulation, and derivative calculation to fit your specific control requirements.
- **Minimalist Design**: Focuses on core functionalities to maintain a small footprint and high performance, ideal for embedded and real-time applications.
- **Extensive Documentation**: Each module and function is thoroughly documented with examples, making it easy to get started and integrate into your projects.

## Quick Start

To use PIDDIY in your project, add the following to your `Cargo.toml`:

```toml
[dependencies]
piddiy = { version = "0.1.0", default-features = false }
```

Example Usage
Here is a basic example of how to set up a PID controller:

```rust
use piddiy::PidController;

// Custom control data for temperature reading.
#[derive(Clone, Copy)]
struct ControlData {
    temperature: f64, // Current Temperature (Celsius)
    dt: f64,          // Time Step (Seconds)
}

// Custom compute function for temperature control.
fn temperature_control_compute(
    pid: &mut PidController<f64, ControlData>,
    data: ControlData,
) -> (f64, f64, f64) {
    // PID control is calculated with library routines.
    let error = pid.calculate_error(data.temperature);
    let integral = pid.calculate_integral_dt(error, data.dt);
    let derivative = pid.calculate_derivative_backward(error, data.dt);
    (error, integral, derivative)
}

fn main() {
    // Initialize the PID controller.
    let mut pid = PidController::<f64, ControlData>::new();
    pid
        .compute_fn(temperature_control_compute) // Custom PID Function
        .set_point(23.0) // Desired Temperature (Celsius)
        .kp(0.1) // Proportional Gain
        .ki(0.05) // Integral Gain
        .kd(0.01); // Derivative Gain

    // Simutlated control data.
    let mut control_data = ControlData {
        temperature: 20.0, // Initial Temperature (Celsius)
        dt: 0.1, // Measurement Time Step (Seconds)
    };

    // Perform PID control computation to determine control action.
    let control_output = pid.compute(control_data);
    println!("Control Action: {}", control_output);
}
```

## License

PIDDIY is licensed under the BSD Zero Clause License (0BSD), allowing for nearly unrestricted use, modification, and distribution of the library in both open and closed source projects.

For more details, see the [LICENSE](LICENSE) file included with the library.

