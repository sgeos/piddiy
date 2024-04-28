// src/main.rs

// custom_compute example ported to used fixed point math.
// PID control and simulation code have been extracted to lib.rs.

#![feature(start)]
#![no_std]

extern crate libc;
use core::fmt::Write;
use fixed::types::I16F16;
use piddiy_fixed_example::{ControlData, ControlSystem};

// Implement minimal formatting features for output.
struct Stdout;

impl Write for Stdout {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let mut buffer = [0u8; 1024]; // Buffer to hold the string and null terminator

        // Ensure we don't exceed the buffer size
        if s.len() + 1 > buffer.len() {
            return Err(core::fmt::Error);
        }

        // Copy the string into the buffer and null-terminate it
        buffer[..s.len()].copy_from_slice(s.as_bytes());
        buffer[s.len()] = 0; // Null terminator

        unsafe {
            // Use %s to print the string from the buffer
            libc::printf(b"%s\0".as_ptr() as *const _, buffer.as_ptr() as *const _);
        }
        Ok(())
    }
}

#[start]
fn _start(_: isize, _: *const *const u8) -> isize {
    let mut stdout = Stdout;

    let initial_data = ControlData {
        altitude: I16F16::from_num(65.0),
        altitude_previous: I16F16::from_num(50.0),
        gyro: I16F16::from_num(15.0),
        gyro_previous: I16F16::from_num(15.0),
        integral_decay: I16F16::from_num(0.85),
        alpha: I16F16::from_num(0.7),
        dt: I16F16::from_num(0.1),
    };

    let mut control_system = ControlSystem::new(
        I16F16::from_num(100.0), // set point
        I16F16::from_num(0.3),   // kp
        I16F16::from_num(0.05),  // ki
        I16F16::from_num(0.65),  // kd
        initial_data,
    );

    writeln!(stdout, "Fixed point custom_compute example.").ok();
    writeln!(
        stdout,
        "Target Altitude: {:.2}",
        control_system.pid.set_point
    )
    .ok();
    writeln!(
        stdout,
        "Initial Altitude: {:.2}",
        control_system.control_data.altitude
    )
    .ok();

    // Run simulation loop for 10 iterations.
    for i in 0..10 {
        control_system.update(I16F16::from_num(0.1)); // Simulate time step update.

        writeln!(
            stdout,
            "Cycle {:2}: Altitude = {:.2}m, Gyro = {:.2}deg/s, Error = {:.2}, Integral = {:.2}, Derivative = {:.2}",
            i,
            control_system.control_data.altitude,
            control_system.control_data.gyro,
            control_system.pid.error,
            control_system.pid.integral,
            control_system.pid.derivative
        ).ok();
    }

    0
}
