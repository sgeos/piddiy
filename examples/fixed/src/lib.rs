// lib.rs

//! A simple example of using fixed point numbers in a `no_std`, `no_alloc`
//! environment. The encapsulation is poor, and the structure is intentionally
//! straightforward.

#![no_std]

extern crate fixed;
use fixed::types::I16F16;
use piddiy::PidController;

/// Struct to hold control data for the PID controller.
#[derive(Clone, Copy)]
pub struct ControlData {
    pub altitude: I16F16, // Current altitude of the object being controlled.
    pub altitude_previous: I16F16, // Previous cycle's altitude, for calculating rate of altitude change.
    pub gyro: I16F16,              // Current gyro reading (angular velocity in degrees per second).
    pub gyro_previous: I16F16, // Previous cycle's gyro reading, for calculating angular acceleration.
    pub integral_decay: I16F16, // Factor to decay the integral term over time.
    pub alpha: I16F16,         // Smoothing factor for blending current and previous derivatives.
    pub dt: I16F16,            // Time interval between control updates.
}

/// Function to compute PID outputs based on control data.
pub fn control_system_compute(
    pid: &mut PidController<I16F16, ControlData>,
    data: ControlData,
) -> (I16F16, I16F16, I16F16) {
    let altitude_change = (data.altitude - data.altitude_previous) / data.dt;
    let error = pid.set_point - data.altitude + I16F16::from_num(0.1) * altitude_change;
    let integral = pid.integral * data.integral_decay + error * data.dt;
    let angular_acceleration = (data.gyro - data.gyro_previous) / data.dt;
    let derivative =
        data.alpha * error / data.dt + (I16F16::from_num(1.0) - data.alpha) * angular_acceleration;

    (error, integral, derivative)
}

/// Struct that encapsulates the entire control system, including the PID controller.
pub struct ControlSystem {
    pub pid: PidController<I16F16, ControlData>,
    pub control_data: ControlData,
}

impl ControlSystem {
    /// Constructs a new ControlSystem with initial settings.
    pub fn new(
        set_point: I16F16,
        kp: I16F16,
        ki: I16F16,
        kd: I16F16,
        control_data: ControlData,
    ) -> Self {
        let mut pid = PidController::<I16F16, ControlData>::new();
        pid.compute_fn(control_system_compute)
            .set_point(set_point)
            .kp(kp)
            .ki(ki)
            .kd(kd);

        ControlSystem { pid, control_data }
    }

    /// Updates the control system state by computing the new PID outputs.
    pub fn update(&mut self, dt: I16F16) {
        self.control_data.dt = dt; // Update the time step.
        let output = self.pid.compute(self.control_data);

        // Simulate the effect of the control output on system state.
        self.control_data.altitude += output * I16F16::from_num(0.1); // Simple model for altitude change.
        self.control_data.gyro = output * I16F16::from_num(0.05); // Simple model for gyro change due to control action.

        // Update historical data for the next cycle.
        self.control_data.altitude_previous = self.control_data.altitude;
        self.control_data.gyro_previous = self.control_data.gyro;
    }
}
