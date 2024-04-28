// src/lib.rs

//! A simple PID control library designed for `no_std` environments.

use num_traits::{Float, FloatConst};

/// Function pointer to compute error, integral, and derivative.
type ComputeFn<T, U> = fn(&mut PidController<T, U>, U) -> (T, T, T);

/// A generic PID controller.
pub struct PidController<T, U>
where
    T: Float + FloatConst,
{
    /// Target setpoint for the PID controller.
    pub set_point: T,
    /// Proportional gain.
    pub kp: T,
    /// Integral gain.
    pub ki: T,
    /// Derivative gain.
    pub kd: T,
    /// Cumulative integral value.
    pub integral: T,
    /// Previous error value.
    pub error: T,
    /// Previous derivative value.
    pub derivative: T,
    /// Function pointer to compute error, integral, and derivative.
    compute: ComputeFn<T, U>,
}

impl<T, U> PidController<T, U>
where
    T: Float + FloatConst,
{
    /// Constructs a new `PidController` with default settings.
    pub fn new() -> Self {
        Self {
            set_point: T::zero(),
            kp: T::one(),
            ki: T::one(),
            kd: T::one(),
            integral: T::zero(),
            error: T::zero(),
            derivative: T::zero(),
            compute: Self::default_compute_fn,
        }
    }

    /// Sets the PID controller's setpoint.
    pub fn set_point(&mut self, set_point: T) -> &mut Self {
        self.set_point = set_point;
        self
    }

    /// Sets the proportional gain.
    pub fn kp(&mut self, kp: T) -> &mut Self {
        self.kp = kp;
        self
    }

    /// Sets the integral gain.
    pub fn ki(&mut self, ki: T) -> &mut Self {
        self.ki = ki;
        self
    }

    /// Sets the derivative gain.
    pub fn kd(&mut self, kd: T) -> &mut Self {
        self.kd = kd;
        self
    }

    /// Sets the compute function.
    pub fn compute_fn(&mut self, compute: ComputeFn<T, U>) -> &mut Self {
        self.compute = compute;
        self
    }

    /// Resets the error to zero.
    pub fn reset_error(&mut self) {
        self.error = T::zero();
    }

    /// Resets the integral accumulator to zero.
    pub fn reset_integral(&mut self) {
        self.integral = T::zero();
    }

    /// Resets the derivative to zero.
    pub fn reset_derivative(&mut self) {
        self.derivative = T::zero();
    }

    /// Resets the error, integral, and derivative values to zero.
    pub fn reset(&mut self) {
        self.reset_error();
        self.reset_integral();
        self.reset_derivative();
    }

    /// Computes the PID control output.
    pub fn compute(&mut self, user_data: U) -> T {
        let (error, integral, derivative) = (self.compute)(self, user_data);

        // Update state
        self.integral = integral;
        self.error = error;
        self.derivative = derivative;

        // Compute output, adjusting integral and derivative terms by the time delta
        let p_output = error * self.kp;
        let i_output = integral * self.ki;
        let d_output = derivative * self.kd;

        // Total output
        p_output + i_output + d_output
    }

    /// Computes the PID control output, adjusting for a variable time step.
    ///
    /// `td`: The time delta since the last update, used to scale the integral and derivative calculations.
    /// `user_data`: Data provided by the user which can include sensor inputs or other relevant information.
    ///
    /// Returns the PID control action output, scaled according to the time delta.
    pub fn compute_dt(&mut self, user_data: U, dt: T) -> T {
        if dt == T::zero() {
            return T::zero(); // Return zero or previous output to handle zero division gracefully
        }

        let (error, integral, derivative) = (self.compute)(self, user_data);

        // Update state
        self.integral = integral;
        self.error = error;
        self.derivative = derivative;

        // Compute output, adjusting integral and derivative terms by the time delta
        let p_output = error * self.kp;
        let i_output = integral * self.ki / dt; // Integral term scaled inversely by time delta
        let d_output = derivative * self.kd * dt; // Derivative term scaled directly by time delta

        // Total output
        p_output + i_output + d_output
    }

    /// Utility method to calculate the error.
    pub fn calculate_error(&mut self, measurement: T) -> T {
        self.set_point - measurement
    }

    /// Utility method to calculate the integral.
    pub fn calculate_integral(&mut self, error: T) -> T {
        self.integral + error
    }

    /// Calculates the integral term for the PID controller with time scaling.
    ///
    /// This method incorporates the time delta (`dt`) into the integral calculation,
    /// which helps maintain consistent control performance regardless
    /// of the variation in the time interval between PID updates.
    /// Including `dt` ensures that the integral term's contribution is
    /// proportional to the actual time elapsed, addressing the integral's
    /// dependence on the sampling rate.
    ///
    /// ## Parameters
    /// - `error`: The current error between the setpoint and the measurement.
    /// - `dt`: The time delta since the last update. This represents the
    /// sampling interval in time units.
    ///
    /// ## Returns
    /// Returns the updated integral value scaled by the time delta.
    ///
    /// ## Usage
    /// This method is typically called within the PID compute function.
    ///
    /// ## Example Usage
    /// ```
    /// use piddiy::PidController;
    ///
    /// // Define a struct to hold control data
    /// struct ControlData {
    ///     measurement: f32,
    ///     dt: f32,
    /// }
    ///
    /// let mut pid: PidController<f32, ControlData> = PidController::new();
    /// pid.compute_fn(|pid, data| {
    ///     let error = pid.calculate_error(data.measurement);
    ///     let integral = pid.calculate_integral_dt(error, data.dt);
    ///     let derivative = pid.calculate_derivative(error);
    ///     (error, integral, derivative)
    /// });
    /// let control_data = ControlData {
    ///     measurement: 0.5f32,
    ///     dt: 0.1f32,
    /// };
    /// let set_point = 1.0f32;
    /// pid.set_point(set_point);
    /// let control_output = pid.compute(control_data);
    /// println!("Control Output: {}", control_output);
    pub fn calculate_integral_dt(&mut self, error: T, dt: T) -> T {
        self.integral + error * dt
    }

    /// Utility method to calculate the derivative.
    pub fn calculate_derivative(&mut self, error: T) -> T {
        error - self.error
    }

    /// Calculates the derivative of the error using the backward difference method.
    ///
    /// This method normalizes the derivative by the time delta (`dt`), making the derivative
    /// calculation consistent regardless of the sampling interval. It's suitable for systems
    /// where the update interval may not be constant, providing a more accurate representation
    /// of the rate of change of the error.
    ///
    /// ## Parameters
    /// - `error`: The current error calculated as the difference between the setpoint and the actual measurement.
    /// - `dt`: The time delta since the last update. This is the duration of the sampling interval in time units.
    ///
    /// ## Returns
    /// Returns the derivative term adjusted for the time delta.
    ///
    /// ## Usage
    /// This method is typically called within the PID compute function, where error dynamics are critical
    /// to the control performance, especially in systems with variable sampling rates.
    ///
    /// ## Example Usage
    /// ```
    /// use piddiy::PidController;
    ///
    /// struct ControlData {
    ///     measurement: f32,
    ///     dt: f32,
    /// }
    ///
    /// let mut pid: PidController<f32, ControlData> = PidController::new();
    /// pid.compute_fn(|pid, data| {
    ///     let error = pid.calculate_error(data.measurement);
    ///     let integral = pid.calculate_integral(error);
    ///     let derivative = pid.calculate_derivative_backward(error, data.dt);
    ///     (error, integral, derivative)
    /// });
    /// let control_data = ControlData {
    ///     measurement: 0.5f32,
    ///     dt: 0.1f32,
    /// };
    /// let set_point = 1.0f32;
    /// pid.set_point(set_point);
    /// let control_output = pid.compute(control_data);
    /// println!("Control Output: {}", control_output);
    /// ```
    pub fn calculate_derivative_backward(&mut self, error: T, dt: T) -> T {
        if dt == T::zero() {
            return T::zero(); // Handle division by zero gracefully
        }
        (error - self.error) / dt
    }

    /// Smoother derivative calculation using a weighted sum of the current and previous derivatives.
    ///
    /// `error`: The current error calculated outside this function.
    /// `weight_current`: Weight for the current derivative calculation.
    /// `weight_previous`: Weight for the previous derivative.
    ///
    /// Returns the smoothed derivative value.
    pub fn calculate_derivative_smooth(
        &self,
        error: T,
        weight_current: T,
        weight_previous: T,
    ) -> T {
        let current_derivative = error - self.error; // Calculate current derivative
        weight_current * current_derivative + weight_previous * self.derivative
    }

    /// Default compute logic to calculate error, integral, and derivative.
    ///
    /// This function provides a basic implementation of the PID compute
    /// logic. It should be used directly within custom compute functions
    /// where the standard calculations are beneficial but require custom setup.
    ///
    /// ## Parameters
    /// - `measurement`: The current measurement from the system or process being controlled.
    ///
    /// ## Returns
    /// Returns a tuple containing the calculated error, integral, and derivative.
    ///
    /// ## Example Usage
    /// ```
    /// use piddiy::PidController;
    ///
    /// struct ControlData {
    ///     measurement: f32,
    ///     other_data: f32,
    /// }
    ///
    /// let mut pid: PidController<f32, ControlData> = PidController::new();
    /// pid.compute_fn(|pid, data| {
    ///     let measurement = data.measurement; // setup here
    ///     pid.default_compute(measurement) // default logic
    /// });
    /// let user_data = ControlData {
    ///     measurement: 0.5f32,
    ///     other_data: 0.0f32,
    /// };
    /// let set_point = 1.0f32;
    /// pid.set_point(set_point);
    /// let control_output = pid.compute(user_data);
    /// println!("Control Output: {}", control_output);
    /// ```
    pub fn default_compute(&mut self, measurement: T) -> (T, T, T) {
        let error = self.calculate_error(measurement);
        let integral = self.calculate_integral(error);
        let derivative = self.calculate_derivative(error);
        (error, integral, derivative)
    }

    /// Computes error, integral, and a smoother derivative using a weighted sum.
    ///
    /// This function provides a variation of the default compute logic by incorporating
    /// a smoothing mechanism into the derivative calculation. It calculates the error and integral
    /// normally but uses a weighted sum of the current and previous derivatives for a smoother output.
    ///
    /// ## Parameters
    /// - `measurement`: The current measurement from the system or process being controlled.
    /// - `weight_current`: The weight for the current derivative calculation.
    /// - `weight_previous`: The weight for the previous derivative, which incorporates historical data.
    ///
    /// ## Returns
    /// Returns a tuple containing the calculated error, integral, and smoother derivative.
    ///
    /// ## Example Usage
    /// ```
    /// use piddiy::PidController;
    ///
    /// // Define a struct to hold control data
    /// struct ControlData {
    ///     measurement: f32,
    ///     weight_current: f32,
    ///     weight_previous: f32,
    /// }
    ///
    /// let mut pid: PidController<f32, ControlData> = PidController::new();
    /// let set_point = 1.0f32;
    /// let control_data = ControlData {
    ///     measurement: 0.5f32,
    ///     weight_current: 0.6f32,
    ///     weight_previous: 0.4f32,
    /// };
    ///
    /// pid.set_point(set_point);
    /// pid.compute_fn(|pid, data| {
    ///     pid.default_compute_smooth(data.measurement, data.weight_current, data.weight_previous)
    /// });
    /// let control_output = pid.compute(control_data);
    /// println!("Control Output: {}", control_output);
    /// ```
    pub fn default_compute_smooth(
        &mut self,
        measurement: T,
        weight_current: T,
        weight_previous: T,
    ) -> (T, T, T) {
        let error = self.calculate_error(measurement);
        let integral = self.calculate_integral(error);
        let derivative = self.calculate_derivative_smooth(error, weight_current, weight_previous);
        (error, integral, derivative)
    }

    /// Default PID controller compute function.
    ///
    /// This function uses a zero value as a placeholder for actual measurements
    /// to allow the PID controller to function safely out of the box,
    /// regardless of the user data type. It is intended as a safe default
    /// and should be replaced with a custom function that accurately
    /// processes real-time data relevant to your specific control application.
    ///
    /// ## Shoehorned Functionality
    ///
    /// Although replacing it is recommended for all practical applications,
    /// useful functionality can be shoehorned if the set_point is manipulated
    /// to directly reflect the error.
    ///
    /// ## Example Usage
    /// ```
    /// use piddiy::PidController;
    ///
    /// let mut pid: PidController<f32, f32> = PidController::new();
    /// let set_point = 1.0f32;
    /// let measurement = 0.5f32;
    /// pid.set_point(set_point - measurement);
    /// let unused_parameter = 0.0f32;
    /// let control_output = pid.compute(unused_parameter); // Example using a dummy value
    /// println!("Control Output: {}", control_output);
    /// ```
    pub fn default_compute_fn(&mut self, _user_data: U) -> (T, T, T) {
        self.default_compute(T::zero())
    }
}
