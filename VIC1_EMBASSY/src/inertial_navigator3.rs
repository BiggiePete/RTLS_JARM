#![no_std]

use defmt::println;
use libm::{acos, asin, atan2, copysign, cos, fabs, sin, sqrt};

// --- Unit Conversion Constants ---
/// Standard gravity in m/s^2.
pub const G_STANDARD: f64 = 9.80665;
/// Conversion factor from degrees to radians.
pub const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;
/// Conversion factor from radians to degrees.
pub const RAD_TO_DEG: f64 = 180.0 / core::f64::consts::PI;

/// High-precision 3D vector using f64 for maximum precision
#[derive(Clone, Copy, Debug)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn magnitude(&self) -> f64 {
        sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        if mag > 1e-12 {
            Self::new(self.x / mag, self.y / mag, self.z / mag)
        } else {
            *self
        }
    }

    pub fn cross(&self, other: &Vec3) -> Vec3 {
        Vec3::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    pub fn dot(&self, other: &Vec3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
}

impl core::ops::Add for Vec3 {
    type Output = Vec3;
    fn add(self, other: Vec3) -> Vec3 {
        Vec3::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl core::ops::Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, other: Vec3) -> Vec3 {
        Vec3::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl core::ops::Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, scalar: f64) -> Vec3 {
        Vec3::new(self.x * scalar, self.y * scalar, self.z * scalar)
    }
}

/// Quaternion for rotation representation (more stable than Euler angles)
#[derive(Clone, Copy, Debug)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    pub fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0)
    }

    pub fn from_axis_angle(axis: Vec3, angle_rad: f64) -> Self {
        let half_angle = angle_rad * 0.5;
        let sin_half = sin(half_angle);
        let cos_half = cos(half_angle);
        let normalized_axis = axis.normalize();

        Self::new(
            cos_half,
            normalized_axis.x * sin_half,
            normalized_axis.y * sin_half,
            normalized_axis.z * sin_half,
        )
    }

    /// Create quaternion to rotate from one vector to another
    pub fn from_two_vectors(from: Vec3, to: Vec3) -> Self {
        let from_norm = from.normalize();
        let to_norm = to.normalize();

        let dot = from_norm.dot(&to_norm);

        // Vectors are nearly opposite
        if dot < -0.999999 {
            // Find an orthogonal vector
            let mut orthogonal = Vec3::new(1.0, 0.0, 0.0);
            if fabs(from_norm.x) > 0.9 {
                orthogonal = Vec3::new(0.0, 1.0, 0.0);
            }
            let axis = from_norm.cross(&orthogonal).normalize();
            return Self::from_axis_angle(axis, core::f64::consts::PI);
        }

        // Vectors are nearly the same
        if dot > 0.999999 {
            return Self::identity();
        }

        let axis = from_norm.cross(&to_norm);
        let angle = acos(dot.clamp(-1.0, 1.0));

        Self::from_axis_angle(axis, angle)
    }

    pub fn normalize(&self) -> Self {
        let mag = sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if mag > 1e-12 {
            Self::new(self.w / mag, self.x / mag, self.y / mag, self.z / mag)
        } else {
            Self::identity()
        }
    }

    pub fn multiply(&self, other: &Quaternion) -> Quaternion {
        Quaternion::new(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        )
    }

    pub fn rotate_vector(&self, v: Vec3) -> Vec3 {
        let qv = Quaternion::new(0.0, v.x, v.y, v.z);
        let q_conj = Quaternion::new(self.w, -self.x, -self.y, -self.z);
        let temp = self.multiply(&qv);
        let result = temp.multiply(&q_conj);
        Vec3::new(result.x, result.y, result.z)
    }

    /// Convert quaternion to Euler angles (roll, pitch, yaw) in degrees.
    pub fn to_euler(&self) -> Vec3 {
        // Calculate angles in radians first
        let roll_rad = atan2(
            2.0 * (self.w * self.x + self.y * self.z),
            1.0 - 2.0 * (self.x * self.x + self.y * self.y),
        );

        let sin_pitch = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch_rad = if fabs(sin_pitch) >= 1.0 {
            copysign(core::f64::consts::PI / 2.0, sin_pitch)
        } else {
            asin(sin_pitch)
        };

        let yaw_rad = atan2(
            2.0 * (self.w * self.z + self.x * self.y),
            1.0 - 2.0 * (self.y * self.y + self.z * self.z),
        );

        // Convert radians to degrees
        Vec3::new(
            roll_rad * RAD_TO_DEG,
            pitch_rad * RAD_TO_DEG,
            yaw_rad * RAD_TO_DEG,
        )
    }
}

/// Navigation state containing position and orientation
#[derive(Clone, Copy, Debug)]
pub struct NavigationState {
    pub position: Vec3,
    pub rotation: Vec3, // Euler angles in degrees
}

impl NavigationState {
    pub fn new() -> Self {
        Self {
            position: Vec3::zero(),
            rotation: Vec3::zero(),
        }
    }
}

/// Complementary filter parameters for sensor fusion
pub struct FilterParams {
    pub accel_weight: f64,     // Weight for accelerometer in orientation estimation
    pub gyro_bias_alpha: f64,  // Low-pass filter coefficient for gyro bias estimation
    pub accel_threshold: f64, // Threshold to detect if device is in free-fall or high acceleration (in m/s²)
    pub velocity_damping: f64, // Damping factor for velocity drift compensation
    pub position_damping: f64, // Damping factor for position drift compensation
    pub stationary_threshold: f64, // Threshold for detecting stationary state (in m/s²)
    pub bias_learning_rate: f64, // Learning rate for bias estimation
    pub calibration_samples: u32, // Number of samples for initial calibration
    pub calibration_stability_threshold: f64, // Stability threshold for calibration (in m/s²)
}

impl Default for FilterParams {
    fn default() -> Self {
        Self {
            accel_weight: 0.05,        // 5% accelerometer, 95% gyroscope for orientation
            gyro_bias_alpha: 0.002,    // Faster bias adaptation
            accel_threshold: 2.0,      // Lower threshold for better stationary detection (m/s²)
            velocity_damping: 0.98,    // Strong velocity damping
            position_damping: 0.99,    // Moderate position damping
            stationary_threshold: 0.5, // Threshold for detecting stationary state (m/s²)
            bias_learning_rate: 0.01,  // Faster bias learning
            calibration_samples: 50,   // Take 50 samples for calibration
            calibration_stability_threshold: 0.1, // Accelerometer readings should be stable within 0.1 m/s²
        }
    }
}

/// Calibration state for initial orientation setup
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CalibrationState {
    WaitingForStability,
    Calibrating,
    Complete,
}

/// Main inertial navigation system
pub struct InertialNavigator {
    orientation: Quaternion,
    velocity: Vec3,
    position: Vec3,
    gyro_bias: Vec3,
    accel_bias: Vec3,
    prev_accel_world: Vec3,
    params: FilterParams,
    gravity: Vec3,
    stationary_counter: u32,
    motion_detected: bool,
    accel_history: [Vec3; 10],
    gyro_history: [Vec3; 10],
    history_index: usize,
    history_full: bool,
    calibration_state: CalibrationState,
    calibration_samples: Vec3,
    calibration_count: u32,
    calibration_variance: Vec3,
    calibration_mean: Vec3,
}

impl InertialNavigator {
    pub fn new() -> Self {
        Self::with_params(FilterParams::default())
    }

    pub fn with_params(params: FilterParams) -> Self {
        Self {
            orientation: Quaternion::identity(),
            velocity: Vec3::zero(),
            position: Vec3::zero(),
            gyro_bias: Vec3::zero(),
            accel_bias: Vec3::zero(),
            prev_accel_world: Vec3::zero(),
            params,
            gravity: Vec3::new(0.0, 0.0, -G_STANDARD), // Standard gravity in m/s²
            stationary_counter: 0,
            motion_detected: false,
            accel_history: [Vec3::zero(); 10],
            gyro_history: [Vec3::zero(); 10],
            history_index: 0,
            history_full: false,
            calibration_state: CalibrationState::WaitingForStability,
            calibration_samples: Vec3::zero(),
            calibration_count: 0,
            calibration_variance: Vec3::zero(),
            calibration_mean: Vec3::zero(),
        }
    }

    /// Update the navigation state with new sensor data.
    ///
    /// # Arguments
    /// * `accel_g` - Accelerometer data in Gs (1 G = 9.80665 m/s²) (body frame).
    /// * `gyro_dps` - Gyroscope data in degrees/s (body frame).
    /// * `dt` - Time delta in seconds.
    pub fn update(&mut self, accel_g: Vec3, gyro_dps: Vec3, dt: f64) {
        if dt <= 0.0 {
            return;
        }

        // Convert inputs to standard units (m/s^2 and rad/s) for all internal calculations.
        let accel = accel_g * G_STANDARD;
        let gyro = gyro_dps * DEG_TO_RAD;

        // Handle initial calibration phase.
        if self.calibration_state != CalibrationState::Complete {
            self.update_calibration(accel, gyro);
            return; // Don't process navigation until calibrated
        }

        self.update_sensor_history(accel, gyro);
        let is_stationary = self.detect_stationary_state(accel, gyro);
        self.update_bias_estimation_enhanced(accel, gyro, dt, is_stationary);

        let corrected_gyro = gyro - self.gyro_bias;
        let corrected_accel = accel - self.accel_bias;

        self.update_orientation_gyro(corrected_gyro, dt);

        if self.should_use_accelerometer(&corrected_accel) {
            self.correct_orientation_accel(corrected_accel);
        }

        let accel_world = self.orientation.rotate_vector(corrected_accel) - self.gravity;

        if is_stationary {
            self.apply_zero_velocity_update();
        } else {
            self.update_velocity_position_enhanced(accel_world, dt);
        }

        self.prev_accel_world = accel_world;
    }

    pub fn is_calibrated(&self) -> bool {
        self.calibration_state == CalibrationState::Complete
    }

    pub fn get_calibration_state(&self) -> CalibrationState {
        self.calibration_state
    }

    pub fn recalibrate(&mut self) {
        self.calibration_state = CalibrationState::WaitingForStability;
        self.calibration_samples = Vec3::zero();
        self.calibration_count = 0;
        self.calibration_variance = Vec3::zero();
        self.calibration_mean = Vec3::zero();
        self.velocity = Vec3::zero();
        self.position = Vec3::zero();
    }

    fn update_calibration(&mut self, accel: Vec3, gyro: Vec3) {
        match self.calibration_state {
            CalibrationState::WaitingForStability => {
                if self.is_stable_for_calibration(accel, gyro) {
                    self.calibration_state = CalibrationState::Calibrating;
                    self.calibration_samples = Vec3::zero();
                    self.calibration_count = 0;
                    self.calibration_mean = accel;
                }
            }
            CalibrationState::Calibrating => {
                self.calibration_samples = self.calibration_samples + accel;
                self.calibration_count += 1;

                let delta = accel - self.calibration_mean;
                self.calibration_mean =
                    self.calibration_mean + delta * (1.0 / self.calibration_count as f64);
                let delta2 = accel - self.calibration_mean;
                self.calibration_variance = self.calibration_variance
                    + Vec3::new(delta.x * delta2.x, delta.y * delta2.y, delta.z * delta2.z);

                if self.calibration_count >= self.params.calibration_samples {
                    self.finalize_calibration();
                }
            }
            CalibrationState::Complete => {}
        }
    }

    fn is_stable_for_calibration(&self, accel: Vec3, gyro: Vec3) -> bool {
        // Inputs are already in m/s² and rad/s.
        let near_gravity = fabs(accel.magnitude() - G_STANDARD) < 0.5;
        // Check if gyroscope readings are low (not rotating). 0.1 rad/s is ~5.7 deg/s.
        let low_rotation = gyro.magnitude() < 0.1;
        println!(
            "is low_rotation: {}, is near_gravity: {}, {} , {}",
            low_rotation,
            near_gravity,
            gyro.magnitude(),
            fabs(accel.magnitude() - G_STANDARD)
        );
        near_gravity && low_rotation
    }

    fn finalize_calibration(&mut self) {
        let avg_accel = self.calibration_samples * (1.0 / self.calibration_count as f64);
        let measured_gravity_body = avg_accel.normalize() * -G_STANDARD;
        let expected_gravity_world = Vec3::new(0.0, 0.0, -G_STANDARD);

        self.orientation =
            Quaternion::from_two_vectors(expected_gravity_world, measured_gravity_body);

        let magnitude_error = avg_accel.magnitude() - G_STANDARD;
        self.accel_bias = avg_accel.normalize() * magnitude_error;

        self.calibration_state = CalibrationState::Complete;
        self.velocity = Vec3::zero();
        self.position = Vec3::zero();
        self.prev_accel_world = Vec3::zero();
    }

    pub fn get_state(&self) -> NavigationState {
        NavigationState {
            position: self.position,
            rotation: self.orientation.to_euler(),
        }
    }

    pub fn reset(&mut self) {
        self.orientation = Quaternion::identity();
        self.velocity = Vec3::zero();
        self.position = Vec3::zero();
        self.gyro_bias = Vec3::zero();
        self.accel_bias = Vec3::zero();
        self.prev_accel_world = Vec3::zero();
        self.stationary_counter = 0;
        self.motion_detected = false;
        self.accel_history = [Vec3::zero(); 10];
        self.gyro_history = [Vec3::zero(); 10];
        self.history_index = 0;
        self.history_full = false;
        self.recalibrate();
    }

    pub fn set_position(&mut self, position: Vec3) {
        self.position = position;
    }

    /// Set initial orientation from Euler angles in degrees (only works after calibration).
    pub fn set_orientation(&mut self, euler_deg: Vec3) {
        if self.calibration_state == CalibrationState::Complete {
            // Convert input degrees to radians for quaternion calculation.
            let euler = euler_deg * DEG_TO_RAD;

            // Convert Euler to quaternion (ZYX order).
            let cy = cos(euler.z * 0.5);
            let sy = sin(euler.z * 0.5);
            let cp = cos(euler.y * 0.5);
            let sp = sin(euler.y * 0.5);
            let cr = cos(euler.x * 0.5);
            let sr = sin(euler.x * 0.5);

            self.orientation = Quaternion::new(
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            );
        }
    }

    fn update_sensor_history(&mut self, accel: Vec3, gyro: Vec3) {
        self.accel_history[self.history_index] = accel;
        self.gyro_history[self.history_index] = gyro;
        self.history_index = (self.history_index + 1) % 10;
        if self.history_index == 0 {
            self.history_full = true;
        }
    }

    fn detect_stationary_state(&mut self, accel: Vec3, gyro: Vec3) -> bool {
        // Inputs are already in m/s² and rad/s.
        let accel_magnitude = accel.magnitude();
        let gyro_magnitude = gyro.magnitude();

        let near_gravity = fabs(accel_magnitude - G_STANDARD) < self.params.stationary_threshold;
        let low_rotation = gyro_magnitude < 0.05; // ~3 degrees/second

        if near_gravity && low_rotation {
            self.stationary_counter += 1;
        } else {
            self.stationary_counter = 0;
            self.motion_detected = true;
        }

        self.stationary_counter > 10
    }

    fn update_bias_estimation_enhanced(
        &mut self,
        accel: Vec3,
        gyro: Vec3,
        dt: f64,
        is_stationary: bool,
    ) {
        if !self.history_full {
            return;
        }

        let accel_magnitude = accel.magnitude();
        let expected_gravity = G_STANDARD;

        let accel_learning_rate = if is_stationary {
            self.params.bias_learning_rate * 2.0
        } else if fabs(accel_magnitude - expected_gravity) < 1.0 {
            self.params.bias_learning_rate
        } else {
            self.params.bias_learning_rate * 0.1
        };

        if fabs(accel_magnitude - expected_gravity) < 2.0 {
            let gravity_body = self.orientation.rotate_vector(self.gravity * -1.0);
            let accel_error = accel - gravity_body;
            self.accel_bias = self.accel_bias + accel_error * (accel_learning_rate * dt);
        }

        let gyro_learning_rate = if is_stationary {
            self.params.bias_learning_rate * 5.0
        } else if gyro.magnitude() < 0.02 {
            self.params.bias_learning_rate
        } else {
            self.params.bias_learning_rate * 0.1
        };

        if is_stationary || gyro.magnitude() < 0.02 {
            self.gyro_bias = self.gyro_bias + gyro * (gyro_learning_rate * dt);
        }

        let max_accel_bias = 2.0; // m/s²
        let max_gyro_bias = 0.1; // rad/s

        self.accel_bias.x = self.accel_bias.x.clamp(-max_accel_bias, max_accel_bias);
        self.accel_bias.y = self.accel_bias.y.clamp(-max_accel_bias, max_accel_bias);
        self.accel_bias.z = self.accel_bias.z.clamp(-max_accel_bias, max_accel_bias);

        self.gyro_bias.x = self.gyro_bias.x.clamp(-max_gyro_bias, max_gyro_bias);
        self.gyro_bias.y = self.gyro_bias.y.clamp(-max_gyro_bias, max_gyro_bias);
        self.gyro_bias.z = self.gyro_bias.z.clamp(-max_gyro_bias, max_gyro_bias);
    }

    fn apply_zero_velocity_update(&mut self) {
        self.velocity = self.velocity * self.params.velocity_damping;
        if self.stationary_counter > 100 {
            self.position = self.position * self.params.position_damping;
        }
    }

    fn update_velocity_position_enhanced(&mut self, accel_world: Vec3, dt: f64) {
        let filtered_accel = if self.motion_detected {
            accel_world
        } else {
            accel_world * 0.5
        };
        let avg_accel = (self.prev_accel_world + filtered_accel) * 0.5;
        self.velocity = (self.velocity + avg_accel * dt) * self.params.velocity_damping;
        let avg_velocity = self.velocity - avg_accel * (dt * 0.5);
        self.position = self.position + avg_velocity * dt;

        if avg_accel.magnitude() < 0.05 {
            self.velocity = self.velocity * 0.95;
        }
    }

    fn update_orientation_gyro(&mut self, gyro: Vec3, dt: f64) {
        let angular_velocity_magnitude = gyro.magnitude();
        if angular_velocity_magnitude > 1e-12 {
            let rotation_quat = Quaternion::from_axis_angle(gyro, angular_velocity_magnitude * dt);
            self.orientation = self.orientation.multiply(&rotation_quat).normalize();
        }
    }

    fn should_use_accelerometer(&self, accel: &Vec3) -> bool {
        fabs(accel.magnitude() - G_STANDARD) < self.params.accel_threshold
    }

    fn correct_orientation_accel(&mut self, accel: Vec3) {
        let accel_normalized = (accel * -1.0).normalize();
        let current_down = self.orientation.rotate_vector(Vec3::new(0.0, 0.0, -1.0));
        let correction_axis = current_down.cross(&accel_normalized);
        let correction_angle = acos(current_down.dot(&accel_normalized).clamp(-1.0, 1.0));

        if correction_axis.magnitude() > 1e-12 && correction_angle > 1e-6 {
            let correction_quat = Quaternion::from_axis_angle(
                correction_axis,
                correction_angle * self.params.accel_weight,
            );
            self.orientation = correction_quat.multiply(&self.orientation).normalize();
        }
    }

    pub fn get_biases(&self) -> (Vec3, Vec3) {
        (self.accel_bias, self.gyro_bias)
    }

    pub fn set_biases(&mut self, accel_bias: Vec3, gyro_bias: Vec3) {
        self.accel_bias = accel_bias;
        self.gyro_bias = gyro_bias;
    }

    pub fn force_zero_velocity_update(&mut self) {
        self.velocity = Vec3::zero();
        self.stationary_counter = 100;
    }

    pub fn get_measured_gravity_body(&self) -> Vec3 {
        self.orientation.rotate_vector(self.gravity * -1.0)
    }
}

impl Default for InertialNavigator {
    fn default() -> Self {
        Self::new()
    }
}
