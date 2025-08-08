#![no_std]


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
        libm::sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
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

    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Self {
        let half_angle = angle * 0.5;
        let sin_half = libm::sin(half_angle);
        let cos_half = libm::cos(half_angle);
        let normalized_axis = axis.normalize();

        Self::new(
            cos_half,
            normalized_axis.x * sin_half,
            normalized_axis.y * sin_half,
            normalized_axis.z * sin_half,
        )
    }

    pub fn normalize(&self) -> Self {
        let mag = libm::sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
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
        // Quaternion rotation: q * v * q^-1
        let qv = Quaternion::new(0.0, v.x, v.y, v.z);
        let q_conj = Quaternion::new(self.w, -self.x, -self.y, -self.z);
        let temp = self.multiply(&qv);
        let result = temp.multiply(&q_conj);
        Vec3::new(result.x, result.y, result.z)
    }

    pub fn to_euler(&self) -> Vec3 {
        // Convert to Euler angles (roll, pitch, yaw)
        let roll = libm::atan2(
            2.0 * (self.w * self.x + self.y * self.z),
            1.0 - 2.0 * (self.x * self.x + self.y * self.y),
        );

        let sin_pitch = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if libm::fabs(sin_pitch) >= 1.0 {
            libm::copysign(core::f64::consts::PI / 2.0, sin_pitch)
        } else {
            libm::asin(sin_pitch)
        };

        let yaw = libm::atan2(
            2.0 * (self.w * self.z + self.x * self.y),
            1.0 - 2.0 * (self.y * self.y + self.z * self.z),
        );

        Vec3::new(roll, pitch, yaw)
    }
}

/// Navigation state containing position and orientation
#[derive(Clone, Copy, Debug)]
pub struct NavigationState {
    pub position: Vec3,
    pub rotation: Vec3, // Euler angles in radians
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
    pub accel_threshold: f64,  // Threshold to detect if device is in free-fall or high acceleration
    pub velocity_damping: f64, // Damping factor for velocity drift compensation
    pub position_damping: f64, // Damping factor for position drift compensation
    pub stationary_threshold: f64, // Threshold to detect stationary state
    pub bias_learning_rate: f64, // Learning rate for bias estimation
}

impl Default for FilterParams {
    fn default() -> Self {
        Self {
            accel_weight: 0.05,        // 5% accelerometer, 95% gyroscope for orientation
            gyro_bias_alpha: 0.002,    // Faster bias adaptation
            accel_threshold: 2.0,      // Lower threshold for better stationary detection
            velocity_damping: 0.98,    // Strong velocity damping
            position_damping: 0.99,    // Moderate position damping
            stationary_threshold: 0.5, // Threshold for detecting stationary state
            bias_learning_rate: 0.01,  // Faster bias learning
        }
    }
}

/// Main inertial navigation system
pub struct InertialNavigator {
    // State variables
    orientation: Quaternion,
    velocity: Vec3,
    position: Vec3,

    // Bias estimation
    gyro_bias: Vec3,
    accel_bias: Vec3,

    // Previous values for integration
    prev_accel_world: Vec3,

    // Filter parameters
    params: FilterParams,

    // Gravity vector in world frame
    gravity: Vec3,

    // Stationary detection
    stationary_counter: u32,
    motion_detected: bool,

    // Bias estimation helpers
    accel_history: [Vec3; 10],
    gyro_history: [Vec3; 10],
    history_index: usize,
    history_full: bool,
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
            gravity: Vec3::new(0.0, 0.0, -9.80665), // Standard gravity
            stationary_counter: 0,
            motion_detected: false,
            accel_history: [Vec3::zero(); 10],
            gyro_history: [Vec3::zero(); 10],
            history_index: 0,
            history_full: false,
        }
    }

    /// Update the navigation state with new sensor data
    ///
    /// # Arguments
    /// * `accel` - Accelerometer data in m/s² (body frame)
    /// * `gyro` - Gyroscope data in rad/s (body frame)  
    /// * `dt` - Time delta in seconds
    pub fn update(&mut self, accel: Vec3, gyro: Vec3, dt: f64) {
        if dt <= 0.0 {
            return;
        }

        // Store sensor history for better bias estimation
        self.update_sensor_history(accel, gyro);

        // Detect if device is stationary
        let is_stationary = self.detect_stationary_state(accel, gyro);

        // Enhanced bias estimation
        self.update_bias_estimation_enhanced(accel, gyro, dt, is_stationary);

        let corrected_gyro = gyro - self.gyro_bias;
        let corrected_accel = accel - self.accel_bias;

        // Update orientation using gyroscope (high frequency, low noise)
        self.update_orientation_gyro(corrected_gyro, dt);

        // Correct orientation using accelerometer (low frequency, high noise but absolute)
        if self.should_use_accelerometer(&corrected_accel) {
            self.correct_orientation_accel(corrected_accel);
        }

        // Transform acceleration to world frame and remove gravity
        let accel_world = self.orientation.rotate_vector(corrected_accel) - self.gravity;

        // Apply zero velocity updates when stationary
        if is_stationary {
            self.apply_zero_velocity_update();
        } else {
            // Update velocity and position using enhanced integration
            self.update_velocity_position_enhanced(accel_world, dt);
        }

        self.prev_accel_world = accel_world;
    }

    /// Get current navigation state
    pub fn get_state(&self) -> NavigationState {
        NavigationState {
            position: self.position,
            rotation: self.orientation.to_euler(),
        }
    }

    /// Reset the navigator to initial state
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
    }

    /// Set initial position
    pub fn set_position(&mut self, position: Vec3) {
        self.position = position;
    }

    /// Set initial orientation from Euler angles
    pub fn set_orientation(&mut self, euler: Vec3) {
        // Convert Euler to quaternion (ZYX order)
        let cy = libm::cos(euler.z * 0.5);
        let sy = libm::sin(euler.z * 0.5);
        let cp = libm::cos(euler.y * 0.5);
        let sp = libm::sin(euler.y * 0.5);
        let cr = libm::cos(euler.x * 0.5);
        let sr = libm::sin(euler.x * 0.5);

        self.orientation = Quaternion::new(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        );
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
        let accel_magnitude = accel.magnitude();
        let gyro_magnitude = gyro.magnitude();

        // Check if acceleration is close to gravity and gyro is low
        let near_gravity = libm::fabs(accel_magnitude - 9.80665) < self.params.stationary_threshold;
        let low_rotation = gyro_magnitude < 0.05; // ~3 degrees/second

        if near_gravity && low_rotation {
            self.stationary_counter += 1;
        } else {
            self.stationary_counter = 0;
            self.motion_detected = true;
        }

        // Consider stationary after 10 consecutive stationary readings
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
            return; // Need some history first
        }

        // Enhanced accelerometer bias estimation
        let accel_magnitude = accel.magnitude();
        let expected_gravity = 9.80665;

        // More aggressive bias correction when stationary
        let accel_learning_rate = if is_stationary {
            self.params.bias_learning_rate * 2.0
        } else if libm::fabs(accel_magnitude - expected_gravity) < 1.0 {
            self.params.bias_learning_rate
        } else {
            self.params.bias_learning_rate * 0.1
        };

        if libm::fabs(accel_magnitude - expected_gravity) < 2.0 {
            let gravity_body = self.orientation.rotate_vector(self.gravity * -1.0);
            let accel_error = accel - gravity_body;
            self.accel_bias = self.accel_bias + accel_error * (accel_learning_rate * dt);
        }

        // Enhanced gyroscope bias estimation
        let gyro_learning_rate = if is_stationary {
            self.params.bias_learning_rate * 5.0 // Very aggressive when stationary
        } else if gyro.magnitude() < 0.02 {
            self.params.bias_learning_rate
        } else {
            self.params.bias_learning_rate * 0.1
        };

        if is_stationary || gyro.magnitude() < 0.02 {
            self.gyro_bias = self.gyro_bias + gyro * (gyro_learning_rate * dt);
        }

        // Limit bias values to reasonable ranges
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
        // Zero velocity update - aggressively correct velocity and position drift
        self.velocity = self.velocity * self.params.velocity_damping;

        // Also apply some position correction toward origin when stationary for long periods
        if self.stationary_counter > 100 {
            // After ~1 second of stationary state
            self.position = self.position * self.params.position_damping;
        }
    }

    fn update_velocity_position_enhanced(&mut self, accel_world: Vec3, dt: f64) {
        // Apply high-pass filter to remove low-frequency drift
        let filtered_accel = if self.motion_detected {
            accel_world
        } else {
            // Reduce acceleration magnitude when motion is questionable
            accel_world * 0.5
        };

        // Trapezoidal integration for better accuracy
        let avg_accel = (self.prev_accel_world + filtered_accel) * 0.5;

        // Update velocity with damping
        self.velocity = (self.velocity + avg_accel * dt) * self.params.velocity_damping;

        // Update position using velocity at midpoint
        let avg_velocity = self.velocity - avg_accel * (dt * 0.5);
        self.position = self.position + avg_velocity * dt;

        // Additional drift compensation based on acceleration magnitude
        if avg_accel.magnitude() < 0.05 {
            // Very low acceleration
            self.velocity = self.velocity * 0.95; // Extra damping
        }
    }

    fn update_bias_estimation(&mut self, accel: Vec3, gyro: Vec3, dt: f64) {
        // Simple bias estimation - assumes periods of low acceleration indicate bias
        let accel_magnitude = accel.magnitude();
        let expected_gravity = 9.80665;

        // If close to 1g, we're likely stationary - estimate accelerometer bias
        if libm::fabs(accel_magnitude - expected_gravity) < 0.5 {
            let gravity_body = self.orientation.rotate_vector(self.gravity * -1.0);
            let accel_error = accel - gravity_body;
            self.accel_bias = self.accel_bias + accel_error * (self.params.gyro_bias_alpha * dt);
        }

        // Gyroscope bias estimation during low rotation periods
        if gyro.magnitude() < 0.1 {
            // Less than ~6 deg/s
            self.gyro_bias = self.gyro_bias + gyro * (self.params.gyro_bias_alpha * dt);
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
        let accel_magnitude = accel.magnitude();
        // Use accelerometer correction only when acceleration is close to gravity
        // (indicating we're not in free-fall or high acceleration)
        libm::fabs(accel_magnitude - 9.80665) < self.params.accel_threshold
    }

    fn correct_orientation_accel(&mut self, accel: Vec3) {
        // Get gravity direction from accelerometer (inverted because accel opposes gravity)
        let accel_normalized = (accel * -1.0).normalize();
        let gravity_world = self.gravity.normalize() * -1.0;

        // Calculate rotation needed to align current "down" with measured "down"
        let current_down = self.orientation.rotate_vector(Vec3::new(0.0, 0.0, -1.0));

        // Cross product gives rotation axis, dot product gives angle
        let correction_axis = current_down.cross(&accel_normalized);
        let correction_angle = libm::acos(current_down.dot(&accel_normalized).clamp(-1.0, 1.0));

        if correction_axis.magnitude() > 1e-12 && correction_angle > 1e-6 {
            let correction_quat = Quaternion::from_axis_angle(
                correction_axis,
                correction_angle * self.params.accel_weight,
            );
            self.orientation = correction_quat.multiply(&self.orientation).normalize();
        }
    }

    fn update_velocity_position(&mut self, accel_world: Vec3, dt: f64) {
        // Trapezoidal integration for better accuracy
        let avg_accel = (self.prev_accel_world + accel_world) * 0.5;

        // Update velocity
        self.velocity = self.velocity + avg_accel * dt;

        // Update position using velocity at midpoint
        let avg_velocity = self.velocity - avg_accel * (dt * 0.5);
        self.position = self.position + avg_velocity * dt;

        // Apply damping to prevent drift in stationary conditions
        if avg_accel.magnitude() < 0.1 {
            self.velocity = self.velocity * 0.99; // Light damping
        }
    }

    /// Get current estimated sensor biases for debugging
    pub fn get_biases(&self) -> (Vec3, Vec3) {
        (self.accel_bias, self.gyro_bias)
    }

    /// Manually set sensor biases if known
    pub fn set_biases(&mut self, accel_bias: Vec3, gyro_bias: Vec3) {
        self.accel_bias = accel_bias;
        self.gyro_bias = gyro_bias;
    }

    /// Force a zero velocity update (useful when you know the device is stationary)
    pub fn force_zero_velocity_update(&mut self) {
        self.velocity = Vec3::zero();
        self.stationary_counter = 100; // Mark as stationary
    }
}

impl Default for InertialNavigator {
    fn default() -> Self {
        Self::new()
    }
}
