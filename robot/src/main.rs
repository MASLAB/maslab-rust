/*
use raven::{MotorChannel, MotorMode, Raven};
use std::thread;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Initializing Raven motor controller...");
    let mut raven = Raven::new(None, Duration::from_millis(1))?;

    let channel1 = MotorChannel::CH1;
    let channel3 = MotorChannel::CH3;

    println!("Setting up position control mode...");
    raven.set_motor_mode(channel1, MotorMode::Position, 0);
    raven.set_motor_mode(channel3, MotorMode::Position, 0);
    raven.set_motor_torque_factor(channel1, 10.0, 0);
    raven.set_motor_max_current(channel3, 1.5, 0);
    raven.set_motor_pid(channel1, 30.0, 10.0, 2.0, 50.0, 0);
    raven.set_motor_pid(channel3, 30.0, 10.0, 2.0, 50.0, 0);

    println!("Motor PID settings: {:?}", raven.get_motor_pid(channel3, 0));

    raven.set_motor_target(channel1, -640.0, 0);
    raven.set_motor_target(channel3, 640.0, 0);

    let mut current_time = Instant::now();
    let mut current_encoder = raven.get_motor_encoder(channel3, 0).unwrap_or(0);

    println!("Starting motor control loop (Ctrl+C to exit)...");
    loop {
        let new_time = Instant::now();
        if let Some(new_encoder) = raven.get_motor_encoder(channel3, 0) {
            if let Some(raven_vel) = raven.get_motor_velocity(channel3, 0) {
                let elapsed = (new_time - current_time).as_secs_f32();
                let cal_vel = (new_encoder - current_encoder) as f32 / elapsed;

                println!("Raven Ecd: {}", new_encoder);
                println!("Raven Vel: {}", raven_vel);
                println!("Calc Vel:  {}", cal_vel);
                println!("Diff: {}", raven_vel - cal_vel);
                println!("---");

                current_time = new_time;
                current_encoder = new_encoder;
            }
        }
        thread::sleep(Duration::from_millis(100));
    }
}
*/

use bno08x::{BNO08x, SensorData, quaternion_to_euler};
use rppal::i2c::I2c;
use std::error::Error;
use std::thread;
use std::time::Duration;

const BNO08X_ADDRESS_A: u16 = 0x4A;

fn main() -> Result<(), Box<dyn Error>> {
    println!("=== BNO08x Debugging ===");
    println!("Step 1: Testing I2C connection...");

    println!("\nStep 2: Initializing BNO08x...");
    let mut imu = BNO08x::new(BNO08X_ADDRESS_A)?;
    imu.init()?;

    println!("✓ BNO08x initialized successfully!");

    println!("\nStep 3: Enabling sensors...");
    imu.enable_game_rotation_vector(20000)?;
    println!("✓ Game rotation vector enabled (50Hz)");
    thread::sleep(Duration::from_millis(100));

    imu.enable_accelerometer(20000)?;
    println!("✓ Accelerometer enabled (50Hz)");
    thread::sleep(Duration::from_millis(100));

    println!("\nStep 4: Reading sensor data (Ctrl+C to exit)...");
    println!("Waiting for data...\n");

    let mut packet_count = 0;
    let mut quat_count = 0;
    let mut accel_count = 0;
    let mut other_count = 0;

    loop {
        match imu.read_sensor_data() {
            Ok(Some(data)) => {
                packet_count += 1;

                match data {
                    SensorData::Quaternion(quat) => {
                        quat_count += 1;
                        let (roll, pitch, yaw) = quaternion_to_euler(&quat);
                        println!(
                            "Quaternion #{} - i: {:.4}, j: {:.4}, k: {:.4}, real: {:.4}",
                            quat_count, quat.i, quat.j, quat.k, quat.real
                        );
                        println!(
                            "  Euler - Roll: {:.2}°, Pitch: {:.2}°, Yaw: {:.2}°",
                            roll.to_degrees(),
                            pitch.to_degrees(),
                            yaw.to_degrees()
                        );
                        println!("  Accuracy: {:.4}", quat.accuracy);
                    }
                    SensorData::Accelerometer(accel) => {
                        accel_count += 1;
                        println!(
                            "Accel #{} - X: {:.3}, Y: {:.3}, Z: {:.3} m/s²",
                            accel_count, accel.x, accel.y, accel.z
                        );
                    }
                    SensorData::Gyroscope(gyro) => {
                        other_count += 1;
                        println!(
                            "Gyro - X: {:.3}, Y: {:.3}, Z: {:.3} rad/s",
                            gyro.x, gyro.y, gyro.z
                        );
                    }
                    SensorData::LinearAcceleration(lin) => {
                        other_count += 1;
                        println!(
                            "Linear Accel - X: {:.3}, Y: {:.3}, Z: {:.3} m/s²",
                            lin.x, lin.y, lin.z
                        );
                    }
                    _ => {
                        other_count += 1;
                        println!("Other sensor data received");
                    }
                }
                println!("---");
            }
            Ok(None) => {
                // No data available, this is normal
            }
            Err(e) => {
                println!("Error reading sensor: {}", e);
            }
        }

        // Print stats every 100 iterations
        if packet_count > 0 && packet_count % 100 == 0 {
            println!(
                "Stats: {} packets ({} quat, {} accel, {} other)",
                packet_count, quat_count, accel_count, other_count
            );
        }

        thread::sleep(Duration::from_millis(10));
    }
}
