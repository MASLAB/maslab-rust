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

use rppal::i2c::I2c;
use std::error::Error;
use std::thread;
use std::time::Duration;

use bno08x::{BNO08x, Quaternion, SensorData, Vector3, quaternion_to_euler};

fn main() -> Result<(), Box<dyn Error>> {
    println!("=== BNO08x Debugging ===");
    println!("Step 1: Initializing BNO08x...");

    let mut imu = BNO08x::new()?;
    imu.init()?;
    println!("✓ BNO08x initialized successfully!");

    println!("Step 2: Enabling sensors at 50Hz (20000 µs)...");

    imu.enable_game_rotation(20000)?;
    println!("✓ Game rotation vector enabled");

    imu.enable_linear_accel(20000)?;
    println!("✓ Linear acceleration enabled");

    println!("\nStep 3: Reading sensor data (Ctrl+C to exit)...\n");

    let mut packet_count = 0;
    let mut quat_count = 0;
    let mut lin_acc_count = 0;
    let mut other_count = 0;

    loop {
        if let Some(data) = imu.read()? {
            packet_count += 1;

            match data {
                SensorData::Quaternion(quat) => {
                    quat_count += 1;
                    let (roll, pitch, yaw) = quaternion_to_euler(&quat);
                    println!(
                        "[Quat #{}] i:{:.4} j:{:.4} k:{:.4} real:{:.4} | Roll:{:.2}° Pitch:{:.2}° Yaw:{:.2}° | Acc:{:} ",
                        quat_count,
                        quat.i,
                        quat.j,
                        quat.k,
                        quat.real,
                        roll.to_degrees(),
                        pitch.to_degrees(),
                        yaw.to_degrees(),
                        quat.accuracy
                    );
                }

                SensorData::LinearAccel(accel) => {
                    lin_acc_count += 1;
                    println!(
                        "[LinAccel #{}] X:{:.3} Y:{:.3} Z:{:.3} m/s²",
                        lin_acc_count, accel.x, accel.y, accel.z
                    );
                }

                _ => {
                    other_count += 1;
                    println!("Other sensor data received");
                }
            }
        }

        // Print stats every 100 packets
        if packet_count > 0 && packet_count % 100 == 0 {
            println!(
                "Stats: {} packets ({} quat, {} linear accel, {} other)",
                packet_count, quat_count, lin_acc_count, other_count
            );
        }

        thread::sleep(Duration::from_millis(10));
    }
}
