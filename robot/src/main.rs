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
