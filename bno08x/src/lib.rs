use rppal::i2c::I2c;
use std::{error::Error, thread, time::Duration};

const BNO08X_ADDR: u16 = 0x4A;

// SHTP Channels
const CH_EXEC: u8 = 0;
const CH_CONTROL: u8 = 2;
const CH_INPUT: u8 = 3;

// Executable reports
const REPORT_RESET: u8 = 0x01;
const REPORT_PRODUCT_ID_REQ: u8 = 0xF9;
const REPORT_PRODUCT_ID_RESP: u8 = 0xF8;

// Control
const REPORT_SET_FEATURE: u8 = 0xFD;

// Sensors
const REPORT_GAME_ROTATION_VECTOR: u8 = 0x08;
const REPORT_LINEAR_ACCEL: u8 = 0x04;

/*

// SHTP (Sensor Hub Transport Protocol) constants
const SHTP_REPORT_COMMAND_RESPONSE: u8 = 0xF1;
const SHTP_REPORT_COMMAND_REQUEST: u8 = 0xF2;
const SHTP_REPORT_FRS_READ_RESPONSE: u8 = 0xF3;
const SHTP_REPORT_PRODUCT_ID_RESPONSE: u8 = 0xF8;
const SHTP_REPORT_BASE_TIMESTAMP: u8 = 0xFB;
const SHTP_REPORT_SET_FEATURE_COMMAND: u8 = 0xFD;

// Report IDs
const REPORT_ACCELEROMETER: u8 = 0x01;
const REPORT_GYROSCOPE: u8 = 0x02;
const REPORT_MAGNETOMETER: u8 = 0x03;
const REPORT_LINEAR_ACCELERATION: u8 = 0x04;
const REPORT_ROTATION_VECTOR: u8 = 0x05;
const REPORT_GRAVITY: u8 = 0x06;
const REPORT_GAME_ROTATION_VECTOR: u8 = 0x08;
const REPORT_GEOMAGNETIC_ROTATION_VECTOR: u8 = 0x09;

*/

#[derive(Debug, Copy, Clone)]
pub struct Quaternion {
    pub i: f32,
    pub j: f32,
    pub k: f32,
    pub real: f32,
    pub accuracy: u8,
}

#[derive(Debug, Copy, Clone)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug)]
pub enum SensorData {
    Quaternion(Quaternion),
    LinearAccel(Vector3),
}

pub struct BNO08x {
    i2c: I2c,
    seq: [u8; 6],
}

impl BNO08x {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        let mut i2c = I2c::new()?;
        i2c.set_slave_address(BNO08X_ADDR)?;

        Ok(Self { i2c, seq: [0; 6] })
    }

    pub fn init(&mut self) -> Result<(), Box<dyn Error>> {
        thread::sleep(Duration::from_millis(300));
        self.reset()?;
        self.wait_for_advertise()?;
        self.read_product_id()?;
        Ok(())
    }

    fn reset(&mut self) -> Result<(), Box<dyn Error>> {
        self.send_packet(CH_EXEC, &[REPORT_RESET, 0x01])?;
        thread::sleep(Duration::from_millis(500));
        Ok(())
    }

    fn wait_for_advertise(&mut self) -> Result<(), Box<dyn Error>> {
        for _ in 0..50 {
            if let Some(_) = self.receive_packet()? {
                return Ok(());
            }
            thread::sleep(Duration::from_millis(20));
        }
        Err("No advertise packet".into())
    }

    fn read_product_id(&mut self) -> Result<(), Box<dyn Error>> {
        self.send_packet(CH_EXEC, &[REPORT_PRODUCT_ID_REQ])?;

        for _ in 0..20 {
            if let Some(pkt) = self.receive_packet()? {
                if pkt[0] == REPORT_PRODUCT_ID_RESP {
                    println!("Product ID OK");
                    return Ok(());
                }
            }
            thread::sleep(Duration::from_millis(20));
        }

        Err("Product ID failed".into())
    }

    pub fn enable_game_rotation(&mut self, interval_us: u32) -> Result<(), Box<dyn Error>> {
        self.set_feature(REPORT_GAME_ROTATION_VECTOR, interval_us)
    }

    pub fn enable_linear_accel(&mut self, interval_us: u32) -> Result<(), Box<dyn Error>> {
        self.set_feature(REPORT_LINEAR_ACCEL, interval_us)
    }

    fn set_feature(&mut self, report: u8, interval_us: u32) -> Result<(), Box<dyn Error>> {
        let mut payload = vec![REPORT_SET_FEATURE, report, 0x00, 0x00, 0x00];
        payload.extend(interval_us.to_le_bytes());
        payload.extend([0, 0, 0, 0]); // batch interval
        payload.extend([0, 0, 0, 0]); // specific config

        self.send_packet(CH_CONTROL, &payload)?;
        Ok(())
    }

    fn send_packet(&mut self, channel: u8, payload: &[u8]) -> Result<(), Box<dyn Error>> {
        let mut packet = vec![0, 0, channel, self.next_seq(channel)];
        packet.extend_from_slice(payload);

        let len = packet.len() as u16;
        packet[0..2].copy_from_slice(&len.to_le_bytes());

        self.i2c.write(&packet)?;
        Ok(())
    }

    fn next_seq(&mut self, ch: u8) -> u8 {
        let s = self.seq[ch as usize];
        self.seq[ch as usize] = s.wrapping_add(1);
        s
    }

    fn receive_packet(&mut self) -> Result<Option<Vec<u8>>, Box<dyn Error>> {
        let mut full = Vec::new();

        loop {
            let mut hdr = [0u8; 4];
            if self.i2c.read(&mut hdr).is_err() {
                return Ok(None);
            }

            let len = u16::from_le_bytes([hdr[0], hdr[1]]);
            let cont = (len & 0x8000) != 0;
            let size = (len & 0x7FFF) as usize;

            if size < 4 || size > 1024 {
                return Ok(None);
            }

            let mut buf = vec![0u8; size - 4];
            self.i2c.read(&mut buf)?;

            full.extend_from_slice(&buf);

            if !cont {
                break;
            }
        }

        Ok(Some(full))
    }

    pub fn read(&mut self) -> Result<Option<SensorData>, Box<dyn Error>> {
        let pkt = match self.receive_packet()? {
            Some(p) => p,
            None => return Ok(None),
        };

        match pkt[0] {
            REPORT_GAME_ROTATION_VECTOR => {
                let acc = pkt[2] & 0x03;
                let i = i16::from_le_bytes([pkt[5], pkt[6]]) as f32 / 16384.0;
                let j = i16::from_le_bytes([pkt[7], pkt[8]]) as f32 / 16384.0;
                let k = i16::from_le_bytes([pkt[9], pkt[10]]) as f32 / 16384.0;
                let r = i16::from_le_bytes([pkt[11], pkt[12]]) as f32 / 16384.0;

                Ok(Some(SensorData::Quaternion(Quaternion {
                    i,
                    j,
                    k,
                    real: r,
                    accuracy: acc,
                })))
            }

            REPORT_LINEAR_ACCEL => {
                let x = i16::from_le_bytes([pkt[5], pkt[6]]) as f32 / 1024.0;
                let y = i16::from_le_bytes([pkt[7], pkt[8]]) as f32 / 1024.0;
                let z = i16::from_le_bytes([pkt[9], pkt[10]]) as f32 / 1024.0;

                Ok(Some(SensorData::LinearAccel(Vector3 { x, y, z })))
            }

            _ => Ok(None),
        }
    }
}

pub fn quaternion_to_euler(quat: &Quaternion) -> (f32, f32, f32) {
    // Extract squared terms
    let sqw = quat.real * quat.real;
    let sqi = quat.i * quat.i;
    let sqj = quat.j * quat.j;
    let sqk = quat.k * quat.k;

    // Roll (x-axis rotation)
    let roll = (2.0 * (quat.real * quat.i + quat.j * quat.k)).atan2(1.0 - 2.0 * (sqi + sqj));

    // Pitch (y-axis rotation)
    let pitch = (2.0 * (quat.real * quat.j - quat.k * quat.i)).asin();

    // Yaw (z-axis rotation)
    let yaw = (2.0 * (quat.real * quat.k + quat.i * quat.j)).atan2(1.0 - 2.0 * (sqj + sqk));

    (roll, pitch, yaw)
}
/*
    fn send_packet(&mut self, channel: u8, payload: &[u8]) -> Result<(), Box<dyn Error>> {
        let mut packet = vec![0, 0, channel, self.next_seq(channel)];
        packet.extend_from_slice(payload);

        let len = packet.len() as u16;
        packet[0..2].copy_from_slice(&len.to_le_bytes());

        self.i2c.write(&packet)?;
        Ok(())
    }

    fn next_seq(&mut self, ch: u8) -> u8 {
        let s = self.seq[ch as usize];
        self.seq[ch as usize] = s.wrapping_add(1);
        s
    }

impl BNO08x {
    pub fn new(address: u16) -> Result<Self, Box<dyn Error>> {
        let mut i2c = I2c::new()?;
        i2c.set_slave_address(address)?;

        Ok(BNO08x {
            i2c,
            sequence_number: [0; 6],
        })
    }

    pub fn init(&mut self) -> Result<(), Box<dyn Error>> {
        // Wait for device to be ready
        thread::sleep(Duration::from_millis(300));

        // Clear any existing data
        let _ = self.receive_packet();

        // Soft reset
        self.soft_reset()?;
        thread::sleep(Duration::from_millis(500));

        // Wait for and read advertisement/unsolicited packets after reset
        println!("Waiting for reset complete...");
        for _ in 0..50 {
            if let Ok(Some(packet)) = self.receive_packet() {
                println!("Received packet after reset, length: {}", packet.len());
                if packet.len() > 4 {
                    let channel = packet[2];
                    println!("  Channel: {}", channel);
                    if packet.len() > 4 {
                        println!(
                            "  Data: {:02X?}",
                            &packet[4..std::cmp::min(10, packet.len())]
                        );
                    }
                }
            }
            thread::sleep(Duration::from_millis(20));
        }

        // Read product ID to verify communication
        println!("Reading product ID...");
        match self.read_product_id() {
            Ok(product_id) => {
                println!("BNO08x Product ID: {:?}", product_id);
            }
            Err(e) => {
                println!("Warning: Could not read product ID: {}", e);
            }
        }

        Ok(())
    }

    fn soft_reset(&mut self) -> Result<(), Box<dyn Error>> {
        let command = [
            11,
            0, // Length
            0, // Channel 0 (command)
            self.get_sequence_number(0),
            0x01, // Reset command
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ];

        self.send_packet(&command)?;
        Ok(())
    }

    fn read_product_id(&mut self) -> Result<ProductId, Box<dyn Error>> {
        // Request product ID report
        let command = [
            6,
            0, // Length
            0, // Channel 0
            self.get_sequence_number(0),
            SHTP_REPORT_PRODUCT_ID_RESPONSE,
            0x00,
        ];

        self.send_packet(&command)?;
        thread::sleep(Duration::from_millis(50));

        // Read response
        if let Some(packet) = self.receive_packet()? {
            if packet.len() >= 16 {
                let sw_major = packet[2];
                let sw_minor = packet[3];
                let sw_patch = u16::from_le_bytes([packet[12], packet[13]]);
                let sw_build = u32::from_le_bytes([packet[4], packet[5], packet[6], packet[7]]);

                return Ok(ProductId {
                    sw_major,
                    sw_minor,
                    sw_patch,
                    sw_build,
                });
            }
        }

        Err("Failed to read product ID".into())
    }

    pub fn enable_rotation_vector(&mut self, interval_us: u32) -> Result<(), Box<dyn Error>> {
        println!("Enabling rotation vector with interval: {} us", interval_us);
        self.set_feature_command(REPORT_ROTATION_VECTOR, interval_us, 0)?;

        // Wait for acknowledgment
        thread::sleep(Duration::from_millis(100));

        // Try to read response
        for _ in 0..10 {
            if let Ok(Some(packet)) = self.receive_packet() {
                println!("Response packet received, length: {}", packet.len());
            }
            thread::sleep(Duration::from_millis(10));
        }

        Ok(())
    }

    pub fn enable_game_rotation_vector(&mut self, interval_us: u32) -> Result<(), Box<dyn Error>> {
        println!(
            "Enabling game rotation vector with interval: {} us",
            interval_us
        );
        self.set_feature_command(REPORT_GAME_ROTATION_VECTOR, interval_us, 0)?;

        // Wait for acknowledgment
        thread::sleep(Duration::from_millis(100));

        // Try to read response
        for _ in 0..10 {
            if let Ok(Some(packet)) = self.receive_packet() {
                println!("Response packet received, length: {}", packet.len());
            }
            thread::sleep(Duration::from_millis(10));
        }

        Ok(())
    }

    pub fn enable_accelerometer(&mut self, interval_us: u32) -> Result<(), Box<dyn Error>> {
        println!("Enabling accelerometer with interval: {} us", interval_us);
        self.set_feature_command(REPORT_ACCELEROMETER, interval_us, 0)?;
        thread::sleep(Duration::from_millis(50));
        Ok(())
    }

    pub fn enable_linear_acceleration(&mut self, interval_us: u32) -> Result<(), Box<dyn Error>> {
        self.set_feature_command(REPORT_LINEAR_ACCELERATION, interval_us, 0)
    }

    pub fn enable_gyroscope(&mut self, interval_us: u32) -> Result<(), Box<dyn Error>> {
        self.set_feature_command(REPORT_GYROSCOPE, interval_us, 0)
    }

    pub fn enable_magnetometer(&mut self, interval_us: u32) -> Result<(), Box<dyn Error>> {
        self.set_feature_command(REPORT_MAGNETOMETER, interval_us, 0)
    }

    fn set_feature_command(
        &mut self,
        report_id: u8,
        interval_us: u32,
        specific_config: u32,
    ) -> Result<(), Box<dyn Error>> {
        let interval_bytes = interval_us.to_le_bytes();
        let config_bytes = specific_config.to_le_bytes();

        let command = [
            17,
            0, // Length
            2, // Channel 2 (control)
            self.get_sequence_number(2),
            SHTP_REPORT_SET_FEATURE_COMMAND,
            report_id,
            0x00, // Feature flags
            0x00, // Change sensitivity (relative)
            0x00, // Change sensitivity (absolute)
            interval_bytes[0],
            interval_bytes[1],
            interval_bytes[2],
            interval_bytes[3],
            0x00,
            0x00,
            0x00,
            0x00, // Batch interval
            config_bytes[0],
            config_bytes[1],
            config_bytes[2],
            config_bytes[3],
        ];

        self.send_packet(&command)?;
        thread::sleep(Duration::from_millis(10));
        Ok(())
    }

    pub fn read_sensor_data(&mut self) -> Result<Option<SensorData>, Box<dyn Error>> {
        if let Some(packet) = self.receive_packet()? {
            // Minimum packet: 4 byte header + at least 1 byte data
            if packet.len() < 5 {
                println!(
                    "DEBUG: Packet too short: {} bytes (need at least 5)",
                    packet.len()
                );
                return Ok(None);
            }

            let channel = packet[2];

            // Some packets might be on channel 0 (executable/device), we want channel 3 (input reports)
            // But let's check what we're getting
            if packet.len() >= 5 {
                let report_id = packet[4];
                println!(
                    "DEBUG: Channel = {}, Report ID = 0x{:02X}, Length = {}",
                    channel,
                    report_id,
                    packet.len()
                );
            }

            // Channel 3 = Input reports (sensor data)
            // Channel 2 = Control reports
            // Channel 0 = Executable/device channel
            if channel == 3 && packet.len() >= 5 {
                let report_id = packet[4];

                match report_id {
                    REPORT_ROTATION_VECTOR
                    | REPORT_GAME_ROTATION_VECTOR
                    | REPORT_GEOMAGNETIC_ROTATION_VECTOR => {
                        println!(
                            "DEBUG: Processing quaternion report, packet length: {}",
                            packet.len()
                        );
                        // Quaternion needs: report_id(1) + sequence(1) + status(1) + delay(2) + i,j,k,real(8) + accuracy(2) = 15 bytes minimum
                        // With 4 byte header = 19 bytes total, but data starts at index 4
                        if packet.len() >= 19 {
                            let quat = self.parse_quaternion(&packet[5..])?;
                            return Ok(Some(SensorData::Quaternion(quat)));
                        } else {
                            println!(
                                "WARNING: Quaternion packet too short: {} bytes (need 19+)",
                                packet.len()
                            );
                            println!("  Packet data: {:02X?}", packet);
                        }
                    }
                    REPORT_ACCELEROMETER => {
                        println!(
                            "DEBUG: Processing accelerometer report, packet length: {}",
                            packet.len()
                        );
                        // Accel needs: report_id(1) + sequence(1) + status(1) + delay(2) + x,y,z(6) = 11 bytes minimum
                        if packet.len() >= 15 {
                            let accel = self.parse_vector3(&packet[5..])?;
                            return Ok(Some(SensorData::Accelerometer(accel)));
                        } else {
                            println!(
                                "WARNING: Accelerometer packet too short: {} bytes",
                                packet.len()
                            );
                        }
                    }
                    REPORT_GYROSCOPE => {
                        if packet.len() >= 15 {
                            let gyro = self.parse_vector3(&packet[5..])?;
                            return Ok(Some(SensorData::Gyroscope(gyro)));
                        }
                    }
                    REPORT_MAGNETOMETER => {
                        if packet.len() >= 15 {
                            let mag = self.parse_vector3(&packet[5..])?;
                            return Ok(Some(SensorData::Magnetometer(mag)));
                        }
                    }
                    REPORT_LINEAR_ACCELERATION => {
                        if packet.len() >= 15 {
                            let lin_accel = self.parse_vector3(&packet[5..])?;
                            return Ok(Some(SensorData::LinearAcceleration(lin_accel)));
                        }
                    }
                    _ => {
                        println!(
                            "DEBUG: Unknown report ID: 0x{:02X}, packet: {:02X?}",
                            report_id,
                            &packet[..std::cmp::min(16, packet.len())]
                        );
                    }
                }
            } else if channel != 3 {
                println!("DEBUG: Ignoring channel {} (not input reports)", channel);
            }
        }

        Ok(None)
    }

    fn parse_quaternion(&self, data: &[u8]) -> Result<Quaternion, Box<dyn Error>> {
        if data.len() < 14 {
            return Err("Insufficient data for quaternion".into());
        }

        let i = i16::from_le_bytes([data[0], data[1]]) as f32 / 16384.0;
        let j = i16::from_le_bytes([data[2], data[3]]) as f32 / 16384.0;
        let k = i16::from_le_bytes([data[4], data[5]]) as f32 / 16384.0;
        let real = i16::from_le_bytes([data[6], data[7]]) as f32 / 16384.0;
        let accuracy = u16::from_le_bytes([data[8], data[9]]) as f32 / 16384.0;

        Ok(Quaternion {
            i,
            j,
            k,
            real,
            accuracy,
        })
    }

    fn parse_vector3(&self, data: &[u8]) -> Result<Vector3, Box<dyn Error>> {
        if data.len() < 6 {
            return Err("Insufficient data for vector3".into());
        }

        let x = i16::from_le_bytes([data[0], data[1]]) as f32 / 100.0;
        let y = i16::from_le_bytes([data[2], data[3]]) as f32 / 100.0;
        let z = i16::from_le_bytes([data[4], data[5]]) as f32 / 100.0;

        Ok(Vector3 { x, y, z })
    }

    fn send_packet(&mut self, data: &[u8]) -> Result<(), Box<dyn Error>> {
        self.i2c.write(data)?;
        Ok(())
    }

    fn receive_packet(&mut self) -> Result<Option<Vec<u8>>, Box<dyn Error>> {
        // Read header (4 bytes)
        let mut header = [0u8; 4];
        match self.i2c.read(&mut header) {
            Ok(_) => {}
            Err(e) => {
                // This is normal when no data is available
                return Ok(None);
            }
        }

        // Check if data is available
        let length = u16::from_le_bytes([header[0], header[1]]) & 0x7FFF;

        // Debug: Show raw packets
        if length > 0 && length < 1024 {
            println!(
                "DEBUG: Header = {:02X} {:02X} {:02X} {:02X}, Length = {}",
                header[0], header[1], header[2], header[3], length
            );
        }

        if length == 0 || length == 0x7FFF {
            return Ok(None);
        }

        if length > 1024 {
            println!("WARNING: Invalid packet length: {}", length);
            return Ok(None);
        }

        // Read full packet
        let mut packet = vec![0u8; length as usize];
        packet[..4].copy_from_slice(&header);

        if length > 4 {
            self.i2c.read(&mut packet[4..])?;
        }

        // Debug: Show full packet contents
        println!(
            "DEBUG: Full packet: {:02X?}",
            &packet[..std::cmp::min(20, packet.len())]
        );

        Ok(Some(packet))
    }

    fn get_sequence_number(&mut self, channel: usize) -> u8 {
        let seq = self.sequence_number[channel];
        self.sequence_number[channel] = seq.wrapping_add(1);
        seq
    }
}

#[derive(Debug)]
pub struct ProductId {
    pub sw_major: u8,
    pub sw_minor: u8,
    pub sw_patch: u16,
    pub sw_build: u32,
}

#[derive(Debug)]
pub enum SensorData {
    Quaternion(Quaternion),
    Accelerometer(Vector3),
    Gyroscope(Vector3),
    Magnetometer(Vector3),
    LinearAcceleration(Vector3),
}

// Helper function to convert quaternion to Euler angles
pub fn quaternion_to_euler(quat: &Quaternion) -> (f32, f32, f32) {
    let sqr = quat.real * quat.real;
    let sqi = quat.i * quat.i;
    let sqj = quat.j * quat.j;
    let sqk = quat.k * quat.k;

    // Roll (x-axis rotation)
    let roll = (2.0 * (quat.real * quat.i + quat.j * quat.k)).atan2(1.0 - 2.0 * (sqi + sqj));

    // Pitch (y-axis rotation)
    let pitch = (2.0 * (quat.real * quat.j - quat.k * quat.i)).asin();

    // Yaw (z-axis rotation)
    let yaw = (2.0 * (quat.real * quat.k + quat.i * quat.j)).atan2(1.0 - 2.0 * (sqj + sqk));

    (roll, pitch, yaw)
}

// Robot IMU wrapper
pub struct RobotIMU {
    imu: BNO08x,
    pub heading: f32,
    pub pitch: f32,
    pub roll: f32,
    pub linear_accel: Vector3,
}

impl RobotIMU {
    pub fn new(address: u16) -> Result<Self, Box<dyn Error>> {
        let mut imu = BNO08x::new(address)?;
        imu.init()?;

        // Enable sensors at 100Hz (10000 microseconds)
        imu.enable_game_rotation_vector(10000)?;
        imu.enable_linear_acceleration(10000)?;

        Ok(RobotIMU {
            imu,
            heading: 0.0,
            pitch: 0.0,
            roll: 0.0,
            linear_accel: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        })
    }

    pub fn update(&mut self) -> Result<bool, Box<dyn Error>> {
        let mut updated = false;

        if let Some(data) = self.imu.read_sensor_data()? {
            match data {
                SensorData::Quaternion(quat) => {
                    let (roll, pitch, yaw) = quaternion_to_euler(&quat);
                    self.roll = roll.to_degrees();
                    self.pitch = pitch.to_degrees();
                    self.heading = yaw.to_degrees();
                    updated = true;
                }
                SensorData::LinearAcceleration(accel) => {
                    self.linear_accel = accel;
                    updated = true;
                }
                _ => {}
            }
        }

        Ok(updated)
    }

    pub fn get_heading(&self) -> f32 {
        self.heading
    }

    pub fn is_level(&self, tolerance: f32) -> bool {
        self.roll.abs() < tolerance && self.pitch.abs() < tolerance
    }

    pub fn get_tilt_magnitude(&self) -> f32 {
        (self.roll.powi(2) + self.pitch.powi(2)).sqrt()
    }
}

fn test_i2c_connection(address: u16) -> Result<u16, Box<dyn Error>> {
    let mut i2c = I2c::new()?;
    i2c.set_slave_address(address)?;

    // Try to read a byte to test connection
    let mut buf = [0u8; 1];
    i2c.read(&mut buf)?;

    Ok(address)
}
*/
