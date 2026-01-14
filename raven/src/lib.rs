use serialport::{self, SerialPort};
use std::io::{Read, Write};
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorChannel {
    CH1 = 0,
    CH2 = 1,
    CH3 = 2,
    CH4 = 3,
    CH5 = 4,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorMode {
    Disable = 0x00,
    Direct = 0x01,
    Position = 0x02,
    Velocity = 0x03,
}

impl MotorMode {
    fn to_bytes(self) -> u8 {
        self as u8
    }

    fn from_bytes(value: u8) -> Option<Self> {
        match value {
            0x00 => Some(MotorMode::Disable),
            0x01 => Some(MotorMode::Direct),
            0x02 => Some(MotorMode::Position),
            0x03 => Some(MotorMode::Velocity),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ServoChannel {
    CH1 = 0,
    CH2 = 1,
    CH3 = 2,
    CH4 = 3,
}

#[derive(Debug, Clone, Copy)]
enum MessageType {
    ServoValue = 0 << 3,
    MotorMode = 1 << 3,
    MotorPid = 2 << 3,
    MotorTarget = 3 << 3,
    MotorVoltage = 4 << 3,
    MotorCurrent = 5 << 3,
    EncoderValue = 6 << 3,
    MotorVelocityValue = 7 << 3,
    MotorVoltageValue = 8 << 3,
    MotorCurrentValue = 9 << 3,
    Reset = 10 << 3,
}

#[derive(Debug, Clone, Copy)]
enum ReadWrite {
    Write = 0x00,
    Read = 0x80,
}

struct CRC8Encoder {
    table: [u8; 256],
}

impl CRC8Encoder {
    fn new(poly: u8) -> Self {
        let mut table = [0u8; 256];
        for i in 0..256 {
            let mut remainder = i as u8;
            for _ in 0..8 {
                if remainder & 0x80 != 0 {
                    remainder = (remainder << 1) ^ poly;
                } else {
                    remainder = remainder << 1;
                }
            }
            table[i] = remainder;
        }
        CRC8Encoder { table }
    }

    fn crc(&self, data: &[u8], start: u8) -> u8 {
        let mut remainder = start;
        for &byte in data {
            remainder = self.table[(byte ^ remainder) as usize];
        }
        remainder
    }
}

struct RavenSerial {
    port: Box<dyn SerialPort>,
    crc: CRC8Encoder,
}

impl RavenSerial {
    const START: u8 = 0xAA;
    const SMBUS_POLY: u8 = 0x07;

    fn new(
        port_name: &str,
        baud_rate: u32,
        timeout: Duration,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(timeout)
            .open()?;

        Ok(RavenSerial {
            port,
            crc: CRC8Encoder::new(Self::SMBUS_POLY),
        })
    }

    fn read(&mut self) -> Option<(u8, Vec<u8>)> {
        let mut crc = 0u8;
        let mut buffer = [0u8; 1];

        // Read start byte
        if self.port.read_exact(&mut buffer).is_err() {
            return None;
        }
        if buffer[0] != Self::START {
            return None;
        }
        crc = self.crc.crc(&buffer, crc);

        // Read header
        if self.port.read_exact(&mut buffer).is_err() {
            return None;
        }
        crc = self.crc.crc(&buffer, crc);
        let length = ((buffer[0] >> 2) & 0b11111) as usize;

        // Read data
        let mut data = vec![0u8; length];
        if self.port.read_exact(&mut data).is_err() {
            return None;
        }
        crc = self.crc.crc(&data, crc);

        // Read CRC
        if self.port.read_exact(&mut buffer).is_err() {
            return None;
        }
        crc = self.crc.crc(&buffer, crc);

        if crc == 0 && !data.is_empty() {
            let header = data[0];
            let message = data[1..].to_vec();
            Some((header, message))
        } else {
            None
        }
    }

    fn make_message(&self, ack: bool, data: &[u8]) -> (Vec<u8>, u8) {
        let mut header = (data.len() as u8) << 2;
        if ack {
            header |= 0x80;
        }

        let mut message = vec![Self::START, header];
        message.extend_from_slice(data);

        let crc = self.crc.crc(&message, 0);
        message.push(crc);

        (message, crc)
    }

    fn send(&mut self, ack: bool, data: &[u8], retry: u32) -> Option<Vec<u8>> {
        let _ = self.port.clear(serialport::ClearBuffer::Input);

        let (message, expected_crc) = self.make_message(ack, data);

        if self.port.write_all(&message).is_err() {
            if retry > 0 {
                return self.send(ack, data, retry - 1);
            }
            return None;
        }

        if let Some((received_crc, received_data)) = self.read() {
            if received_crc == expected_crc {
                return Some(received_data);
            }
        }

        if retry > 0 {
            self.send(ack, data, retry - 1)
        } else {
            None
        }
    }
}

pub struct Raven {
    serial: RavenSerial,
}

impl Raven {
    const MOTOR_MAX_VALUE: i32 = 4095;
    const MOTOR_MAX_VALUE_PERCENT: f32 = 4095.0 / 100.0;
    const MOTOR_MAX_PERCENT_VALUE: f32 = 1.0 / (4095.0 / 100.0);
    const MOTOR_MAX_CURRENT: f32 = 6.5;
    const PID_FREQ: f32 = 1000.0;
    const PID_DT: f32 = 1.0 / 1000.0;
    const SERVO_COUNT_PER_US: f32 = 168.0 / 51.0;

    pub fn new(port: Option<&str>, timeout: Duration) -> Result<Self, Box<dyn std::error::Error>> {
        let port_name = if let Some(p) = port {
            p.to_string()
        } else {
            let ports = serialport::available_ports()?;
            if ports.is_empty() {
                return Err("No serial ports found".into());
            }
            ports[0].port_name.clone()
        };

        let mut raven = Raven {
            serial: RavenSerial::new(&port_name, 460800, timeout)?,
        };

        raven.reset(0);
        Ok(raven)
    }

    fn make_message(
        message_type: MessageType,
        rw: ReadWrite,
        channel: Option<u8>,
        data: &[u8],
    ) -> Vec<u8> {
        let channel_value = channel.unwrap_or(0);
        let mut message = vec![(message_type as u8) + (rw as u8) + channel_value];
        message.extend_from_slice(data);
        message
    }

    fn read_value(
        &mut self,
        message_type: MessageType,
        channel: Option<u8>,
        retry: u32,
    ) -> Option<Vec<u8>> {
        let message = Self::make_message(message_type, ReadWrite::Read, channel, &[]);
        self.serial.send(true, &message, retry)
    }

    fn write_value(
        &mut self,
        message_type: MessageType,
        channel: Option<u8>,
        data: &[u8],
        retry: u32,
    ) -> bool {
        let message = Self::make_message(message_type, ReadWrite::Write, channel, data);
        self.serial.send(true, &message, retry).is_some()
    }

    pub fn get_motor_mode(&mut self, motor_channel: MotorChannel, retry: u32) -> Option<MotorMode> {
        let value = self.read_value(MessageType::MotorMode, Some(motor_channel as u8), retry)?;
        if value.len() == 1 {
            MotorMode::from_bytes(value[0])
        } else {
            None
        }
    }

    pub fn set_motor_mode(
        &mut self,
        motor_channel: MotorChannel,
        motor_mode: MotorMode,
        retry: u32,
    ) -> bool {
        self.write_value(
            MessageType::MotorMode,
            Some(motor_channel as u8),
            &[motor_mode.to_bytes()],
            retry,
        )
    }

    pub fn get_motor_target(&mut self, motor_channel: MotorChannel, retry: u32) -> Option<f32> {
        let value = self.read_value(MessageType::MotorTarget, Some(motor_channel as u8), retry)?;
        if value.len() == 4 {
            Some(f32::from_le_bytes([value[0], value[1], value[2], value[3]]))
        } else {
            None
        }
    }

    pub fn set_motor_target(
        &mut self,
        motor_channel: MotorChannel,
        value: f32,
        retry: u32,
    ) -> bool {
        let bytes = value.to_le_bytes();
        self.write_value(
            MessageType::MotorTarget,
            Some(motor_channel as u8),
            &bytes,
            retry,
        )
    }

    pub fn set_motor_speed_factor(
        &mut self,
        motor_channel: MotorChannel,
        percent: f32,
        reverse: bool,
        retry: u32,
    ) -> bool {
        assert!(percent >= 0.0 && percent <= 100.0);
        let mut value = (percent * Self::MOTOR_MAX_VALUE_PERCENT) as i16;
        if reverse {
            value = -value;
        }
        let bytes = value.to_le_bytes();
        self.write_value(
            MessageType::MotorVoltage,
            Some(motor_channel as u8),
            &bytes,
            retry,
        )
    }

    pub fn set_motor_torque_factor(
        &mut self,
        motor_channel: MotorChannel,
        percent: f32,
        retry: u32,
    ) -> bool {
        assert!(percent >= 0.0 && percent <= 100.0);
        let value = (percent * Self::MOTOR_MAX_VALUE_PERCENT) as u16;
        let bytes = value.to_le_bytes();
        self.write_value(
            MessageType::MotorCurrent,
            Some(motor_channel as u8),
            &bytes,
            retry,
        )
    }

    pub fn set_motor_max_current(
        &mut self,
        motor_channel: MotorChannel,
        current: f32,
        retry: u32,
    ) -> bool {
        assert!(current >= 0.0 && current < Self::MOTOR_MAX_CURRENT);
        self.set_motor_torque_factor(
            motor_channel,
            current / Self::MOTOR_MAX_CURRENT * 100.0,
            retry,
        )
    }

    pub fn get_motor_pid(
        &mut self,
        motor_channel: MotorChannel,
        retry: u32,
    ) -> Option<(f32, f32, f32, f32)> {
        let value = self.read_value(MessageType::MotorPid, Some(motor_channel as u8), retry)?;
        if value.len() == 16 {
            let p = f32::from_le_bytes([value[0], value[1], value[2], value[3]]);
            let i = f32::from_le_bytes([value[4], value[5], value[6], value[7]]);
            let d = f32::from_le_bytes([value[8], value[9], value[10], value[11]]);
            let sat = f32::from_le_bytes([value[12], value[13], value[14], value[15]]);
            Some((
                p,
                i * Self::PID_FREQ,
                d * Self::PID_DT,
                sat * Self::MOTOR_MAX_PERCENT_VALUE,
            ))
        } else {
            None
        }
    }

    pub fn set_motor_pid(
        &mut self,
        motor_channel: MotorChannel,
        p_gain: f32,
        i_gain: f32,
        d_gain: f32,
        percent: f32,
        retry: u32,
    ) -> bool {
        assert!(percent >= 0.0 && percent <= 100.0);
        let sat = percent * Self::MOTOR_MAX_VALUE_PERCENT;

        let mut data = Vec::new();
        data.extend_from_slice(&p_gain.to_le_bytes());
        data.extend_from_slice(&(i_gain * Self::PID_DT).to_le_bytes());
        data.extend_from_slice(&(d_gain * Self::PID_FREQ).to_le_bytes());
        data.extend_from_slice(&sat.to_le_bytes());

        self.write_value(
            MessageType::MotorPid,
            Some(motor_channel as u8),
            &data,
            retry,
        )
    }

    fn deg_to_count(deg: f32, min_us: u32, max_us: u32) -> u16 {
        let us_range = (max_us - min_us) as f32;
        let us = ((deg + 90.0) / 180.0 * us_range) + min_us as f32;
        (us * Self::SERVO_COUNT_PER_US) as u16
    }

    fn count_to_deg(count: u16, min_us: u32, max_us: u32) -> f32 {
        let us_range = (max_us - min_us) as f32;
        let us = count as f32 / Self::SERVO_COUNT_PER_US;
        (us - min_us as f32) / us_range * 180.0 - 90.0
    }

    pub fn get_servo_position(
        &mut self,
        servo_channel: ServoChannel,
        min_us: u32,
        max_us: u32,
        retry: u32,
    ) -> Option<f32> {
        let value = self.read_value(MessageType::ServoValue, Some(servo_channel as u8), retry)?;
        if value.len() == 2 {
            let count = u16::from_le_bytes([value[0], value[1]]);
            Some(Self::count_to_deg(count, min_us, max_us))
        } else {
            None
        }
    }

    pub fn set_servo_position(
        &mut self,
        servo_channel: ServoChannel,
        degree: f32,
        min_us: u32,
        max_us: u32,
        retry: u32,
    ) -> Result<bool, String> {
        if degree < -90.0 || degree > 90.0 {
            return Err("Invalid degree".to_string());
        }
        let count = Self::deg_to_count(degree, min_us, max_us);
        let bytes = count.to_le_bytes();
        Ok(self.write_value(
            MessageType::ServoValue,
            Some(servo_channel as u8),
            &bytes,
            retry,
        ))
    }

    pub fn set_servo_off(&mut self, servo_channel: ServoChannel, retry: u32) -> bool {
        let bytes = 0u16.to_le_bytes();
        self.write_value(
            MessageType::ServoValue,
            Some(servo_channel as u8),
            &bytes,
            retry,
        )
    }

    pub fn get_motor_encoder(&mut self, motor_channel: MotorChannel, retry: u32) -> Option<i32> {
        let value = self.read_value(MessageType::EncoderValue, Some(motor_channel as u8), retry)?;
        if value.len() == 4 {
            Some(i32::from_le_bytes([value[0], value[1], value[2], value[3]]))
        } else {
            None
        }
    }

    pub fn set_motor_encoder(
        &mut self,
        motor_channel: MotorChannel,
        value: i32,
        retry: u32,
    ) -> bool {
        let bytes = value.to_le_bytes();
        self.write_value(
            MessageType::EncoderValue,
            Some(motor_channel as u8),
            &bytes,
            retry,
        )
    }

    pub fn get_motor_velocity(&mut self, motor_channel: MotorChannel, retry: u32) -> Option<f32> {
        let value = self.read_value(
            MessageType::MotorVelocityValue,
            Some(motor_channel as u8),
            retry,
        )?;
        if value.len() == 4 {
            Some(f32::from_le_bytes([value[0], value[1], value[2], value[3]]))
        } else {
            None
        }
    }

    pub fn reset(&mut self, retry: u32) -> bool {
        self.write_value(MessageType::Reset, None, &[], retry)
    }
}

impl Drop for Raven {
    fn drop(&mut self) {
        self.reset(0);
    }
}
