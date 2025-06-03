#![no_std] // Assuming this is for an embedded target

use embassy_stm32::dma::NoDma; // Default DMA type
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Error as UartError, UartRx};
use embassy_time::{Duration, TimeoutError, Timer};

use core::str;
use defmt::*; // For logging, optional but helpful

// Max NMEA sentence length is typically around 82 characters + CR/LF
const NMEA_BUFFER_SIZE: usize = 100;
const NMEA_READ_TIMEOUT: Duration = Duration::from_millis(1500); // Increased timeout for reading a full line

#[derive(Debug, defmt::Format)]
pub enum NmeaError {
    Uart(UartError),
    Timeout,
    InvalidSentenceFormat,
    ChecksumMismatch,
    ParseError(&'static str), // Field that failed to parse
    Utf8Error,
    BufferOverflow,
}

impl From<UartError> for NmeaError {
    fn from(e: UartError) -> Self {
        NmeaError::Uart(e)
    }
}

impl From<TimeoutError> for NmeaError {
    fn from(_: TimeoutError) -> Self {
        NmeaError::Timeout
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FixQuality {
    Invalid = 0,
    GpsFix = 1,
    DgpsFix = 2,
    PpsFix = 3,
    Rtk = 4,
    FloatRtk = 5,
    Estimated = 6,
    Manual = 7,
    Simulation = 8,
    Unknown,
}

impl Default for FixQuality {
    fn default() -> Self {
        FixQuality::Invalid
    }
}

impl From<u8> for FixQuality {
    fn from(val: u8) -> Self {
        match val {
            0 => FixQuality::Invalid,
            1 => FixQuality::GpsFix,
            2 => FixQuality::DgpsFix,
            3 => FixQuality::PpsFix,
            4 => FixQuality::Rtk,
            5 => FixQuality::FloatRtk,
            6 => FixQuality::Estimated,
            7 => FixQuality::Manual,
            8 => FixQuality::Simulation,
            _ => {
                warn!("NMEA: Unknown fix quality value: {}", val);
                FixQuality::Unknown
            }
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct GgaData {
    pub timestamp_utc: Option<f32>, // HHMMSS.sss
    pub latitude: Option<f32>,      // Decimal degrees
    pub longitude: Option<f32>,     // Decimal degrees
    pub fix_quality: FixQuality,
    pub num_satellites: u8,
    pub hdop: Option<f32>,             // Horizontal Dilution of Precision
    pub altitude_msl: Option<f32>,     // Meters above mean sea level
    pub geoid_separation: Option<f32>, // Meters
}

pub struct NeoGps<'a> {
    uart_rx: &'a mut UartRx<'static, Async>,
    buffer: [u8; NMEA_BUFFER_SIZE],
    buffer_len: usize,
}

impl<'a> NeoGps<'a> {
    pub fn new(uart_rx: &'a mut UartRx<'static, Async>) -> Self {
        Self {
            uart_rx,
            buffer: [0u8; NMEA_BUFFER_SIZE],
            buffer_len: 0,
        }
    }

    /// Reads a single NMEA sentence from the UART.
    /// Returns the sentence string *including* the type, payload, and checksum part, but without '$' or '\r\n'.
    async fn read_sentence_raw(&mut self) -> Result<&str, NmeaError> {
        self.buffer_len = 0;
        let mut found_dollar = false;

        loop {
            let mut byte_buf = [0u8; 1];
            embassy_time::with_timeout(NMEA_READ_TIMEOUT, self.uart_rx.read(&mut byte_buf))
                .await??;
            let byte = byte_buf[0];

            if byte == b'$' {
                found_dollar = true;
                self.buffer_len = 0; // Reset buffer on new sentence start
                continue; // Skip the '$' itself
            }

            if found_dollar {
                if self.buffer_len >= NMEA_BUFFER_SIZE {
                    warn!("NMEA buffer overflow while reading sentence.");
                    found_dollar = false; // Reset to look for next $
                    self.buffer_len = 0;
                    return Err(NmeaError::BufferOverflow);
                }

                if byte == b'\r' {
                    // Next byte should be '\n'
                    embassy_time::with_timeout(NMEA_READ_TIMEOUT, self.uart_rx.read(&mut byte_buf))
                        .await??;
                    if byte_buf[0] == b'\n' {
                        if self.buffer_len > 0 {
                            // Successfully read a sentence up to \r\n
                            let sentence_slice = &self.buffer[..self.buffer_len];
                            return str::from_utf8(sentence_slice)
                                .map_err(|_| NmeaError::Utf8Error);
                        } else {
                            // Empty sentence (e.g., "$ \r\n"), try again
                            found_dollar = false;
                        }
                    } else {
                        warn!(
                            "NMEA sentence framing error: Expected LF after CR, got {:#02X}",
                            byte_buf[0]
                        );
                        found_dollar = false; // Reset state
                        self.buffer_len = 0;
                        return Err(NmeaError::InvalidSentenceFormat);
                    }
                } else if byte.is_ascii_graphic()
                    || byte == b','
                    || byte == b'.'
                    || byte == b'*'
                    || byte == b'-'
                {
                    // Accept printable ASCII, comma, period, star (for checksum), minus
                    self.buffer[self.buffer_len] = byte;
                    self.buffer_len += 1;
                } else if byte == b'\n' {
                    // If we get a \n without a preceding \r, treat as end of line (some GPS might do this)
                    // Or if \r was missed.
                    warn!("NMEA: Encountered LF without CR. Treating as end of sentence.");
                    if self.buffer_len > 0 {
                        let sentence_slice = &self.buffer[..self.buffer_len];
                        return str::from_utf8(sentence_slice).map_err(|_| NmeaError::Utf8Error);
                    } else {
                        found_dollar = false;
                    }
                } else {
                    warn!("NMEA: Invalid character in sentence: {:#02X}", byte);
                    // Optionally, could discard current sentence attempt here
                    // found_dollar = false;
                    // self.buffer_len = 0;
                    // return Err(NmeaError::InvalidSentenceFormat);
                }
            }
            // If not found_dollar, we are just consuming bytes until a $ is found
        }
    }

    fn validate_and_parse_checksum(sentence_with_checksum: &str) -> Result<(&str, u8), NmeaError> {
        if let Some(star_pos) = sentence_with_checksum.rfind('*') {
            // Ensure star_pos is not the first character and there's space for checksum
            if star_pos == 0 || star_pos + 3 > sentence_with_checksum.len() {
                warn!("NMEA: Malformed checksum part: {}", sentence_with_checksum);
                return Err(NmeaError::InvalidSentenceFormat);
            }

            let payload = &sentence_with_checksum[..star_pos]; // This is "GPGGA,data,..."
            let checksum_str = &sentence_with_checksum[star_pos + 1..];

            if checksum_str.len() != 2 {
                warn!(
                    "NMEA: Invalid checksum string length: '{}' in '{}'",
                    checksum_str, sentence_with_checksum
                );
                return Err(NmeaError::InvalidSentenceFormat);
            }

            let received_checksum = u8::from_str_radix(checksum_str, 16).map_err(|_| {
                warn!(
                    "NMEA: Failed to parse checksum hex: '{}' in '{}'",
                    checksum_str, sentence_with_checksum
                );
                NmeaError::InvalidSentenceFormat
            })?;

            let mut calculated_checksum = 0u8;
            for byte_char in payload.bytes() {
                // Iterate over bytes of "GPGGA,data,..."
                calculated_checksum ^= byte_char;
            }

            if calculated_checksum == received_checksum {
                Ok((payload, calculated_checksum))
            } else {
                warn!(
                    "NMEA Checksum mismatch! Full: '{}', Payload: '{}', Calc: {:#02X}, Recv: {:#02X}",
                    sentence_with_checksum, payload, calculated_checksum, received_checksum
                );
                Err(NmeaError::ChecksumMismatch)
            }
        } else {
            // This case should ideally not be hit often if read_sentence_raw captures till *
            warn!(
                "NMEA: No checksum '*' found in sentence: {}",
                sentence_with_checksum
            );
            Err(NmeaError::InvalidSentenceFormat)
        }
    }

    fn parse_lat_lon(val_str: &str, hem_str: &str) -> Option<f32> {
        if val_str.is_empty() || hem_str.is_empty() {
            return None;
        }

        // Example: val_str = "4043.8400" (lat) or "07359.6835" (lon)
        let dot_pos = val_str.find('.')?;
        if dot_pos < 2 {
            return None;
        } // Need at least MM.MMMM

        let degrees_part_len = dot_pos - 2; // DD or DDD
        let deg_str = &val_str[..degrees_part_len];
        let min_str = &val_str[degrees_part_len..];

        let deg: f32 = deg_str.parse().ok()?;
        let min: f32 = min_str.parse().ok()?;

        let mut decimal_degrees = deg + (min / 60.0);

        match hem_str {
            "S" | "W" => decimal_degrees *= -1.0,
            "N" | "E" => {} // Positive
            _ => {
                warn!("NMEA: Invalid hemisphere: {}", hem_str);
                return None;
            } // Invalid hemisphere
        }
        Some(decimal_degrees)
    }

    fn parse_gpgga(payload_with_type: &str) -> Result<GgaData, NmeaError> {
        // payload_with_type is "GPGGA,timestamp,lat,N/S,lon,E/W,fix_quality,num_sats,hdop,altitude,M,geoid_sep,M,..."
        let mut fields = payload_with_type.split(',');

        let _sentence_type = fields
            .next()
            .ok_or(NmeaError::ParseError("Sentence Type"))?; // Consume "GPGGA"

        let timestamp_str = fields.next().ok_or(NmeaError::ParseError("Timestamp"))?;
        let lat_str = fields.next().ok_or(NmeaError::ParseError("LatitudeVal"))?;
        let lat_hem_str = fields.next().ok_or(NmeaError::ParseError("LatitudeHem"))?;
        let lon_str = fields.next().ok_or(NmeaError::ParseError("LongitudeVal"))?;
        let lon_hem_str = fields.next().ok_or(NmeaError::ParseError("LongitudeHem"))?;
        let fix_quality_str = fields.next().ok_or(NmeaError::ParseError("FixQuality"))?;
        let num_satellites_str = fields
            .next()
            .ok_or(NmeaError::ParseError("NumSatellites"))?;
        let hdop_str = fields.next().ok_or(NmeaError::ParseError("HDOP"))?;
        let altitude_msl_str = fields.next().ok_or(NmeaError::ParseError("Altitude"))?;
        let _alt_unit_str = fields.next().ok_or(NmeaError::ParseError("AltitudeUnit"))?; // 'M'
        let geoid_sep_str = fields
            .next()
            .ok_or(NmeaError::ParseError("GeoidSeparation"))?;
        // let _geoid_unit_str = fields.next().ok_or(NmeaError::ParseError("GeoidUnit"))?; // 'M'
        // Skip age_dgps and dgps_ref_id for simplicity

        let gga_data = GgaData {
            timestamp_utc: timestamp_str.parse().ok(),
            latitude: Self::parse_lat_lon(lat_str, lat_hem_str),
            longitude: Self::parse_lat_lon(lon_str, lon_hem_str),
            fix_quality: fix_quality_str
                .parse::<u8>()
                .ok()
                .map(FixQuality::from)
                .unwrap_or(FixQuality::Invalid),
            num_satellites: num_satellites_str.parse().unwrap_or_default(),
            hdop: hdop_str.parse().ok(),
            altitude_msl: altitude_msl_str.parse().ok(),
            geoid_separation: geoid_sep_str.parse().ok(),
        };
        trace!("Parsed GPGGA: {:?}", Debug2Format(&gga_data));
        Ok(gga_data)
    }

    /// Reads and attempts to parse the next GPGGA sentence.
    pub async fn read_gga_data(&mut self) -> Result<GgaData, NmeaError> {
        loop {
            let sentence_with_checksum = self.read_sentence_raw().await?;
            trace!(
                "NMEA Raw sentence with checksum: {}",
                sentence_with_checksum
            );

            match Self::validate_and_parse_checksum(sentence_with_checksum) {
                Ok((payload_with_type, _checksum)) => {
                    // payload_with_type is "GPGGA,data..."
                    if payload_with_type.starts_with("GPGGA")
                        || payload_with_type.starts_with("GNGGA")
                    {
                        match Self::parse_gpgga(payload_with_type) {
                            Ok(data) => return Ok(data),
                            Err(parse_err) => {
                                warn!(
                                    "NMEA: Failed to parse GPGGA payload '{}': {:?}",
                                    payload_with_type, parse_err
                                );
                                // Continue to read next sentence
                            }
                        }
                    } else {
                        let type_str = payload_with_type.split(',').next().unwrap_or("");
                        trace!("NMEA: Skipping non-GPGGA/GNGGA sentence type: {}", type_str);
                        // Continue loop to read next sentence
                    }
                }
                Err(NmeaError::ChecksumMismatch) => {
                    // Warning already logged in validate_and_parse_checksum
                    // Continue loop to read next sentence
                }
                Err(e) => {
                    // More critical format error or UTF8 error from read_sentence_raw
                    return Err(e);
                }
            }
        }
    }
}
