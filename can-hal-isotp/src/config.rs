use std::time::Duration;

use can_hal::CanId;

/// ISO-TP addressing mode.
#[derive(Debug, Clone, Copy)]
pub enum AddressingMode {
    /// Normal addressing: only PCI bytes in the payload.
    Normal,
    /// Extended addressing: a target address byte precedes the PCI bytes.
    ///
    /// Per ISO 15765-2, the target address (N_TA) in transmitted frames identifies
    /// the remote ECU, while the target address in received frames identifies the
    /// local ECU. These are typically different values:
    ///
    /// - `tx_target_address`: written into byte 0 of every transmitted CAN frame
    /// - `rx_target_address`: expected in byte 0 of every received CAN frame
    Extended {
        tx_target_address: u8,
        rx_target_address: u8,
    },
}

/// Configuration for an ISO-TP channel.
#[derive(Debug, Clone)]
pub struct IsoTpConfig {
    /// CAN ID used for transmitted frames.
    pub tx_id: CanId,
    /// CAN ID expected for received frames.
    pub rx_id: CanId,
    /// Addressing mode (Normal or Extended).
    pub addressing: AddressingMode,
    /// Block Size advertised in Flow Control frames (0 = no limit).
    pub block_size: u8,
    /// Raw STmin byte advertised in our Flow Control frames (controls sender's inter-frame gap).
    ///
    /// Encoding (ISO 15765-2):
    /// - `0x00`--`0x7F`: 0--127 milliseconds
    /// - `0xF1`--`0xF9`: 100--900 microseconds
    /// - All other values: reserved, treated as 0
    ///
    /// **Platform note**: Values in the 0xF1--0xF9 range (sub-millisecond) are rounded
    /// up to 1 ms because `std::thread::sleep` on Linux has ~1 ms granularity without
    /// real-time scheduling (`SCHED_RR`). For strict sub-millisecond timing, the caller
    /// must configure the thread with `SCHED_RR` before calling `send`.
    pub st_min: u8,
    /// Timeout for N_Bs (waiting for FC) and N_Cr (waiting for CF).
    pub timeout: Duration,
    /// If `Some(byte)`, pad all transmitted CAN frames to 8 bytes with this value.
    /// `None` means no padding (only the required bytes are sent).
    pub padding: Option<u8>,
    /// Maximum number of consecutive FC(Wait) frames to accept before returning
    /// `IsoTpError::WaitLimitExceeded`. For example, a value of `3` permits up to
    /// 2 Wait frames and aborts on the 3rd. `0` means unlimited (rely solely on
    /// the overall `timeout`).
    pub max_fc_wait: u8,
    /// CAN ID used for functionally addressed (broadcast) requests.
    /// Functional requests MUST fit in a single frame — no multi-frame allowed.
    /// Typically 0x7DF (11-bit) for UDS. `None` means functional addressing is not used.
    pub functional_id: Option<CanId>,
}

impl IsoTpConfig {
    /// Create a new `IsoTpConfig` with sensible defaults.
    ///
    /// Defaults: Normal addressing, block_size=0, st_min=0, timeout=1s,
    /// padding=None, max_fc_wait=10.
    pub fn new(tx_id: CanId, rx_id: CanId) -> Self {
        IsoTpConfig {
            tx_id,
            rx_id,
            addressing: AddressingMode::Normal,
            block_size: 0,
            st_min: 0,
            timeout: Duration::from_millis(1000),
            padding: None,
            max_fc_wait: 10,
            functional_id: None,
        }
    }

    /// Interpret the raw `st_min` byte as a `Duration`.
    ///
    /// - 0x00..=0x7F: value in milliseconds
    /// - 0xF1..=0xF9: 100..900 microseconds (rounded up to 1ms in practice)
    /// - All other values (0x80..=0xF0, 0xFA..=0xFF): treated as 0
    pub fn st_min_duration(&self) -> Duration {
        match self.st_min {
            0x00..=0x7F => Duration::from_millis(self.st_min as u64),
            0xF1..=0xF9 => {
                let us = (self.st_min - 0xF0) as u64 * 100;
                // Round up to 1ms minimum since std::thread::sleep granularity is ~1ms
                Duration::from_micros(us).max(Duration::from_millis(1))
            }
            _ => Duration::ZERO,
        }
    }

    /// Returns the number of overhead bytes for the current addressing mode.
    /// Normal = 0 extra bytes, Extended = 1 byte (target address).
    pub fn overhead(&self) -> usize {
        match self.addressing {
            AddressingMode::Normal => 0,
            AddressingMode::Extended { .. } => 1,
        }
    }

    /// Returns the TX target address for Extended addressing, if configured.
    pub fn tx_target_address(&self) -> Option<u8> {
        match self.addressing {
            AddressingMode::Normal => None,
            AddressingMode::Extended {
                tx_target_address, ..
            } => Some(tx_target_address),
        }
    }

    /// Returns the RX target address for Extended addressing, if configured.
    pub fn rx_target_address(&self) -> Option<u8> {
        match self.addressing {
            AddressingMode::Normal => None,
            AddressingMode::Extended {
                rx_target_address, ..
            } => Some(rx_target_address),
        }
    }
}
