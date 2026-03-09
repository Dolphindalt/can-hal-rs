use std::fmt;

/// Errors that can occur during ISO-TP communication.
///
/// The `Send + Sync + 'static` bounds on `E` match the [`CanError`](can_hal::error::CanError)
/// trait bound, ensuring `IsoTpError` is safe to use across thread boundaries
/// and with error-handling crates like `anyhow`.
#[derive(Debug)]
pub enum IsoTpError<E: Send + Sync + 'static> {
    /// Error from the underlying CAN channel.
    CanError(E),
    /// A timeout expired (N_Bs or N_Cr).
    Timeout,
    /// The receiver sent a Flow Control frame with the Overflow flag.
    BufferOverflow,
    /// A received frame could not be parsed as a valid ISO-TP PDU.
    InvalidFrame,
    /// Consecutive Frame sequence number mismatch.
    SequenceError { expected: u8, got: u8 },
    /// Payload exceeds the maximum allowed size (4095 for classic, 0xFFFF_FFFF for long FF).
    PayloadTooLarge,
    /// Too many consecutive FC(Wait) frames received (exceeds `max_fc_wait`).
    WaitLimitExceeded,
}

impl<E: fmt::Display + Send + Sync + 'static> fmt::Display for IsoTpError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IsoTpError::CanError(e) => write!(f, "CAN error: {}", e),
            IsoTpError::Timeout => write!(f, "ISO-TP timeout"),
            IsoTpError::BufferOverflow => write!(f, "ISO-TP receiver buffer overflow"),
            IsoTpError::InvalidFrame => write!(f, "invalid ISO-TP frame"),
            IsoTpError::SequenceError { expected, got } => {
                write!(
                    f,
                    "ISO-TP sequence error: expected {}, got {}",
                    expected, got
                )
            }
            IsoTpError::PayloadTooLarge => write!(f, "ISO-TP payload too large"),
            IsoTpError::WaitLimitExceeded => write!(f, "ISO-TP FC Wait limit exceeded"),
        }
    }
}

impl<E: std::error::Error + Send + Sync + 'static> std::error::Error for IsoTpError<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            IsoTpError::CanError(e) => Some(e),
            _ => None,
        }
    }
}
