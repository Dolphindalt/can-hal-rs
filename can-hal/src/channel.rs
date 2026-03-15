use core::time::Duration;

use crate::error::CanError;
use crate::frame::{CanFdFrame, CanFrame, Frame, Timestamped};

/// Transmit classic CAN frames.
pub trait Transmit {
    type Error: CanError;

    /// Send a classic CAN frame.
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), Self::Error>;
}

/// Receive classic CAN frames.
pub trait Receive {
    type Error: CanError;
    /// The timestamp type used by this backend (e.g., `std::time::Instant`).
    type Timestamp: Clone;

    /// Blocks until a classic CAN frame is available.
    fn receive(&mut self) -> Result<Timestamped<CanFrame, Self::Timestamp>, Self::Error>;

    /// Returns immediately with `Ok(None)` if no frame is available.
    fn try_receive(
        &mut self,
    ) -> Result<Option<Timestamped<CanFrame, Self::Timestamp>>, Self::Error>;

    /// Blocks until a frame is available or the timeout expires.
    /// Returns `Ok(None)` on timeout.
    fn receive_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<CanFrame, Self::Timestamp>>, Self::Error>;
}

/// Transmit CAN FD frames.
pub trait TransmitFd {
    type Error: CanError;

    /// Send a CAN FD frame.
    fn transmit_fd(&mut self, frame: &CanFdFrame) -> Result<(), Self::Error>;
}

/// Receive any frame (classic or FD) from an FD-capable bus.
pub trait ReceiveFd {
    type Error: CanError;
    /// The timestamp type used by this backend (e.g., `std::time::Instant`).
    type Timestamp: Clone;

    /// Blocks until any frame is available; returns `Frame` enum.
    fn receive_fd(&mut self) -> Result<Timestamped<Frame, Self::Timestamp>, Self::Error>;

    /// Non-blocking variant.
    fn try_receive_fd(
        &mut self,
    ) -> Result<Option<Timestamped<Frame, Self::Timestamp>>, Self::Error>;

    /// Blocks until a frame is available or the timeout expires.
    /// Returns `Ok(None)` on timeout.
    fn receive_fd_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<Frame, Self::Timestamp>>, Self::Error>;
}
