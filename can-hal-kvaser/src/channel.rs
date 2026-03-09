use std::os::raw::{c_long, c_ulong};
use std::sync::Arc;
use std::time::{Duration, Instant};

use can_hal::{
    BusState, BusStatus, CanFdFrame, CanFrame, ErrorCounters, Filter, Filterable, Frame, Receive,
    ReceiveFd, Timestamped, Transmit, TransmitFd,
};

/// Maximum poll interval for the event fd.
///
/// The mhydra (linuxcan) driver uses edge-triggered event semantics: the event fd
/// becomes readable when new frames arrive, but may not re-signal if frames arrive
/// between `read_frame()` returning `None` and `poll()` starting. By capping the
/// poll timeout we guarantee periodic queue drains regardless of event fd state.
const MAX_POLL_INTERVAL: Duration = Duration::from_millis(50);

use crate::convert::{from_canlib_frame, to_canlib_id};
use crate::error::{check_status, KvaserError};
use crate::event::ReceiveEvent;
use crate::ffi::{
    CAN_ERR_NOMSG, CAN_FILTER_SET_CODE_EXT, CAN_FILTER_SET_CODE_STD, CAN_FILTER_SET_MASK_EXT,
    CAN_FILTER_SET_MASK_STD, CAN_MSG_BRS, CAN_MSG_ESI, CAN_MSG_FDF, CAN_STAT_BUS_OFF,
    CAN_STAT_ERROR_PASSIVE, CAN_STAT_ERROR_WARNING,
};
use crate::library::KvaserLibrary;

/// An open, on-bus KVASER CAN channel.
pub struct KvaserChannel {
    pub(crate) lib: Arc<KvaserLibrary>,
    pub(crate) handle: i32,
    pub(crate) fd_mode: bool,
    pub(crate) event: ReceiveEvent,
}

impl Drop for KvaserChannel {
    fn drop(&mut self) {
        unsafe {
            (self.lib.bus_off)(self.handle);
            (self.lib.close)(self.handle);
        }
    }
}

impl KvaserChannel {
    /// Non-blocking read. Returns `Ok(None)` if the queue is empty.
    fn read_frame(&mut self) -> Result<Option<Frame>, KvaserError> {
        let mut raw_id: c_long = 0;
        // 64 bytes covers both classic CAN (≤8 bytes used) and CAN FD (≤64 bytes).
        let mut data = [0u8; 64];
        let mut dlc: u32 = 0;
        let mut flags: u32 = 0;
        let mut timestamp: c_ulong = 0;

        let status = unsafe {
            (self.lib.read)(
                self.handle,
                &mut raw_id,
                data.as_mut_ptr().cast(),
                &mut dlc,
                &mut flags,
                &mut timestamp,
            )
        };

        if status == CAN_ERR_NOMSG {
            return Ok(None);
        }
        check_status(status)?;
        from_canlib_frame(raw_id as u32, &data, dlc, flags)
    }
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------

impl Transmit for KvaserChannel {
    type Error = KvaserError;

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), KvaserError> {
        let (id, flags) = to_canlib_id(frame.id());
        check_status(unsafe {
            (self.lib.write)(
                self.handle,
                id as c_long,
                frame.data().as_ptr().cast(),
                frame.len() as u32,
                flags,
            )
        })?;
        // Wait up to 100 ms for the frame to leave the transmit buffer.
        check_status(unsafe { (self.lib.write_sync)(self.handle, 100) })
    }
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------

impl Receive for KvaserChannel {
    type Error = KvaserError;

    fn receive(&mut self) -> Result<Timestamped<CanFrame>, KvaserError> {
        loop {
            match self.read_frame()? {
                Some(Frame::Can(f)) => return Ok(Timestamped::new(f, Instant::now())),
                Some(Frame::Fd(_)) => {} // FD frame on classic receive — skip and retry
                None => {
                    // Bounded wait: see MAX_POLL_INTERVAL doc for rationale.
                    let _ = self.event.wait(Some(MAX_POLL_INTERVAL))?;
                }
            }
        }
    }

    fn try_receive(&mut self) -> Result<Option<Timestamped<CanFrame>>, KvaserError> {
        loop {
            match self.read_frame()? {
                Some(Frame::Can(f)) => return Ok(Some(Timestamped::new(f, Instant::now()))),
                Some(Frame::Fd(_)) => continue, // skip FD frames
                None => return Ok(None),
            }
        }
    }

    fn receive_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<CanFrame>>, KvaserError> {
        let deadline = Instant::now() + timeout;
        loop {
            // Drain the queue before waiting on the event. If we hit an FD frame,
            // keep reading — there may be a classic frame queued behind it. Only
            // block on the event when the queue is truly empty.
            loop {
                match self.read_frame()? {
                    Some(Frame::Can(f)) => return Ok(Some(Timestamped::new(f, Instant::now()))),
                    Some(Frame::Fd(_)) => continue, // skip FD frames, drain queue
                    None => break,                  // queue empty, need to wait
                }
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Ok(None);
            }
            // Bounded wait: see MAX_POLL_INTERVAL doc for rationale.
            let poll_timeout = remaining.min(MAX_POLL_INTERVAL);
            let _ = self.event.wait(Some(poll_timeout))?;
        }
    }
}

// ---------------------------------------------------------------------------
// TransmitFd
// ---------------------------------------------------------------------------

impl TransmitFd for KvaserChannel {
    type Error = KvaserError;

    fn transmit_fd(&mut self, frame: &CanFdFrame) -> Result<(), KvaserError> {
        if !self.fd_mode {
            return Err(KvaserError::NotSupported(
                "channel was not opened in FD mode; call data_bitrate() before connect()".into(),
            ));
        }
        let (id, mut flags) = to_canlib_id(frame.id());
        flags |= CAN_MSG_FDF;
        if frame.brs() {
            flags |= CAN_MSG_BRS;
        }
        if frame.esi() {
            flags |= CAN_MSG_ESI;
        }
        check_status(unsafe {
            (self.lib.write)(
                self.handle,
                id as c_long,
                frame.data().as_ptr().cast(),
                frame.len() as u32,
                flags,
            )
        })?;
        check_status(unsafe { (self.lib.write_sync)(self.handle, 100) })
    }
}

// ---------------------------------------------------------------------------
// ReceiveFd
// ---------------------------------------------------------------------------

impl ReceiveFd for KvaserChannel {
    type Error = KvaserError;

    fn receive_fd(&mut self) -> Result<Timestamped<Frame>, KvaserError> {
        if !self.fd_mode {
            return Err(KvaserError::NotSupported(
                "channel was not opened in FD mode; call data_bitrate() before connect()".into(),
            ));
        }
        loop {
            match self.read_frame()? {
                Some(frame) => return Ok(Timestamped::new(frame, Instant::now())),
                None => {
                    // Bounded wait: see MAX_POLL_INTERVAL doc for rationale.
                    let _ = self.event.wait(Some(MAX_POLL_INTERVAL))?;
                }
            }
        }
    }

    fn try_receive_fd(&mut self) -> Result<Option<Timestamped<Frame>>, KvaserError> {
        if !self.fd_mode {
            return Err(KvaserError::NotSupported(
                "channel was not opened in FD mode; call data_bitrate() before connect()".into(),
            ));
        }
        match self.read_frame()? {
            Some(frame) => Ok(Some(Timestamped::new(frame, Instant::now()))),
            None => Ok(None),
        }
    }

    fn receive_fd_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<Frame>>, KvaserError> {
        if !self.fd_mode {
            return Err(KvaserError::NotSupported(
                "channel was not opened in FD mode; call data_bitrate() before connect()".into(),
            ));
        }
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(frame) = self.read_frame()? {
                return Ok(Some(Timestamped::new(frame, Instant::now())));
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Ok(None);
            }
            // Bounded wait: see MAX_POLL_INTERVAL doc for rationale.
            let poll_timeout = remaining.min(MAX_POLL_INTERVAL);
            let _ = self.event.wait(Some(poll_timeout))?;
        }
    }
}

// ---------------------------------------------------------------------------
// Filterable
// ---------------------------------------------------------------------------

impl Filterable for KvaserChannel {
    type Error = KvaserError;

    /// Apply acceptance filters.
    ///
    /// CANlib supports only a single (code, mask) pair per frame type (standard /
    /// extended). When multiple filters of the same type are provided, they are
    /// merged into one pair: masks are OR-ed (more bits checked) and codes are
    /// AND-ed (only common bits survive).
    ///
    /// **Important**: this merge computes the *intersection* of the filter
    /// conditions, not the union. If you need to accept IDs 0x100 **and** 0x200,
    /// a single code/mask pair cannot represent that. In such cases, use a
    /// permissive mask (e.g., `mask = 0x000` to accept all) and do
    /// software-level filtering, or provide a single filter whose mask already
    /// covers the desired range.
    fn set_filters(&mut self, filters: &[Filter]) -> Result<(), KvaserError> {
        // Process standard and extended filters in a single pass each, without
        // heap allocation.
        apply_merged_filter(
            &self.lib,
            self.handle,
            filters,
            |f| f.id.is_standard(),
            CAN_FILTER_SET_CODE_STD,
            CAN_FILTER_SET_MASK_STD,
        )?;
        apply_merged_filter(
            &self.lib,
            self.handle,
            filters,
            |f| f.id.is_extended(),
            CAN_FILTER_SET_CODE_EXT,
            CAN_FILTER_SET_MASK_EXT,
        )?;
        Ok(())
    }

    /// Remove all acceptance filters (accept all frames).
    fn clear_filters(&mut self) -> Result<(), KvaserError> {
        // mask = 0 means no bits are checked → every frame passes.
        for &flag in &[
            CAN_FILTER_SET_CODE_STD,
            CAN_FILTER_SET_MASK_STD,
            CAN_FILTER_SET_CODE_EXT,
            CAN_FILTER_SET_MASK_EXT,
        ] {
            check_status(unsafe { (self.lib.accept)(self.handle, 0, flag) })?;
        }
        Ok(())
    }
}

/// Merge all filters matching `predicate` into a single (code, mask) pair and
/// apply it via `canAccept`. No heap allocation.
fn apply_merged_filter(
    lib: &KvaserLibrary,
    handle: i32,
    filters: &[Filter],
    predicate: impl Fn(&Filter) -> bool,
    code_flag: u32,
    mask_flag: u32,
) -> Result<(), KvaserError> {
    let mut merged: Option<(u32, u32)> = None; // (code, mask)

    for f in filters.iter().filter(|f| predicate(f)) {
        let code = f.id.raw() & f.mask;
        merged = Some(match merged {
            None => (code, f.mask),
            Some((c, m)) => (c & code, m | f.mask),
        });
    }

    if let Some((code, mask)) = merged {
        check_status(unsafe { (lib.accept)(handle, code as c_long, code_flag) })?;
        check_status(unsafe { (lib.accept)(handle, mask as c_long, mask_flag) })?;
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// BusStatus
// ---------------------------------------------------------------------------

impl BusStatus for KvaserChannel {
    type Error = KvaserError;

    fn bus_state(&self) -> Result<BusState, KvaserError> {
        let mut flags: c_ulong = 0;
        check_status(unsafe { (self.lib.read_status)(self.handle, &mut flags) })?;

        if flags & CAN_STAT_BUS_OFF != 0 {
            Ok(BusState::BusOff)
        } else if flags & (CAN_STAT_ERROR_PASSIVE | CAN_STAT_ERROR_WARNING) != 0 {
            Ok(BusState::ErrorPassive)
        } else {
            Ok(BusState::ErrorActive)
        }
    }

    fn error_counters(&self) -> Result<ErrorCounters, KvaserError> {
        let mut tx_err: u32 = 0;
        let mut rx_err: u32 = 0;
        // `overrun` counts frames lost due to receive buffer overflow.
        // It is read to satisfy the CANlib API but is not exposed by `ErrorCounters`.
        let mut overrun: u32 = 0;
        check_status(unsafe {
            (self.lib.read_error_counters)(self.handle, &mut tx_err, &mut rx_err, &mut overrun)
        })?;
        Ok(ErrorCounters {
            transmit: tx_err.min(255) as u8,
            receive: rx_err.min(255) as u8,
        })
    }
}
