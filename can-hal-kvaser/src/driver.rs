use std::sync::Arc;

use can_hal::{ChannelBuilder, Driver};

use crate::channel::KvaserChannel;
use crate::error::{check_status, KvaserError};
use crate::event::ReceiveEvent;
use crate::ffi::CAN_OPEN_CAN_FD;
use crate::library::KvaserLibrary;

// Nominal bitrate timing: tseg1=13, tseg2=6, sjw=4, noSamp=1, syncMode=0.
// Gives 70% sample point; the hardware adjusts the prescaler for the target frequency.
const TSEG1: u32 = 13;
const TSEG2: u32 = 6;
const SJW: u32 = 4;

// FD data-phase timing: tseg1=7, tseg2=2, sjw=2.
// Gives 80% sample point (8 of 10 TQs); hardware adjusts prescaler.
const FD_TSEG1: u32 = 7;
const FD_TSEG2: u32 = 2;
const FD_SJW: u32 = 2;

/// Driver for KVASER CAN adapters using the CANlib API.
///
/// Loads `libcanlib.so.1` (Linux) or `canlib32.dll` (Windows) at construction time.
///
/// # Example
///
/// ```rust,no_run
/// use can_hal::{ChannelBuilder, Driver};
/// use can_hal_kvaser::KvaserDriver;
///
/// let driver = KvaserDriver::new().expect("CANlib not found");
/// let mut channel = driver.channel(0).unwrap().bitrate(500_000).unwrap().connect().unwrap();
/// ```
pub struct KvaserDriver {
    lib: Arc<KvaserLibrary>,
}

impl KvaserDriver {
    /// Load CANlib from the default system location.
    pub fn new() -> Result<Self, KvaserError> {
        Ok(KvaserDriver {
            lib: KvaserLibrary::load()?,
        })
    }

    /// Load CANlib from a custom path.
    pub fn with_library_path(path: &str) -> Result<Self, KvaserError> {
        Ok(KvaserDriver {
            lib: KvaserLibrary::load_from(path)?,
        })
    }
}

impl Driver for KvaserDriver {
    type Channel = KvaserChannel;
    type Builder = KvaserChannelBuilder;
    type Error = KvaserError;

    /// Begin configuring the channel at the given 0-based index.
    fn channel(&self, index: u32) -> Result<KvaserChannelBuilder, KvaserError> {
        Ok(KvaserChannelBuilder {
            lib: Arc::clone(&self.lib),
            channel_index: index as i32,
            bitrate_hz: None,
            fd_bitrate_hz: None,
        })
    }
}

/// Builder for configuring a KVASER channel before going on-bus.
pub struct KvaserChannelBuilder {
    pub(crate) lib: Arc<KvaserLibrary>,
    pub(crate) channel_index: i32,
    pub(crate) bitrate_hz: Option<u32>,
    pub(crate) fd_bitrate_hz: Option<u32>,
}

impl ChannelBuilder for KvaserChannelBuilder {
    type Channel = KvaserChannel;
    type Error = KvaserError;

    fn bitrate(mut self, hz: u32) -> Result<Self, KvaserError> {
        self.bitrate_hz = Some(hz);
        Ok(self)
    }

    fn data_bitrate(mut self, hz: u32) -> Result<Self, KvaserError> {
        self.fd_bitrate_hz = Some(hz);
        Ok(self)
    }

    fn sample_point(self, _sample_point: f32) -> Result<Self, KvaserError> {
        Err(KvaserError::NotSupported(
            "sample_point() is not supported; the sample point is fixed at 70% for nominal \
             and 80% for data phase"
                .into(),
        ))
    }

    fn connect(self) -> Result<KvaserChannel, KvaserError> {
        let bitrate_hz = self.bitrate_hz.ok_or_else(|| {
            KvaserError::NotSupported("bitrate() must be called before connect()".into())
        })?;

        let mut flags = 0i32;
        if self.fd_bitrate_hz.is_some() {
            flags |= CAN_OPEN_CAN_FD;
        }

        let handle = unsafe { (self.lib.open_channel)(self.channel_index, flags) };
        if handle < 0 {
            return Err(KvaserError::Canlib(crate::error::KvaserStatus(handle)));
        }

        // Close the handle on any subsequent failure to avoid a resource leak.
        let result = (|| {
            check_status(unsafe {
                (self.lib.set_bus_params)(
                    handle,
                    bitrate_hz as i32,
                    TSEG1,
                    TSEG2,
                    SJW,
                    1, // noSamp
                    0, // syncMode
                )
            })?;

            if let Some(fd_hz) = self.fd_bitrate_hz {
                check_status(unsafe {
                    (self.lib.set_bus_params_fd)(handle, fd_hz as i32, FD_TSEG1, FD_TSEG2, FD_SJW)
                })?;
            }

            check_status(unsafe { (self.lib.bus_on)(handle) })?;

            let event = ReceiveEvent::new(&self.lib, handle)?;

            Ok(KvaserChannel {
                lib: self.lib.clone(),
                handle,
                fd_mode: self.fd_bitrate_hz.is_some(),
                event,
            })
        })();

        if result.is_err() {
            unsafe { (self.lib.close)(handle) };
        }

        result
    }
}
