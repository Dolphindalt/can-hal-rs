use can_hal::frame::CanFrame;
use can_hal::{BusState, BusStatus, CanId, ChannelBuilder, Driver, Transmit};
use can_hal_pcan::PcanDriver;

fn open_pcan_classic(
) -> impl Transmit<Error = can_hal_pcan::PcanError> + BusStatus<Error = can_hal_pcan::PcanError> {
    let driver = PcanDriver::new().expect("PCAN-Basic library not found");
    driver
        .channel(0)
        .unwrap()
        .bitrate(500_000)
        .unwrap()
        .connect()
        .expect("Failed to open PCAN channel")
}

#[test]
fn test_pcan_bus_status() {
    let channel = open_pcan_classic();
    let state = channel.bus_state().expect("Failed to read bus state");
    assert_eq!(
        state,
        BusState::ErrorActive,
        "Expected ErrorActive bus state"
    );

    let counters = channel
        .error_counters()
        .expect("Failed to read error counters");
    assert_eq!(counters.transmit, 0, "Expected 0 TX errors");
    assert_eq!(counters.receive, 0, "Expected 0 RX errors");
}

#[test]
fn test_pcan_transmit() {
    let mut channel = open_pcan_classic();
    let frame = CanFrame::new(
        CanId::new_standard(0x100).unwrap(),
        &[0xDE, 0xAD, 0xBE, 0xEF],
    )
    .expect("Failed to create frame");
    channel.transmit(&frame).expect("PCAN transmit failed");
}

#[test]
fn test_pcan_fd_init() {
    let driver = PcanDriver::new().expect("PCAN-Basic library not found");
    let _channel = driver
        .channel(0)
        .unwrap()
        .bitrate(500_000)
        .unwrap()
        .data_bitrate(2_000_000)
        .unwrap()
        .connect()
        .expect("Failed to open PCAN FD channel");
}
