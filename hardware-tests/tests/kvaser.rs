use can_hal::frame::CanFrame;
use can_hal::{CanId, ChannelBuilder, Driver, Transmit};
use can_hal_kvaser::KvaserDriver;

fn open_kvaser_classic() -> impl Transmit<Error = can_hal_kvaser::KvaserError> {
    let driver = KvaserDriver::new().expect("Kvaser CANlib not found");
    driver
        .channel(0)
        .unwrap()
        .bitrate(500_000)
        .unwrap()
        .connect()
        .expect("Failed to open Kvaser channel")
}

#[test]
fn test_kvaser_channel_open() {
    let _channel = open_kvaser_classic();
}

#[test]
fn test_kvaser_transmit() {
    let mut channel = open_kvaser_classic();
    let frame = CanFrame::new(CanId::new_standard(0x200).unwrap(), &[0xCA, 0xFE])
        .expect("Failed to create frame");
    channel.transmit(&frame).expect("Kvaser transmit failed");
}

#[test]
fn test_kvaser_fd_init() {
    let driver = KvaserDriver::new().expect("Kvaser CANlib not found");
    let _channel = driver
        .channel(0)
        .unwrap()
        .bitrate(500_000)
        .unwrap()
        .data_bitrate(2_000_000)
        .unwrap()
        .connect()
        .expect("Failed to open Kvaser FD channel");
}
