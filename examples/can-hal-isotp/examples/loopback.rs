// ISO-TP loopback test: send and receive on the same SocketCAN interface.
//
// Opens two SocketCAN channels on the same interface. A sender thread transmits
// ISO-TP messages of increasing size and a receiver thread reassembles and
// prints them. This works because SocketCAN delivers frames to all sockets
// bound to the same interface.
//
// Software requirements:
//   - Linux only (SocketCAN is a Linux kernel subsystem)
//   - A SocketCAN interface (virtual or physical)
//
// Virtual interface setup:
//   sudo modprobe vcan
//   sudo ip link add dev vcan0 type vcan
//   sudo ip link set vcan0 up
//
// Usage:
//   cargo run --example loopback -p can-hal-isotp-examples -- [interface]
//
// Default interface: vcan0

use std::env;
use std::thread;
use std::time::Duration;

use can_hal::CanId;
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};
use can_hal_socketcan::SocketCanChannel;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ifname = env::args().nth(1).unwrap_or_else(|| "vcan0".into());

    println!("ISO-TP loopback test on '{ifname}'");
    println!("----------------------------------");

    // Test payloads of increasing size.
    let payloads: Vec<Vec<u8>> = vec![
        // 7 bytes: fits in a Single Frame
        vec![0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
        // 20 bytes: requires First Frame + Consecutive Frames
        (0..20).map(|i| (i * 3) as u8).collect(),
        // 100 bytes
        (0..100).map(|i| i as u8).collect(),
        // 4000 bytes
        (0..4000u16).map(|i| (i & 0xFF) as u8).collect(),
    ];

    for (i, payload) in payloads.iter().enumerate() {
        println!(
            "\n[Test {}/{}] Sending {} byte(s)...",
            i + 1,
            payloads.len(),
            payload.len()
        );

        let ifname_tx = ifname.clone();
        let ifname_rx = ifname.clone();
        let tx_data = payload.clone();

        // Spawn receiver thread first so it's ready before sender starts.
        let rx_handle = thread::spawn(move || -> Result<Vec<u8>, String> {
            let channel =
                SocketCanChannel::open(&ifname_rx).map_err(|e| format!("RX open error: {e}"))?;

            // Receiver: tx_id=0x7E8 (for FC), rx_id=0x7E0 (data from sender)
            let config = IsoTpConfig {
                timeout: Duration::from_secs(5),
                ..IsoTpConfig::new(
                    CanId::new_standard(0x7E8).expect("valid CAN ID"),
                    CanId::new_standard(0x7E0).expect("valid CAN ID"),
                )
            };

            let mut isotp = IsoTpChannel::new(channel, config);
            isotp.receive().map_err(|e| format!("RX error: {e}"))
        });

        // Small delay to let the receiver thread bind its socket.
        thread::sleep(Duration::from_millis(50));

        // Spawn sender thread.
        let tx_handle = thread::spawn(move || -> Result<(), String> {
            let channel =
                SocketCanChannel::open(&ifname_tx).map_err(|e| format!("TX open error: {e}"))?;

            // Sender: tx_id=0x7E0, rx_id=0x7E8 (for FC from receiver)
            let config = IsoTpConfig {
                timeout: Duration::from_secs(5),
                ..IsoTpConfig::new(
                    CanId::new_standard(0x7E0).expect("valid CAN ID"),
                    CanId::new_standard(0x7E8).expect("valid CAN ID"),
                )
            };

            let mut isotp = IsoTpChannel::new(channel, config);
            isotp.send(&tx_data).map_err(|e| format!("TX error: {e}"))
        });

        // Wait for both threads.
        let tx_result = tx_handle.join().expect("TX thread panicked");
        let rx_result = rx_handle.join().expect("RX thread panicked");

        if let Err(e) = &tx_result {
            eprintln!("  TX failed: {e}");
        }

        match rx_result {
            Ok(received) => {
                let matches = received == *payload;
                println!(
                    "  RX: {} byte(s) received, data {}",
                    received.len(),
                    if matches { "MATCHES" } else { "MISMATCH!" },
                );
                if !matches {
                    eprintln!(
                        "  Expected first 8: {:02X?}",
                        &payload[..payload.len().min(8)]
                    );
                    eprintln!(
                        "  Got first 8:      {:02X?}",
                        &received[..received.len().min(8)]
                    );
                }
            }
            Err(e) => {
                eprintln!("  RX failed: {e}");
            }
        }
    }

    println!("\nLoopback test complete.");
    Ok(())
}
