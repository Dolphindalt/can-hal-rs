// Extended addressing ISO-TP example: sender and receiver on vcan0.
//
// Demonstrates ISO-TP extended addressing, where a target address byte (TA)
// is prepended to every CAN frame's payload before the PCI bytes.
//
// Wire format for a Single Frame with extended addressing (TA = 0xF1):
//   [0xF1] [SF_DL] [data...]
//   byte 0: target address
//   byte 1: PCI (0x0 | SF_DL)
//   bytes 2..: payload
//
// This reduces the per-frame payload capacity by one byte:
//   - Single Frame: max 6 data bytes (vs 7 for normal)
//   - Consecutive Frame: max 6 data bytes (vs 7 for normal)
//
// Software requirements:
//   - Linux only (SocketCAN)
//   - A virtual CAN interface (vcan0)
//
// Virtual interface setup:
//   sudo modprobe vcan
//   sudo ip link add dev vcan0 type vcan
//   sudo ip link set vcan0 up
//
// Usage:
//   cargo run --example extended_addressing -p can-hal-isotp-examples -- [interface]
//
// Default interface: vcan0

use std::env;
use std::thread;
use std::time::Duration;

use can_hal::CanId;
use can_hal_isotp::{AddressingMode, IsoTpChannel, IsoTpConfig};
use can_hal_socketcan::SocketCanChannel;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ifname = env::args().nth(1).unwrap_or_else(|| "vcan0".into());

    println!("Extended addressing ISO-TP example on '{ifname}'");
    println!("------------------------------------------------");

    // 20-byte test payload (requires multi-frame: FF + CFs).
    let payload: Vec<u8> = (0..20).collect();
    let tx_data = payload.clone();

    println!(
        "Sending {} bytes with extended addressing (TA=0xF1)...",
        payload.len()
    );

    let ifname_rx = ifname.clone();
    let ifname_tx = ifname.clone();

    // Spawn receiver thread first so it's ready before sender starts.
    let rx_handle = thread::spawn(move || -> Result<Vec<u8>, String> {
        let channel =
            SocketCanChannel::open(&ifname_rx).map_err(|e| format!("RX open error: {e}"))?;

        // Receiver config: tx_id=0x7E8 (for FC frames), rx_id=0x7E0 (data from sender)
        // Extended addressing with TA=0xF1: receiver filters on this TA byte.
        let config = IsoTpConfig {
            addressing: AddressingMode::Extended {
                target_address: 0xF1,
            },
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

        // Sender config: tx_id=0x7E0, rx_id=0x7E8 (for FC from receiver)
        // Extended addressing with TA=0xF1: every transmitted frame gets 0xF1 as byte 0.
        let config = IsoTpConfig {
            addressing: AddressingMode::Extended {
                target_address: 0xF1,
            },
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
        eprintln!("TX failed: {e}");
    }

    match rx_result {
        Ok(received) => {
            let matches = received == payload;
            println!(
                "RX: {} byte(s) received, data {}",
                received.len(),
                if matches { "MATCHES" } else { "MISMATCH!" },
            );
            if !matches {
                eprintln!("  Expected: {:02X?}", &payload[..payload.len().min(8)]);
                eprintln!("  Got:      {:02X?}", &received[..received.len().min(8)]);
            }
        }
        Err(e) => {
            eprintln!("RX failed: {e}");
        }
    }

    // Wire format explanation:
    // With extended addressing (TA=0xF1), the first CAN frame (First Frame) looks like:
    //   CAN ID 0x7E0: [F1 10 14 00 01 02 03 04]
    //                   ^^ TA byte
    //                      ^^ ^^ FF PCI: type=1, total_len=0x014 (20)
    //                            ^^ ^^ ^^ ^^ ^^ first 5 data bytes
    //
    // Consecutive Frames:
    //   CAN ID 0x7E0: [F1 21 05 06 07 08 09 0A]
    //                   ^^ TA byte
    //                      ^^ CF PCI: type=2, SN=1
    //                         ^^ ^^ ^^ ^^ ^^ ^^ 6 data bytes

    println!("\nExtended addressing example complete.");
    Ok(())
}
