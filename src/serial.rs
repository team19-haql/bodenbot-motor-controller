//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip as well as how to create multiple usb classes for one device
//!
//! This creates a USB serial port that echos. It will also print out logging information on a separate serial device

use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
pub async fn serial_task(usb: USB) {
    defmt::info!("Starting USB serial logger");
    // Create the driver, from the HAL.
    let driver = Driver::new(usb, Irqs);

    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}
