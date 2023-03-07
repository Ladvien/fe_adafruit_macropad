#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code
// does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;

const USB_HOST_POLL_MS: u8 = 10;
const KEY_I: u8 = 0x0c;
const KEY_ESC: u8 = 0x29;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    // pac,
    sio::Sio,
    watchdog::Watchdog,
};

// Some traits we need
use embedded_hal::blocking::i2c::Write;
use fugit::RateExtU32;

// // Alias for our HAL crate
use rp2040_hal as hal;

use hal::pac;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(bsp::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB HID Class Device driver, providing Keyboard Reports
    let mut usb_hid = HIDClass::new(&usb_bus, KeyboardReport::desc(), USB_HOST_POLL_MS);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27da))
        .manufacturer("Thomas Brittain")
        .product("Adafruit Macropad")
        .serial_number("0")
        .device_class(0)
        .build();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Set the pins to their default state
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio13.into_push_pull_output();

    let mut switch_pin = pins.gpio0.into_pull_up_input();
    let mut switch_state = switch_pin.is_low();

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio20.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio21.into_mode::<hal::gpio::FunctionI2C>();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Write three bytes to the I²C device with 7-bit address 0x2C
    i2c.write(0x2c, &[1, 2, 3]).unwrap();

    loop {
        usb_dev.poll(&mut [&mut usb_hid]);

        let previous_switch_state = switch_state;
        switch_state = switch_pin.is_low();

        match (previous_switch_state, switch_state) {
            (Ok(true), Ok(false)) => {
                info!("normal mode!");
                led_pin.set_low().unwrap();
                send_key_press(&usb_hid, &mut delay, KEY_ESC);
            }
            (Ok(false), Ok(true)) => {
                info!("insert mode!");
                led_pin.set_high().unwrap();
                send_key_press(&usb_hid, &mut delay, KEY_I);
            }
            _ => {}
        }
    }
}

fn send_key_press(
    usb_hid: &HIDClass<bsp::hal::usb::UsbBus>,
    delay: &mut cortex_m::delay::Delay,
    key_code: u8,
) {
    let mut keyboard_report = KeyboardReport {
        modifier: 0,
        reserved: 0,
        leds: 0,
        keycodes: [0; 6],
    };
    keyboard_report.keycodes[0] = key_code;
    usb_hid.push_input(&keyboard_report).unwrap();
    delay.delay_ms(USB_HOST_POLL_MS.into());

    keyboard_report.keycodes[0] = 0;
    usb_hid.push_input(&keyboard_report).unwrap();
    delay.delay_ms(USB_HOST_POLL_MS.into());
}

// End of file
