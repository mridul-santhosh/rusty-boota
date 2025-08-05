#![no_std]
#![no_main]

use rp235x_hal as hal;
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::FullDuplex;
use embedded_hal::digital::OutputPin;
use core::fmt::Write; // For core::write! if you want to log over UART etc.
use hal::fugit::RateExtU32;
use panic_halt as _;

// CYW43439 register addresses (from datasheet)
const GPIO_DIR_REG: u32 = 0x10006a;
const GPIO_OUT_REG: u32 = 0x10006c;

// Helper: format a gSPI write command for a 1-byte register write (per datasheet 4.2.1, Figure 12)
// F1:F0=0b01 (function 1), C=1 (write), A=0 (fixed), Addr=17bit, Length=1
fn gspi_write_cmd(addr: u32, value: u8) -> [u8; 5] {
    // Function = 0b01, Command = 1, Addr Mode = 0 (fixed), Address = 17bits, Length = 1
    let cmd: u32 = (0b01 << 30) | (1 << 29) | (0 << 28)
        | ((addr & 0x1FFFF) << 11)
        | 1; // length = 1
    [
        ((cmd >> 24) & 0xFF) as u8,
        ((cmd >> 16) & 0xFF) as u8,
        ((cmd >> 8 ) & 0xFF) as u8,
        ((cmd      ) & 0xFF) as u8,
        value,
    ]
}

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    // Setup clocks
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // Setup pins per Pico 2 W wireless interface wiring
    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // SPI pins for wireless, Pico 2 W: CS= GPIO25, D(DIN/DOUT/IRQ)=GPIO24, CLK=GPIO29
    let mut cs_pin = pins.gpio25.into_push_pull_output();
    let spi_d = pins.gpio24.into_function::<hal::gpio::FunctionSpi>();
    let spi_clk = pins.gpio29.into_function::<hal::gpio::FunctionSpi>();

    // For simplicity, use both MOSI and MISO as D (same GPIO) — the HAL's SPI expects a tuple.
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_d, spi_d, spi_clk));

    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let mut delay = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let mut led_pin = pins.gpio25.into_push_pull_output(); // Reuse as user indicator

    // Minimal: delay to let CYW43439 power up
    delay.delay_ms(50);

    // 1. Set CYW43439 GPIO0 as output (write 0x01 to DIR register)
    let dir_cmd = gspi_write_cmd(GPIO_DIR_REG, 0x01);
    cs_pin.set_low().unwrap();
    for b in dir_cmd.iter() {
        let _ = nb::block!(spi.send(*b));
        let _ = nb::block!(spi.read());
    }
    cs_pin.set_high().unwrap();
    delay.delay_ms(1); // Short delay between ops

    // 2. Set CYW43439 GPIO0 'high' (write 0x01 to OUT register)
    let out_cmd = gspi_write_cmd(GPIO_OUT_REG, 0x01);
    cs_pin.set_low().unwrap();
    for b in out_cmd.iter() {
        let _ = nb::block!(spi.send(*b));
        let _ = nb::block!(spi.read());
    }
    cs_pin.set_high().unwrap();

    // (Optional: Readback or check status -- omitted for clarity)

    // Indicate success: slow blink
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Set CYW43439 GPIO0 via gSPI"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
