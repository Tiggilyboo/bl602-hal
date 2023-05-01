// Don't include standard rust library, since it's incompatible with embedded systems
#![no_std]

// We use our own entrypoint marked with #[riscv_rt::entry], not rust's default.
#![no_main]

// import Hardware Abstraction Layer for bl602
use bl602_hal as hal;

use core::fmt::Write;
use embedded_hal::digital::blocking::{OutputPin, ToggleableOutputPin};
use embedded_hal::{delay::blocking::DelayMs, adc::nb::Channel};
use embedded_hal::adc::nb::OneShot;

use hal::{
	clock::{Strict, SysclkFreq, UART_PLL_FREQ},
	pac,
	prelude::*,
	serial::*,
    adc::*,
};

// We want our embedded system to halt indefinitely when we panic.
// We discard the import since its functionality is a side-effect
use panic_halt as _;

// define a baudrate for serial communication
const BAUD_RATE:u32 = 1000000;

// RiscV entrypoint. The ! notation indicates we have no panic handler
#[riscv_rt::entry]
fn main() -> ! {

    // Load all peripherals
    let dp = pac::Peripherals::take().unwrap();
    // split the register block into individual pins and modules
    let mut parts = dp.GLB.split();

    // Set up uart output. Bl602 uses a pin matrix so we set muxes too.
    let pin16 = parts.pin16.into_uart_sig0();
    let pin7 = parts.pin7.into_uart_sig7();
    let mux0 = parts.uart_mux0.into_uart0_tx();
    let mux7 = parts.uart_mux7.into_uart0_rx();
    
    let mut red_led = parts.pin17.into_pull_down_output();
    let mut _blue_led = parts.pin11.into_pull_down_output();
    let mut green_led = parts.pin14.into_pull_down_output();

    let mut adc_pin = parts.pin15.into_adc();
    
    // If UART does not work, keep red high so we know where it fails...
    red_led.set_high().unwrap();
    
    let clocks = Strict::new()
        .use_pll(40_000_000u32.Hz())
        .sys_clk(SysclkFreq::Pll160Mhz)
        .uart_clk(UART_PLL_FREQ.Hz())
        .adc_clk(1_000u32.Hz())
        .freeze(&mut parts.clk_cfg);
	
    let mut delay = hal::delay::McycleDelay::new(clocks.sysclk().0);

    // Configure our UART to our baudrate, and use the pins we configured above
    let mut serial = Serial::new(dp.UART0, Config::default().baudrate(BAUD_RATE.Bd()), ((pin16, mux0), (pin7, mux7)), clocks);
    
    serial.write_str("Serial initialized\n").ok();
    red_led.set_low().unwrap();
    
    // Pins for ADC for Pinecone dev board
    let adc_channel = adc_pin.channel();
    let mut adc = Adc1::new(adc_channel);
    serial.write_str("Created adc config\n").ok();
    adc.init(&clocks);
    
    serial.write_fmt(format_args!("Initialized adc {adc_channel:?}\n")).ok();

    loop {
        delay.delay_ms(1000).unwrap();
        green_led.toggle().unwrap();

        // Read the ADC channel
        let adc_value: Result<u16, _> = adc.read(&mut adc_pin);

        // Write the ADC value to UART
        match adc_value {
            Ok(adc_value) => {
                serial.write_fmt(format_args!("{adc_channel:?} = {adc_value:?}\n")).unwrap();
            },
            Err(err) => {
                serial.write_fmt(format_args!("{adc_channel:?} = {err:?}\n")).unwrap();
            }
        }
    }
}

