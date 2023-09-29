//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::cell::RefCell;
use core::f32;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use fugit::{MicrosDurationU32, RateExtU32};
use mcp4725::*;
use panic_probe as _;
use rp2040_hal::{
    clocks::init_clocks_and_plls, gpio::bank0::Gpio16, gpio::bank0::Gpio17, gpio::Function,
    gpio::Pin, gpio::Pins, i2c::Controller, pac, pac::interrupt, timer::Alarm, timer::Timer,
    watchdog::Watchdog, Clock, Sio, I2C,
};
use rp_pico::entry;
mod types;
use types::I2CType;

type DACAlarm = rp2040_hal::timer::Alarm0;
static mut DAC_ALARM: Mutex<RefCell<Option<DACAlarm>>> = Mutex::new(RefCell::new(None));

type DACType = MCP4725<I2CType>;
static mut DAC: Mutex<RefCell<Option<DACType>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    info!("Program start");
    let pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut resets = pac.RESETS;
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut resets,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);
    let mut timer = Timer::new(pac.TIMER, &mut resets, &clocks);
    let dac_scl = pins.gpio17.into_function();
    let dac_sda = pins.gpio16.into_function();
    let dac_i2c = I2C::i2c0(
        pac.I2C0,
        dac_sda,
        dac_scl,
        400.kHz(),
        &mut resets,
        125_000_000.Hz(),
    );
    let dac = MCP4725::new(dac_i2c, 0b010);

    critical_section::with(|cs| {
        unsafe { DAC.borrow(cs).replace(Some(dac)) };
    });

    loop {}
}

// Handle DAC_ALARM
#[interrupt]
fn TIMER_IRQ_0() {
    critical_section::with(|cs| {
        let dac = unsafe { DAC.borrow(cs).take().unwrap() };
        let mut dac_alarm = unsafe { DAC_ALARM.borrow(cs).take().unwrap() };
        dac_alarm.clear_interrupt();

        unsafe { DAC_ALARM.borrow(cs).replace(Some(dac_alarm)) };
        unsafe { DAC.borrow(cs).replace(Some(dac)) };
    });
}
