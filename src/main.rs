//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
use core::cell::RefCell;
use cortex_m::singleton;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use fugit::{MicrosDurationU32, RateExtU32};
use mcp4725::*;
use panic_probe as _;
use rp2040_hal::{
    clocks::init_clocks_and_plls,
    gpio::bank0::Gpio12,
    gpio::bank0::Gpio13,
    gpio::bank0::Gpio14,
    gpio::bank0::Gpio15,
    gpio::bank0::Gpio16,
    gpio::bank0::Gpio17,
    gpio::bank0::Gpio8,
    gpio::bank0::Gpio9,
    gpio::Pins,
    pac,
    pac::interrupt,
    timer::{Alarm, Alarm0, Alarm1, Alarm2, Alarm3, Timer},
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    watchdog::Watchdog,
    Clock, Sio, I2C,
};
use rp_pico::entry;
mod interrupts;
mod types;
use rotary_encoder_embedded::{standard::StandardMode, Direction, RotaryEncoder};
use types::{I2CType, RotaryEncoder1Type, RotaryEncoder2Type, UartType};

const ENCODER_POLL_FREQUENCY: MicrosDurationU32 = MicrosDurationU32::millis(2);

static mut UART1_INST: Mutex<RefCell<Option<UartType>>> = Mutex::new(RefCell::new(None));

static ALARM_DEFAULT_DURATION_US: u32 = 100000;

static mut ALARM_0_DURATION: Mutex<RefCell<Option<MicrosDurationU32>>> = Mutex::new(RefCell::new(
    Some(MicrosDurationU32::micros(ALARM_DEFAULT_DURATION_US)),
));
static mut ALARM_1_DURATION: Mutex<RefCell<Option<MicrosDurationU32>>> = Mutex::new(RefCell::new(
    Some(MicrosDurationU32::micros(ALARM_DEFAULT_DURATION_US)),
));
static mut ALARM_2_DURATION: Mutex<RefCell<Option<MicrosDurationU32>>> = Mutex::new(RefCell::new(
    Some(MicrosDurationU32::micros(ALARM_DEFAULT_DURATION_US)),
));
static mut ALARM_3_DURATION: Mutex<RefCell<Option<MicrosDurationU32>>> = Mutex::new(RefCell::new(
    Some(MicrosDurationU32::micros(ALARM_DEFAULT_DURATION_US)),
));
static mut ALARM_0: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static mut ALARM_1: Mutex<RefCell<Option<Alarm1>>> = Mutex::new(RefCell::new(None));
static mut ALARM_2: Mutex<RefCell<Option<Alarm2>>> = Mutex::new(RefCell::new(None));
static mut ALARM_3: Mutex<RefCell<Option<Alarm3>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    info!("Program start");
    let pac = pac::Peripherals::take().unwrap();
    let mut resets = pac.RESETS;
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    let external_xtal_freq_hz = 12_000_000u32;
    info!("Setting clocks");
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
    info!("creating timer");
    let mut timer = Timer::new(pac.TIMER, &mut resets, &clocks);

    info!("unwrapping alarms");
    let mut alarm_0 = timer.alarm_0().unwrap();
    let mut alarm_1 = timer.alarm_1().unwrap();
    let mut alarm_2 = timer.alarm_2().unwrap();
    let mut alarm_3 = timer.alarm_3().unwrap();

    // let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);

    // // I2C DAC
    // let dac_scl = pins.gpio17.into_function();
    // let dac_sda = pins.gpio16.into_function();
    // let dac_i2c = I2C::i2c0(
    //     pac.I2C0,
    //     dac_sda,
    //     dac_scl,
    //     400.kHz(),
    //     &mut resets,
    //     125_000_000.Hz(),
    // );
    // let dac = MCP4725::new(dac_i2c, 0b010);

    // // UART0
    // let uart_pins = (pins.gpio8.into_function(), pins.gpio9.into_function());
    // let mut uart = UartPeripheral::new(pac.UART1, uart_pins, &mut resets)
    //     .enable(
    //         UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
    //         clocks.peripheral_clock.freq(),
    //     )
    //     .unwrap();
    // uart.enable_rx_interrupt();

    // Rotary Encoders
    // let rotary_1_dt = pins.gpio15.into_pull_up_input();
    // let rotary_1_clk = pins.gpio14.into_pull_up_input();
    // let rotary_2_dt = pins.gpio13.into_pull_up_input();
    // let rotary_2_clk = pins.gpio12.into_pull_up_input();
    // let rotary_1 = RotaryEncoder::new(rotary_1_dt, rotary_1_clk).into_standard_mode();
    // let rotary_2 = RotaryEncoder::new(rotary_2_dt, rotary_2_clk).into_standard_mode();

    info!("going critical");
    critical_section::with(|cs| {
        let alarm_0_duration = unsafe { ALARM_0_DURATION.borrow(cs).take().unwrap() };
        info!("{}", alarm_0_duration.ticks());
        let alarm_1_duration = unsafe { ALARM_1_DURATION.borrow(cs).take().unwrap() };
        let alarm_2_duration = unsafe { ALARM_2_DURATION.borrow(cs).take().unwrap() };
        let alarm_3_duration = unsafe { ALARM_3_DURATION.borrow(cs).take().unwrap() };
        info!("durations done");
        alarm_0.schedule(alarm_0_duration).ok();
        alarm_0.enable_interrupt();
        alarm_1.schedule(alarm_1_duration).ok();
        alarm_1.enable_interrupt();
        alarm_2.schedule(alarm_2_duration).ok();
        alarm_2.enable_interrupt();
        alarm_3.schedule(alarm_3_duration).ok();
        alarm_3.enable_interrupt();
        info!("alarms scheduled");
        unsafe { ALARM_0.borrow(cs).replace(Some(alarm_0)) };
        unsafe { ALARM_1.borrow(cs).replace(Some(alarm_1)) };
        unsafe { ALARM_2.borrow(cs).replace(Some(alarm_2)) };
        unsafe { ALARM_3.borrow(cs).replace(Some(alarm_3)) };
        unsafe { ALARM_0_DURATION.borrow(cs).replace(Some(alarm_0_duration)) };
        unsafe { ALARM_1_DURATION.borrow(cs).replace(Some(alarm_1_duration)) };
        unsafe { ALARM_2_DURATION.borrow(cs).replace(Some(alarm_2_duration)) };
        unsafe { ALARM_3_DURATION.borrow(cs).replace(Some(alarm_3_duration)) };
        info!("alarm globals replaced");
        info!("unmasking interrupts");
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
            pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_1);
            pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_2);
            pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_3);
            // pac::NVIC::unmask(pac::Interrupt::UART1_IRQ);
        }
    });

    loop {}
}
