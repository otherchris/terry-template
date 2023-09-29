use rp2040_hal::timer::Alarm;

use crate::{
    info, interrupt, singleton, ALARM_0, ALARM_0_DURATION, ALARM_1, ALARM_1_DURATION, ALARM_2,
    ALARM_2_DURATION, ALARM_3, ALARM_3_DURATION, UART1_INST,
};

#[interrupt]
fn TIMER_IRQ_0() {
    info!("alarm 0 fired and caught");
    critical_section::with(|cs| {
        let alarm_0_duration = unsafe { ALARM_0_DURATION.borrow(cs).take().unwrap() };
        let mut alarm_0 = unsafe { ALARM_0.borrow(cs).take().unwrap() };
        alarm_0.clear_interrupt();
        alarm_0.schedule(alarm_0_duration);
        unsafe { ALARM_0.borrow(cs).replace(Some(alarm_0)) };
        unsafe { ALARM_0_DURATION.borrow(cs).replace(Some(alarm_0_duration)) };
    });
}

#[interrupt]
fn TIMER_IRQ_1() {
    critical_section::with(|cs| {
        let alarm_1_duration = unsafe { ALARM_1_DURATION.borrow(cs).take().unwrap() };
        let mut alarm_1 = unsafe { ALARM_1.borrow(cs).take().unwrap() };
        alarm_1.clear_interrupt();
        alarm_1.schedule(alarm_1_duration).ok();
        unsafe { ALARM_1.borrow(cs).replace(Some(alarm_1)) };
        unsafe { ALARM_1_DURATION.borrow(cs).replace(Some(alarm_1_duration)) };
    });
}

#[interrupt]
fn TIMER_IRQ_2() {
    critical_section::with(|cs| {
        let alarm_2_duration = unsafe { ALARM_2_DURATION.borrow(cs).take().unwrap() };
        let mut alarm_2 = unsafe { ALARM_2.borrow(cs).take().unwrap() };
        alarm_2.clear_interrupt();
        alarm_2.schedule(alarm_2_duration).ok();
        unsafe { ALARM_2.borrow(cs).replace(Some(alarm_2)) };
        unsafe { ALARM_2_DURATION.borrow(cs).replace(Some(alarm_2_duration)) };
    });
}

#[interrupt]
fn TIMER_IRQ_3() {
    critical_section::with(|cs| {
        let alarm_3_duration = unsafe { ALARM_3_DURATION.borrow(cs).take().unwrap() };
        let mut alarm_3 = unsafe { ALARM_3.borrow(cs).take().unwrap() };
        alarm_3.clear_interrupt();
        alarm_3.schedule(alarm_3_duration).ok();
        unsafe { ALARM_3.borrow(cs).replace(Some(alarm_3)) };
        unsafe { ALARM_3_DURATION.borrow(cs).replace(Some(alarm_3_duration)) };
    });
}
//Handle UART data
#[interrupt]
fn UART1_IRQ() {
    critical_section::with(|cs| {
        let uart = unsafe { UART1_INST.borrow(cs).take().unwrap() };
        info!("here go");
        info!("{}", uart.uart_is_readable());
        let buf = singleton!(: [u8; 5] = [0; 5]).unwrap();
        uart.read_full_blocking(buf).unwrap();
        info!("{}", buf);

        unsafe { UART1_INST.borrow(cs).replace(Some(uart)) };
    })
}
