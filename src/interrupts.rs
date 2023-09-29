use crate::{
    info, interrupt, singleton, Alarm, DAC, DAC_ALARM, ENCODER_1_POLL_ALARM, ENCODER_2_POLL_ALARM,
    ROTARY_ENCODER_1, ROTARY_ENCODER_2, UART1_INST,
};
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

// Rotary Encoder 1 Poll
#[interrupt]
fn TIMER_IRQ_1() {
    critical_section::with(|cs| {
        let rotary = unsafe { ROTARY_ENCODER_1.borrow(cs).take().unwrap() };
        let mut rotary_alarm = unsafe { ENCODER_1_POLL_ALARM.borrow(cs).take().unwrap() };
        rotary_alarm.clear_interrupt();

        unsafe { ROTARY_ENCODER_1.borrow(cs).replace(Some(rotary)) };
        unsafe { ENCODER_1_POLL_ALARM.borrow(cs).replace(Some(rotary_alarm)) };
    });
}

// Rotary Encoder 2 Poll
#[interrupt]
fn TIMER_IRQ_2() {
    critical_section::with(|cs| {
        let rotary = unsafe { ROTARY_ENCODER_2.borrow(cs).take().unwrap() };
        let mut rotary_alarm = unsafe { ENCODER_2_POLL_ALARM.borrow(cs).take().unwrap() };
        rotary_alarm.clear_interrupt();

        unsafe { ROTARY_ENCODER_2.borrow(cs).replace(Some(rotary)) };
        unsafe { ENCODER_2_POLL_ALARM.borrow(cs).replace(Some(rotary_alarm)) };
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
