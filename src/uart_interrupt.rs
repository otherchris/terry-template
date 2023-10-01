use rotary_encoder_embedded::Direction;
use rp2040_hal::timer::Alarm;

use crate::{
    info, interrupt, singleton, ALARM_0, ALARM_0_DURATION, ALARM_1, ALARM_1_DURATION, ALARM_2,
    ALARM_2_DURATION, ENCODER_1, ENCODER_POLL_ALARM, ENCODER_POLL_DURATION, UART1_INST,
};

//Handle UART data
#[interrupt]
fn UART1_IRQ() {
    critical_section::with(|cs| {
        let uart = unsafe { UART1_INST.borrow(cs).take().unwrap() };
        info!("here go");
        info!("{}", uart.uart_is_readable());
        if uart.uart_is_readable() {
            let buf = singleton!(: [u8; 5] = [0; 5]).unwrap();
            uart.read_full_blocking(buf).unwrap();
            info!("{}", buf);
        }

        unsafe { UART1_INST.borrow(cs).replace(Some(uart)) };
    })
}
