use rotary_encoder_embedded::Direction;
use rp2040_hal::timer::Alarm;

use crate::{
    info, interrupt, singleton, ALARM_0, ALARM_0_DURATION, ALARM_1, ALARM_1_DURATION, ALARM_2,
    ALARM_2_DURATION, ENCODER_1, ENCODER_POLL_ALARM, ENCODER_POLL_DURATION, UART1_INST,
};

#[interrupt]
fn TIMER_IRQ_3() {
    critical_section::with(|cs| {
        let encoder_alarm_duration = unsafe { ENCODER_POLL_DURATION.borrow(cs).take().unwrap() };
        let mut encoder_alarm = unsafe { ENCODER_POLL_ALARM.borrow(cs).take().unwrap() };
        let mut encoder_1 = unsafe { ENCODER_1.borrow(cs).take().unwrap() };
        encoder_alarm.clear_interrupt();
        encoder_alarm.schedule(encoder_alarm_duration).ok();

        encoder_1.update();
        match encoder_1.direction() {
            Direction::Clockwise => {
                info!("Clockwise")
            }
            Direction::Anticlockwise => {
                info!("AntiClockwise")
            }
            Direction::None => {
                // info!("None")
            }
        }
        unsafe { ENCODER_POLL_ALARM.borrow(cs).replace(Some(encoder_alarm)) };
        unsafe {
            ENCODER_POLL_DURATION
                .borrow(cs)
                .replace(Some(encoder_alarm_duration))
        };
        unsafe { ENCODER_1.borrow(cs).replace(Some(encoder_1)) };
    });
}
