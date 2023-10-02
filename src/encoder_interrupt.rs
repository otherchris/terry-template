use rotary_encoder_embedded::Direction;
use rp2040_hal::timer::Alarm;

use crate::{info, interrupt, ModuleState, MODULE_STATE};

#[interrupt]
fn TIMER_IRQ_3() {
    critical_section::with(|cs| {
        let module_state = unsafe { MODULE_STATE.borrow(cs).take().unwrap() };
        let ModuleState {
            mut encoder_poll_duration,
            mut encoder_poll_alarm,
            mut encoder_1,
            ..
        } = module_state;
        encoder_poll_alarm.clear_interrupt();
        encoder_poll_alarm.schedule(encoder_poll_duration).ok();

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
        unsafe {
            MODULE_STATE.borrow(cs).replace(Some(ModuleState {
                encoder_poll_duration,
                encoder_poll_alarm,
                encoder_1,
                ..module_state
            }))
        }
    });
}
