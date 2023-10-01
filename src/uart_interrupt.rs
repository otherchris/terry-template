use crate::{info, interrupt, singleton, UART1_INST};

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
