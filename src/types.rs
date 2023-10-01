use crate::{
    pac::I2C0, Gpio12, Gpio13, Gpio14, Gpio15, Gpio16, Gpio17, Gpio20, Gpio21, Gpio8, Gpio9, I2C,
};
use crate::{RotaryEncoder, StandardMode};
use rp2040_hal::{
    gpio::{FunctionI2c, FunctionSioInput, FunctionUart, Pin, PullDown, PullUp},
    pac::UART1,
    uart::{Enabled, UartPeripheral},
};

pub type Dac1SdaPin = rp2040_hal::gpio::Pin<Gpio16, FunctionI2c, PullDown>;
pub type Dac1SclPin = rp2040_hal::gpio::Pin<Gpio17, FunctionI2c, PullDown>;
pub type Dac1I2CType = I2C<I2C0, (Dac1SdaPin, Dac1SclPin)>;

pub type OledSdaPin = rp2040_hal::gpio::Pin<Gpio20, FunctionI2c, PullDown>;
pub type OledSclPin = rp2040_hal::gpio::Pin<Gpio21, FunctionI2c, PullDown>;
pub type OledI2CType = I2C<I2C0, (OledSdaPin, OledSclPin)>;

pub type TxPin = rp2040_hal::gpio::Pin<Gpio8, FunctionUart, PullDown>;
pub type RxPin = rp2040_hal::gpio::Pin<Gpio9, FunctionUart, PullDown>;
pub type UartType = UartPeripheral<Enabled, UART1, (TxPin, RxPin)>;

pub type RotaryEncoder1Type = RotaryEncoder<
    StandardMode,
    Pin<Gpio15, FunctionSioInput, PullUp>,
    Pin<Gpio14, FunctionSioInput, PullUp>,
>;

pub type RotaryEncoder2Type = RotaryEncoder<
    StandardMode,
    Pin<Gpio13, FunctionSioInput, PullUp>,
    Pin<Gpio12, FunctionSioInput, PullUp>,
>;
