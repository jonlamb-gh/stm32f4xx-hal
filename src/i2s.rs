//! I2S bus

// NOTE: based on https://github.com/astro/stm32f429-hal/blob/master/src/i2s.rs

// TODO
// - port to the PINS pattern used in this crate, like in SPI
// - macro gen for I2S 2 and 3
// - as master mode support

use crate::gpio::gpiob::{PB10, PB12, PB13, PB14, PB15, PB9};
use crate::gpio::gpioc::PC6;
use crate::gpio::{Alternate, AF5, AF6};
use crate::rcc::Clocks;
use crate::stm32::{RCC, SPI2};
use core::marker::PhantomData;

/// SD: Serial Data (mapped on the MOSI pin) to transmit or receive
/// the two time- multiplexed data channels (in half-duplex mode
/// only).
pub unsafe trait SdPin<I2S> {}
unsafe impl SdPin<SPI2> for PB15<Alternate<AF5>> {}

/// WS: Word Select (mapped on the NSS pin) is the data control signal output in master
/// mode and input in slave mode.
pub unsafe trait WsPin<SPI1> {}
unsafe impl WsPin<SPI2> for PB9<Alternate<AF5>> {}
unsafe impl WsPin<SPI2> for PB12<Alternate<AF5>> {}

/// CK: Serial Clock (mapped on the SCK pin) is the serial clock output in master mode
/// and serial clock input in slave mode.
pub unsafe trait CkPin<I2S> {}
unsafe impl CkPin<SPI2> for PB10<Alternate<AF5>> {}
unsafe impl CkPin<SPI2> for PB13<Alternate<AF5>> {}

/// SPI2ext_SD and SPI3ext_SD: additional pins (mapped on the MISO pin) to control the
/// I 2 S full duplex mode.
pub unsafe trait ExtSdPin<I2S> {}
unsafe impl ExtSdPin<SPI2> for PB14<Alternate<AF6>> {}

/// MCK: Master Clock (mapped separately) is used, when the I 2 S is configured in master
/// mode (and when the MCKOE bit in the SPI_I2SPR register is set), to output this
/// additional clock generated at a preconfigured frequency rate equal to 256 × F S , where
/// F S is the audio sampling frequency.
pub unsafe trait MckPin<I2S> {}
unsafe impl MckPin<SPI2> for PC6<Alternate<AF5>> {}

/// Slave role (doesn't provide clock)
pub struct SlaveRole {}
/// Master role (provides clock)
pub struct MasterRole {}

/// I2S standard
pub enum I2sStandard {
    /// I2S Philips standard.
    Philips = 0b00,
    /// MSB justified standard (left justified)
    MsbJustified = 0b01,
    /// LSB justified standard (right justified)
    LsbJustified = 0b10,
    /// PCM standard
    Pcm = 0b11,
}

/// I2S peripheral
#[allow(unused)]
pub struct I2s<SPI, SD, CK, WS> {
    spi: SPI,
    sd: SD,
    ck: CK,
    ws: WS,
}

/// I2S peripheral
#[allow(unused)]
pub struct I2sOutput<Role, Data, SPI, SD, CK, WS> {
    role: PhantomData<Role>,
    data: PhantomData<Data>,
    spi: SPI,
    sd: SD,
    ck: CK,
    ws: WS,
}

/// Implemented by data types that fit the device's data width: `u16`,
/// and `u32`.
pub trait I2sData {
    /// Value for I2C `datlen` register field.
    fn datlen() -> u8;
    /// Run given `f` closure for each 16-bit part of the value.
    fn for_u16<F: Fn(u16)>(&self, f: F);
}

impl I2sData for u16 {
    fn datlen() -> u8 {
        0b00
    }
    #[inline]
    fn for_u16<F: Fn(u16)>(&self, f: F) {
        f(*self);
    }
}

impl I2sData for u32 {
    fn datlen() -> u8 {
        0b10
    }
    #[inline]
    fn for_u16<F: Fn(u16)>(&self, f: F) {
        f((*self >> 16) as u16);
        f(*self as u16);
    }
}

/// I2S interface on SPI pins
impl<SD, CK, WS> I2s<SPI2, SD, CK, WS> {
    pub fn i2s2(spi: SPI2, sd: SD, ck: CK, ws: WS, _clocks: Clocks) -> Self
    where
        SD: SdPin<SPI2>,
        CK: CkPin<SPI2>,
        WS: WsPin<SPI2>,
    {
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // Enable peripheral
        rcc.apb1enr.modify(|_, w| w.spi2en().set_bit());
        // Reset peripheral
        rcc.apb1rstr.modify(|_, w| w.spi2rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.spi2rst().clear_bit());

        I2s { spi, sd, ck, ws }
    }

    /// Configure in slave mode as output
    pub fn into_slave_output<S: I2sData>(
        self,
        standard: I2sStandard,
    ) -> I2sOutput<SlaveRole, S, SPI2, SD, CK, WS> {
        self.spi.i2scfgr.modify(|_, w| {
            unsafe {
                // Select I2S mode
                w.i2smod()
                    .set_bit()
                    // Configuration (slave, output)
                    .i2scfg()
                    .bits(0b00)
                    .i2sstd()
                    .bits(standard as u8)
                    // data length
                    .datlen()
                    .bits(S::datlen())
                    // "auto"
                    .chlen()
                    .clear_bit()
            }
        });
        // If needed, select all the potential interrupt
        // sources and the DMA capabilities by writing the
        // SPI_CR2 register.

        // The I2SE bit in SPI_I2SCFGR register must be
        // set.
        self.spi.i2scfgr.modify(|_, w| w.i2se().set_bit());

        I2sOutput {
            role: PhantomData,
            data: PhantomData,
            spi: self.spi,
            sd: self.sd,
            ck: self.ck,
            ws: self.ws,
        }
    }
}

impl<'s, Role, S: I2sData + Sized + 's, SD, CK, WS> I2sOutput<Role, S, SPI2, SD, CK, WS> {
    /// Disable and return `I2s`
    pub fn into_i2s(self) -> I2s<SPI2, SD, CK, WS> {
        // Wait
        while self.spi.sr.read().bsy().bit() || !self.spi.sr.read().txe().bit() {}
        // Disable first
        self.spi.i2scfgr.modify(|_, w| w.i2se().clear_bit());

        I2s {
            spi: self.spi,
            sd: self.sd,
            ck: self.ck,
            ws: self.ws,
        }
    }

    /// Write data word
    pub fn write(&mut self, data: S) {
        data.for_u16(|word| {
            while !self.spi.sr.read().txe().bit() {}
            self.spi.dr.write(|w| w.dr().bits(word));
        });
    }
}
