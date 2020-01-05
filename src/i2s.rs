//! I2S bus
//!
//! This was based on
//! https://github.com/astro/stm32f429-hal/blob/master/src/i2s.rs

use crate::gpio::gpiob::{PB10, PB12, PB13, PB14, PB15, PB9};
use crate::gpio::gpioc::{PC2, PC6};
use crate::gpio::{Alternate, AF5, AF6};
use crate::rcc::Clocks;
use crate::stm32::{I2S2EXT, RCC, SPI2};
use core::marker::PhantomData;

/// I2S error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Underrun occurred
    Underrun,
    /// Frame format error occured
    FrameFormat,
    #[doc(hidden)]
    _Extensible,
}

pub trait Pins<I2S> {}

/// SD: Serial Data (mapped on the MOSI pin) to transmit or receive
/// the two time- multiplexed data channels (in half-duplex mode
/// only).
pub trait PinSd<I2S> {}
impl PinSd<SPI2> for PB15<Alternate<AF5>> {}

/// WS: Word Select (mapped on the NSS pin) is the data control signal output in master
/// mode and input in slave mode.
pub trait PinWs<SPI1> {}
impl PinWs<SPI2> for PB9<Alternate<AF5>> {}
impl PinWs<SPI2> for PB12<Alternate<AF5>> {}

/// CK: Serial Clock (mapped on the SCK pin) is the serial clock output in master mode
/// and serial clock input in slave mode.
pub trait PinCk<I2S> {}
impl PinCk<SPI2> for PB10<Alternate<AF5>> {}
impl PinCk<SPI2> for PB13<Alternate<AF5>> {}

/// SPI2ext_SD and SPI3ext_SD: additional pins (mapped on the MISO pin) to control the
/// I 2 S full duplex mode.
pub trait PinExtSd<I2S> {}
impl PinExtSd<SPI2> for PB14<Alternate<AF6>> {}
impl PinExtSd<SPI2> for PC2<Alternate<AF6>> {}

/// MCK: Master Clock (mapped separately) is used, when the I 2 S is configured in master
/// mode (and when the MCKOE bit in the SPI_I2SPR register is set), to output this
/// additional clock generated at a preconfigured frequency rate equal to 256 × F S , where
/// F S is the audio sampling frequency.
pub trait PinMck<I2S> {}
impl PinMck<SPI2> for PC6<Alternate<AF5>> {}

impl<I2S, SD, CK, WS, MCK> Pins<I2S> for (SD, CK, WS, MCK)
where
    SD: PinSd<I2S>,
    CK: PinCk<I2S>,
    WS: PinWs<I2S>,
    MCK: PinMck<I2S>,
{
}

/// Slave role (doesn't provide clock)
pub struct SlaveRole {}
/// Master role (provides clock)
pub struct MasterRole {}

/// I2S standard
#[derive(Debug, Copy, Clone, PartialEq)]
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

pub trait Write<W: I2sData + Sized> {
    type Error;

    fn write(&mut self, words: &[W]) -> Result<(), Self::Error>;
}

pub trait Read<W: I2sData + Sized> {
    type Error;

    fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error>;
}

/// I2S peripheral
#[allow(unused)]
pub struct I2s<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

/// I2S transmitter
#[allow(unused)]
pub struct I2sOutput<Role, Data, SPI, PINS> {
    role: PhantomData<Role>,
    data: PhantomData<Data>,
    spi: SPI,
    pins: PINS,
}

/// I2S transmitter and receiver, full-duplex
#[allow(unused)]
pub struct I2sFullDuplex<Role, Data, SPI, EXT, PINS, SD_EXT> {
    role: PhantomData<Role>,
    data: PhantomData<Data>,
    spi: SPI,
    ext: EXT,
    pins: PINS,
    pin_sd_ext: SD_EXT,
}

/// Implemented by data types that fit the device's data width: `u16`,
/// and `u32`.
pub trait I2sData {
    /// Value for I2C `datlen` register field.
    fn datlen() -> u8;
    /// Run given `f` closure for each 16-bit part of the value.
    #[allow(patterns_in_fns_without_body)]
    fn for_u16<F: FnMut(u16) -> Result<(), Error>>(&self, mut f: F) -> Result<(), Error>;
}

impl I2sData for u16 {
    fn datlen() -> u8 {
        0b00
    }
    #[inline]
    fn for_u16<F: FnMut(u16) -> Result<(), Error>>(&self, mut f: F) -> Result<(), Error> {
        f(*self)
    }
}

impl I2sData for u32 {
    fn datlen() -> u8 {
        0b10
    }
    #[inline]
    fn for_u16<F: FnMut(u16) -> Result<(), Error>>(&self, mut f: F) -> Result<(), Error> {
        f((*self >> 16) as u16)?;
        f(*self as u16)
    }
}

/// I2S interface on SPI pins
impl<PINS> I2s<SPI2, PINS> {
    pub fn i2s2(spi: SPI2, pins: PINS, _clocks: Clocks) -> Self
    where
        PINS: Pins<SPI2>,
    {
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // Enable peripheral
        rcc.apb1enr.modify(|_, w| w.spi2en().set_bit());
        // Reset peripheral
        rcc.apb1rstr.modify(|_, w| w.spi2rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.spi2rst().clear_bit());

        rcc.cr.modify(|_, w| w.plli2son().set_bit());
        while !rcc.cr.read().plli2srdy().bit() {}

        // TODO - values taken from the examples
        rcc.plli2scfgr
            .modify(|_, w| unsafe { w.plli2sn().bits(86).plli2sr().bits(4).plli2sq().bits(0) });

        I2s { spi, pins }
    }

    /// Configure in master mode as output, half-duplex
    pub fn into_master_output<S: I2sData>(
        self,
        standard: I2sStandard,
    ) -> I2sOutput<MasterRole, S, SPI2, PINS> {
        // TODO - frequency provided as param
        // for now assume:
        // - I2S_AUDIOFREQ_44K == 44100
        self.spi.i2spr.modify(|_, w| {
            unsafe {
                // Master clock output disabled
                w.mckoe()
                    .clear_bit()
                    // With master mode and 44K, always zero
                    .odd()
                    .clear_bit()
                    // 44K divider
                    .i2sdiv()
                    .bits(15)
            }
        });

        // TODO
        // - PCMSYNC used with PCM standard
        // - assume I2S_CPOL_LOW
        self.spi.i2scfgr.modify(|_, w| {
            unsafe {
                // Select I2S mode
                w.i2smod()
                    .set_bit()
                    // Configuration (master, output)
                    .i2scfg()
                    .bits(0b10)
                    .i2sstd()
                    .bits(standard as u8)
                    // Polarity
                    .ckpol()
                    .clear_bit()
                    // Data length
                    .datlen()
                    .bits(S::datlen())
                    // "auto" / 16-bit
                    .chlen()
                    .clear_bit()
            }
        });

        // If needed, select all the potential interrupt
        // sources and the DMA capabilities by writing the
        // SPI_CR2 register.

        // Enable I2S
        self.spi.i2scfgr.modify(|_, w| w.i2se().set_bit());

        I2sOutput {
            role: PhantomData,
            data: PhantomData,
            spi: self.spi,
            pins: self.pins,
        }
    }

    /// Master transmit, slave receive
    pub fn into_master_full_duplex<S: I2sData, SD_EXT: PinExtSd<SPI2>>(
        self,
        ext: I2S2EXT,
        pin_sd_ext: SD_EXT,
        standard: I2sStandard,
    ) -> I2sFullDuplex<MasterRole, S, SPI2, I2S2EXT, PINS, SD_EXT> {
        // Enable receiver before enabling I2S
        ext.i2scfgr.modify(|_, w| w.i2se().set_bit());
        //ext.i2scfgr.modify(|_, w| w.i2se().clear_bit());

        // Clear overrun, read DR, read SR
        let _ = ext.dr.read().bits();
        let _ = ext.sr.read().bits();

        // Configure I2S, master tx
        let master_out = self.into_master_output::<S>(standard);
        let spi = master_out.spi;
        let pins = master_out.pins;

        // Configure I2S_ext, slave rx
        ext.i2spr
            .modify(|_, w| unsafe { w.mckoe().clear_bit().odd().clear_bit().i2sdiv().bits(2) });
        //.modify(|_, w| unsafe { w.mckoe().clear_bit().odd().clear_bit().i2sdiv().bits(15) });

        ext.i2scfgr.modify(|_, w| {
            unsafe {
                // Select I2S mode
                w.i2smod()
                    .set_bit()
                    // Configuration (slave, input))
                    .i2scfg()
                    .bits(0b01)
                    .i2sstd()
                    .bits(standard as u8)
                    // Polarity
                    .ckpol()
                    .clear_bit()
                    // Data length
                    .datlen()
                    .bits(S::datlen())
                    // "auto" / 16-bit
                    .chlen()
                    .clear_bit()
            }
        });

        // Enable I2S
        ext.i2scfgr.modify(|_, w| w.i2se().set_bit());

        I2sFullDuplex {
            role: PhantomData,
            data: PhantomData,
            spi,
            ext,
            pins,
            pin_sd_ext,
        }
    }
}

impl<'s, Role, S: I2sData + Sized + 's, PINS> I2sOutput<Role, S, SPI2, PINS> {
    /// Disable and return `I2s`
    pub fn into_i2s(self) -> I2s<SPI2, PINS> {
        // Wait
        while self.spi.sr.read().bsy().bit() || !self.spi.sr.read().txe().bit() {}
        // Disable first
        self.spi.i2scfgr.modify(|_, w| w.i2se().clear_bit());

        I2s {
            spi: self.spi,
            pins: self.pins,
        }
    }

    pub fn write_word(&mut self, data: S) -> Result<(), Error> {
        data.for_u16(|word| nb::block!(self.send(word)))?;
        Ok(())
    }

    pub fn send(&mut self, data: u16) -> nb::Result<(), Error> {
        let sr = self.spi.sr.read();

        Err(if sr.ovr().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if sr.fre().bit_is_set() {
            nb::Error::Other(Error::FrameFormat)
        } else if sr.udr().bit_is_set() {
            nb::Error::Other(Error::Underrun)
        } else if sr.txe().bit_is_set() {
            self.spi.dr.write(|w| w.dr().bits(data));
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }
}

impl<Role, PINS> Write<u16> for I2sOutput<Role, u16, SPI2, PINS> {
    type Error = Error;

    fn write(&mut self, words: &[u16]) -> Result<(), Self::Error> {
        for w in words.iter() {
            self.write_word(*w)?;
        }
        Ok(())
    }
}

impl<'s, Role, S: I2sData + Sized + 's, PINS, SD_EXT>
    I2sFullDuplex<Role, S, SPI2, I2S2EXT, PINS, SD_EXT>
{
    pub fn write_word(&mut self, data: S) -> Result<(), Error> {
        data.for_u16(|word| nb::block!(self.send(word)))?;
        Ok(())
    }

    // TODO - handle u32
    pub fn read_word(&mut self) -> Result<u16, Error> {
        let data = nb::block!(self.read())?;
        Ok(data)
    }

    pub fn send(&mut self, data: u16) -> nb::Result<(), Error> {
        let sr = self.spi.sr.read();

        Err(if sr.ovr().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if sr.fre().bit_is_set() {
            nb::Error::Other(Error::FrameFormat)
        } else if sr.udr().bit_is_set() {
            nb::Error::Other(Error::Underrun)
        } else if sr.txe().bit_is_set() {
            self.spi.dr.write(|w| w.dr().bits(data));
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }

    pub fn read(&mut self) -> nb::Result<u16, Error> {
        let sr = self.ext.sr.read();

        Err(if sr.ovr().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if sr.fre().bit_is_set() {
            nb::Error::Other(Error::FrameFormat)
        } else if sr.udr().bit_is_set() {
            nb::Error::Other(Error::Underrun)
        } else if sr.rxne().bit_is_set() {
            panic!("GOT RX DATA");
            return Ok(self.ext.dr.read().dr().bits());
        } else {
            let bits = sr.bits();
            if bits != 0x2 {
                panic!("0x{:X}", bits);
            }
            nb::Error::WouldBlock
        })
    }
}

impl<Role, PINS, SD_EXT> Write<u16> for I2sFullDuplex<Role, u16, SPI2, I2S2EXT, PINS, SD_EXT> {
    type Error = Error;

    fn write(&mut self, words: &[u16]) -> Result<(), Self::Error> {
        for w in words.iter() {
            self.write_word(*w)?;
        }
        Ok(())
    }
}

impl<Role, PINS, SD_EXT> Read<u16> for I2sFullDuplex<Role, u16, SPI2, I2S2EXT, PINS, SD_EXT> {
    type Error = Error;

    fn read(&mut self, words: &mut [u16]) -> Result<(), Self::Error> {
        for w in words {
            *w = self.read_word()?;
        }
        Ok(())
    }
}
