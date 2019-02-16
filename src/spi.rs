use core::{ptr, ops::Deref, sync::atomic::{self, Ordering}};

use cast::u16;
pub use crate::hal::spi::{Mode, Phase, Polarity};
use nb;
use crate::device::{SPI1, SPI2};

use crate::afio::MAPR;
use crate::dma::dma1;
use crate::gpio::Output;
use crate::gpio::gpioa::{PA5, PA6, PA7};
use crate::gpio::gpiob::{PB13, PB14, PB15, PB3, PB4, PB5};
use crate::gpio::gpioc::PC14;
use crate::gpio::{Alternate, Floating, Input, PushPull};
use crate::rcc::{APB1, APB2, Clocks};
use crate::time::Hertz;
use crate::time::U32Ext;
use crate::bb;

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)]
    _Extensible,
}

pub trait Pins<SPI> {
    const REMAP: bool;
}

impl Pins<SPI1>
for (
    PA5<Alternate<PushPull>>,
    PA6<Input<Floating>>,
    PA7<Alternate<PushPull>>,
)
{
    const REMAP: bool = false;
}

impl Pins<SPI1>
for (
    PB3<Alternate<PushPull>>,
    PB4<Input<Floating>>,
    PB5<Alternate<PushPull>>,
)
{
    const REMAP: bool = true;
}

impl Pins<SPI2>
for (
    PB13<Alternate<PushPull>>,
    PB14<Input<Floating>>,
    PB15<Alternate<PushPull>>,
)
{
    const REMAP: bool = false;
}

pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
    clock_freq: u32,
}

impl<PINS> Spi<SPI1, PINS> {
    pub fn spi1<F>(
        spi: SPI1,
        pins: PINS,
        mapr: &mut MAPR,
        mode: Mode,
        freq: F,
        clocks: Clocks,
        apb: &mut APB2,
    ) -> Self
        where
            F: Into<Hertz>,
            PINS: Pins<SPI1>,
    {
        mapr.mapr().modify(|_, w| w.spi1_remap().bit(PINS::REMAP));
        Spi::_spi1(spi, pins, mode, freq.into(), clocks.pclk2(), apb)
    }
}

impl<PINS> Spi<SPI2, PINS> {
    pub fn spi2<F>(
        spi: SPI2,
        pins: PINS,
        mode: Mode,
        freq: F,
        clocks: Clocks,
        apb: &mut APB1,
    ) -> Self
        where
            F: Into<Hertz>,
            PINS: Pins<SPI2>,
    {
        Spi::_spi2(spi, pins, mode, freq.into(), clocks.pclk1(), apb)
    }
}

fn compute_baudrate(clock: u32, freq: u32) -> u8 {
    match clock / freq {
        0 => unreachable!(),
        1...2 => 0b000,
        3...5 => 0b001,
        6...11 => 0b010,
        12...23 => 0b011,
        24...47 => 0b100,
        48...95 => 0b101,
        96...191 => 0b110,
        _ => 0b111,
    }
}

macro_rules! hal {
    ($($SPIX:ident: ($spiX:ident, $spiXen:ident, $spiXrst:ident, $APB:ident),)+) => {
        $(
            impl<PINS> Spi<$SPIX, PINS> {
                fn $spiX(
                    spi: $SPIX,
                    pins: PINS,
                    mode: Mode,
                    freq: Hertz,
                    bus_freq: Hertz,
                    apb: &mut $APB,
                ) -> Self {
                    // enable or reset $SPIX
                    apb.enr().modify(|_, w| w.$spiXen().enabled());
                    apb.rstr().modify(|_, w| w.$spiXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$spiXrst().clear_bit());

                    // disable SS output
                    spi.cr2.write(|w| w.ssoe().clear_bit());

                    let br = compute_baudrate(bus_freq.0 , freq.0);

                    // mstr: master configuration
                    // lsbfirst: MSB first
                    // ssm: enable software slave management (NSS pin free for other uses)
                    // ssi: set nss high = master mode
                    // dff: 8 bit frames
                    // bidimode: 2-line unidirectional
                    // spe: enable the SPI bus
                    spi.cr1.write(|w| {
                        w.cpha()
                            .bit(mode.phase == Phase::CaptureOnSecondTransition)
                            .cpol()
                            .bit(mode.polarity == Polarity::IdleHigh)
                            .mstr()
                            .set_bit()
                            .br()
                            .bits(br)
                            .lsbfirst()
                            .clear_bit()
                            .ssm()
                            .set_bit()
                            .ssi()
                            .set_bit()
                            .rxonly()
                            .clear_bit()
                            .dff()
                            .clear_bit()
                            .bidimode()
                            .clear_bit()
                            .spe()
                            .set_bit()
                    });

                    Spi { spi, pins, clock_freq: bus_freq.0 }
                }

                pub fn free(self) -> ($SPIX, PINS) {
                    (self.spi, self.pins)
                }

                pub fn change_baud_rate(&mut self, freq: Hertz) {
                    let br = compute_baudrate(self.clock_freq , freq.0);
                    self.spi.cr1.modify(|_, w| { w.br().bits(br) });
                }
            }

            impl<PINS> crate::hal::spi::FullDuplex<u8> for Spi<$SPIX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.rxne().bit_is_set() {
                        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
                        // reading a half-word)
                        return Ok(unsafe {
                            ptr::read_volatile(&self.spi.dr as *const _ as *const u8)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.txe().bit_is_set() {
                        // NOTE(write_volatile) see note above
                        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
                        return Ok(());
                    } else {
                        nb::Error::WouldBlock
                    })
                }

            }

            impl<PINS> crate::hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS> {}

            impl<PINS> crate::hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS> {}
        )+
    }
}

hal! {
    SPI1: (_spi1, spi1en, spi1rst, APB2),
    SPI2: (_spi2, spi2en, spi2rst, APB1),
}

pub trait SpiDmaChannels {
    type RxChannel;
    type TxChannel;
}

impl SpiDmaChannels for (dma1::C2, dma1::C3) {
    type RxChannel = dma1::C2;
    type TxChannel = dma1::C3;
}

pub struct DmaSpi<PINS: Pins<SPI2>> {
    rx_dma: dma1::C4,
    tx_dma: dma1::C5,
    spi: Spi<SPI2, PINS>,
}

impl<PINS: Pins<SPI2>> DmaSpi<PINS> {
    pub fn new(spi: Spi<SPI2, PINS>, mut rx_dma: dma1::C4, mut tx_dma: dma1::C5) -> Self {
        rx_dma.cpar().write(|w| unsafe {
            w.pa().bits(&(*spi.spi.deref()).dr as *const _ as usize as u32)
        });
        rx_dma.ccr().modify(|_, w| {
            w.mem2mem()
                .clear_bit()
                .pl()
                .medium()
                .msize()
                .bit8()
                .psize()
                .bit8()
                .minc()
                .set_bit()
                .pinc()
                .clear_bit()
                .circ()
                .clear_bit()
                .dir()
                .clear_bit()
                .en()
                .clear_bit()
        });

        tx_dma.cpar().write(|w| unsafe {
            w.pa().bits(&(*spi.spi.deref()).dr as *const _ as usize as u32)
        });
        tx_dma.ccr().modify(|_, w| {
            w.mem2mem()
                .clear_bit()
                .pl()
                .medium()
                .msize()
                .bit8()
                .psize()
                .bit8()
                .minc()
                .set_bit()
                .pinc()
                .clear_bit()
                .circ()
                .clear_bit()
                .dir()
                .set_bit()
                .en()
                .clear_bit()
        });

        spi.spi.cr2.modify(|_, w| w.txdmaen().set_bit());
        atomic::compiler_fence(Ordering::SeqCst);
        DmaSpi { spi, rx_dma, tx_dma }
    }

    pub fn release(self) -> (Spi<SPI2, PINS>, dma1::C4, dma1::C5) {
        (self.spi, self.rx_dma, self.tx_dma)
    }

    pub fn change_baud_rate(&mut self, freq: Hertz) {
        self.spi.change_baud_rate(freq);
    }
}

impl<PINS: Pins<SPI2>> crate::hal::blocking::spi::Transfer<u8> for DmaSpi<PINS> {
    type Error = Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        let spi = &mut self.spi.spi;
        if spi.sr.read().ovr().bit_is_set() {
            return Err(Error::Overrun);
        }
        let rx_chan = &mut self.rx_dma;
        rx_chan.cmar().write(|w| unsafe {
            w.ma().bits(words.as_ptr() as usize as u32)
        });
        rx_chan.cndtr().write(|w| unsafe {
            w.ndt().bits(u16(words.len()).unwrap())
        });
        let tx_chan = &mut self.tx_dma;
        tx_chan.cmar().write(|w| unsafe {
            w.ma().bits(words.as_ptr() as usize as u32)
        });
        tx_chan.cndtr().write(|w| unsafe {
            w.ndt().bits(u16(words.len()).unwrap())
        });
        bb::set(rx_chan.ccr(), 0/*EN*/);
        bb::set(tx_chan.ccr(), 0/*EN*/);
        let words = tx_chan.wait_for_tranfer(words);
        let words = rx_chan.wait_for_tranfer(words);
        loop {
            let sr = spi.sr.read();
            if sr.ovr().bit_is_set() {
                return Err(Error::Overrun);
            } else if sr.modf().bit_is_set() {
                return Err(Error::ModeFault);
            } else if sr.crcerr().bit_is_set() {
                return Err(Error::Crc);
            } else if sr.txe().bit_is_set() && spi.sr.read().bsy().bit_is_clear() {
                return Ok(words);
            }
        }
    }
}

impl<PINS: Pins<SPI2>> crate::hal::blocking::spi::Write<u8> for DmaSpi<PINS> {
    type Error = Error;
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let spi = &mut self.spi.spi;
        bb::clear(&(spi.deref()).cr2, 0/*RXDMAEN*/);
        let tx_chan = &mut self.tx_dma;
        tx_chan.cmar().write(|w| unsafe {
            w.ma().bits(words.as_ptr() as usize as u32)
        });
        tx_chan.cndtr().write(|w| unsafe {
            w.ndt().bits(u16(words.len()).unwrap())
        });
        atomic::compiler_fence(Ordering::Release);
        bb::set(tx_chan.ccr(), 0/*EN*/);
        let result = loop {
            let sr = spi.sr.read();
            // don't care about OVR, will be cleared later
            if sr.modf().bit_is_set() {
                break Err(Error::ModeFault);
            } else if sr.crcerr().bit_is_set() {
                break Err(Error::Crc);
            } else if sr.txe().bit_is_set() {
                tx_chan.wait_for_tranfer(words);
                break Ok(());
            }
        };
        // block for the whole communication because the caller will release the slave select upon return
        loop {
            let sr = spi.sr.read();
            if sr.ovr().bit_is_set() {
                // clear ovr
                unsafe { ptr::read_volatile(&spi.dr as *const _ as *const u8) };
            } else if sr.bsy().bit_is_clear() {
                break;
            }
        }
        bb::set(&(spi.deref()).cr2, 0/*RXDMAEN*/);
        result
    }
}

