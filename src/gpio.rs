// Taken from stm32f103xx-hal
//
// Copyright (c) 2017-2018 Jorge Aparicio
//               2021 Mathias Gottschlag
//
// Permission is hereby granted, free of charge, to any
// person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the
// Software without restriction, including without
// limitation the rights to use, copy, modify, merge,
// publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software
// is furnished to do so, subject to the following
// conditions:
//
// The above copyright notice and this permission notice
// shall be included in all copies or substantial portions
// of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF
// ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
// TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
// PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
// SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
// IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

//! General Purpose Input / Output

use core::marker::PhantomData;

use kl05_pac::SIM;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, sim: &mut SIM) -> Self::Parts;
}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
pub struct Floating;
/// Pulled down input (type state)
pub struct PullDown;
/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Push pull output (type state)
pub struct PushPull;
/// Open drain output (type state)
pub struct OpenDrain;

pub trait Alternate {
    const MUX: u8;
}

/// Analog/disabled (alternate function 0).
pub struct Analog;
impl Alternate for Analog {
    const MUX: u8 = 0;
}
/// Alternate function 2
pub struct Alternate2;
impl Alternate for Alternate2 {
    const MUX: u8 = 2;
}
/// Alternate function 3
pub struct Alternate3;
impl Alternate for Alternate3 {
    const MUX: u8 = 3;
}
/// Alternate function 4
pub struct Alternate4;
impl Alternate for Alternate4 {
    const MUX: u8 = 4;
}
/// Alternate function 5
pub struct Alternate5;
impl Alternate for Alternate5 {
    const MUX: u8 = 5;
}
/// Alternate function 6
pub struct Alternate6;
impl Alternate for Alternate6 {
    const MUX: u8 = 6;
}
/// Alternate function 7
pub struct Alternate7;
impl Alternate for Alternate7 {
    const MUX: u8 = 7;
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $PORTX:ident, $portx:ident, $PXx:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use core::marker::PhantomData;

            use super::super::hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, toggleable};
            use kl05_pac::{$PORTX, $GPIOX, $gpiox, SIM};

            use super::{
                Alternate, Floating, GpioExt, Input,
                // OpenDrain,
                Output,
                PullDown, PullUp,
                PushPull,
            };

            /// GPIO parts
            pub struct Parts {
                pub pddr: PDDR,
                $(
                    /// Pin
                    pub $pxi: $PXi<$MODE>,
                )+
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, sim: &mut SIM) -> Parts {
                    sim.scgc5.modify(|_, w| w.$portx().set_bit());

                    Parts {
                        pddr: PDDR { _0: () },
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }
            }

            /// Opaque PDDR register
            pub struct PDDR {
                _0: (),
            }

            impl PDDR {
                pub(crate) fn pddr(&mut self) -> &$gpiox::PDDR {
                    unsafe { &(*$GPIOX::ptr()).pddr }
                }
            }

            /// Partially erased pin
            pub struct $PXx<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> OutputPin for $PXx<Output<MODE>> {
                type Error = crate::NoError;

                fn set_high(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*$GPIOX::ptr()).psor.write(|w| w.bits(1 << self.i)) }
                    Ok(())
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*$GPIOX::ptr()).pcor.write(|w| w.bits(1 << self.i)) }
                    Ok(())
                }
            }

            impl <MODE> StatefulOutputPin for $PXx<Output<MODE>> {
                fn is_set_high(&self) -> Result<bool, Self::Error> {
                    self.is_set_low().map(|x| !x)
                }

                fn is_set_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).pdor.read().bits() & (1 << self.i) == 0 })
                }
            }

            impl <MODE> toggleable::Default for $PXx<Output<MODE>> {}

            $(
                /// Pin
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl<MODE> $PXi<MODE> {
                    /// Configures the pin to operate as an alternate function pin
                    pub fn into_alternate<ALTERNATE>(
                        self,
                        pddr: &mut PDDR,
                    ) -> $PXi<ALTERNATE>
                    where
                        ALTERNATE: Alternate,
                    {
                        unsafe {
                            // Configure GPIO as input.
                            pddr.pddr().modify(|r, w| w.pdd().bits(r.pdd().bits() & !(1 << $i)));
                            // Configure pin as alternate.
                            (*$PORTX::ptr()).pcr[$i].write(|w| w.bits(0)
                                                           .mux().bits(ALTERNATE::MUX) // GPIO
                                                           .dse().set_bit() // High drive strength
                                                           );
                        }

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a floating input pin
                    pub fn into_floating_input(
                        self,
                        pddr: &mut PDDR,
                    ) -> $PXi<Input<Floating>> {
                        unsafe {
                            // Configure GPIO as input.
                            pddr.pddr().modify(|r, w| w.pdd().bits(r.pdd().bits() & !(1 << $i)));
                            // Configure pin (no pullup/pulldown configured).
                            (*$PORTX::ptr()).pcr[$i].write(|w| w.bits(0)
                                                           .mux()._001() // GPIO
                                                           .dse().set_bit() // High drive strength
                                                           );
                        }

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pull-up input pin
                    pub fn into_pull_up_input(
                        self,
                        pddr: &mut PDDR,
                    ) -> $PXi<Input<PullUp>> {
                        unsafe {
                            // Configure GPIO as input.
                            pddr.pddr().modify(|r, w| w.pdd().bits(r.pdd().bits() & !(1 << $i)));
                            // Configure pin (enable pullup).
                            (*$PORTX::ptr()).pcr[$i].write(|w| w.bits(0)
                                                           .mux()._001() // GPIO
                                                           .dse().set_bit() // High drive strength
                                                           .pe().set_bit() // Pull enable
                                                           .ps().set_bit() // Pullup
                                                           );
                        }

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pull-down input pin
                    pub fn into_pull_down_input(
                        self,
                        pddr: &mut PDDR,
                    ) -> $PXi<Input<PullDown>> {
                        unsafe {
                            // Configure GPIO as input.
                            pddr.pddr().modify(|r, w| w.pdd().bits(r.pdd().bits() & !(1 << $i)));
                            // Configure pin (enable pulldown).
                            (*$PORTX::ptr()).pcr[$i].write(|w| w.bits(0)
                                                           .mux()._001() // GPIO
                                                           .dse().set_bit() // High drive strength
                                                           .pe().set_bit() // Pull enable
                                                           .ps().clear_bit() // Pulldown
                                                           );
                        }

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an push pull output pin
                    pub fn into_push_pull_output(
                        self,
                        pddr: &mut PDDR,
                    ) -> $PXi<Output<PushPull>> {
                        unsafe {
                            // Configure GPIO as output.
                            pddr.pddr().modify(|r, w| w.pdd().bits(r.pdd().bits() | (1 << $i)));
                            // Configure pin.
                            (*$PORTX::ptr()).pcr[$i].write(|w| w.bits(0)
                                                           .mux()._001() // GPIO
                                                           .dse().set_bit() // High drive strength
                                                           );
                        }

                        $PXi { _mode: PhantomData }
                    }
                }

                impl<MODE> $PXi<Output<MODE>> {
                    /// Erases the pin number from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> $PXx<Output<MODE>> {
                        $PXx {
                            i: $i,
                            _mode: self._mode,
                        }
                    }
                }

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    type Error = crate::NoError;

                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).psor.write(|w| w.bits(1 << $i)) }
                        Ok(())
                    }

                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).pcor.write(|w| w.bits(1 << $i)) }
                        Ok(())
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                    fn is_set_high(&self) -> Result<bool, Self::Error> {
                        self.is_set_low().map(|x| !x)
                    }

                    fn is_set_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).pdor.read().bits() & (1 << $i) == 0 })
                    }
                }

                impl <MODE> toggleable::Default for $PXi<Output<MODE>> {}

                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    type Error = crate::NoError;

                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|x| !x)
                    }

                    fn is_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).pdir.read().bits() & (1 << $i) == 0 })
                    }
                }
            )+
        }
    }
}

gpio!(GPIOA, gpioa, PORTA, porta, PAx, [
    PA0: (pa0, 0, Input<Floating>),
    PA1: (pa1, 1, Input<Floating>),
    PA2: (pa2, 2, Input<Floating>),
    PA3: (pa3, 3, Input<Floating>),
    PA4: (pa4, 4, Input<Floating>),
    PA5: (pa5, 5, Input<Floating>),
    PA6: (pa6, 6, Input<Floating>),
    PA7: (pa7, 7, Input<Floating>),
    PA8: (pa8, 8, Input<Floating>),
    PA9: (pa9, 9, Input<Floating>),
    PA10: (pa10, 10, Input<Floating>),
    PA11: (pa11, 11, Input<Floating>),
    PA12: (pa12, 12, Input<Floating>),
    PA13: (pa13, 13, Input<Floating>),
    PA14: (pa14, 14, Input<Floating>),
    PA15: (pa15, 15, Input<Floating>),
    PA16: (pa16, 16, Input<Floating>),
    PA17: (pa17, 17, Input<Floating>),
    PA18: (pa18, 18, Input<Floating>),
    PA19: (pa19, 19, Input<Floating>),
    PA20: (pa20, 20, Input<Floating>),
    PA21: (pa21, 21, Input<Floating>),
    PA22: (pa22, 22, Input<Floating>),
    PA23: (pa23, 23, Input<Floating>),
    PA24: (pa24, 24, Input<Floating>),
    PA25: (pa25, 25, Input<Floating>),
    PA26: (pa26, 26, Input<Floating>),
    PA27: (pa27, 27, Input<Floating>),
    PA28: (pa28, 28, Input<Floating>),
    PA29: (pa29, 29, Input<Floating>),
    PA30: (pa30, 30, Input<Floating>),
    PA31: (pa31, 31, Input<Floating>),
]);

gpio!(GPIOB, gpiob, PORTB, portb, PBx, [
    PB0: (pb0, 0, Input<Floating>),
    PB1: (pb1, 1, Input<Floating>),
    PB2: (pb2, 2, Input<Floating>),
    PB3: (pb3, 3, Input<Floating>),
    PB4: (pb4, 4, Input<Floating>),
    PB5: (pb5, 5, Input<Floating>),
    PB6: (pb6, 6, Input<Floating>),
    PB7: (pb7, 7, Input<Floating>),
    PB8: (pb8, 8, Input<Floating>),
    PB9: (pb9, 9, Input<Floating>),
    PB10: (pb10, 10, Input<Floating>),
    PB11: (pb11, 11, Input<Floating>),
    PB12: (pb12, 12, Input<Floating>),
    PB13: (pb13, 13, Input<Floating>),
    PB14: (pb14, 14, Input<Floating>),
    PB15: (pb15, 15, Input<Floating>),
    PB16: (pb16, 16, Input<Floating>),
    PB17: (pb17, 17, Input<Floating>),
    PB18: (pb18, 18, Input<Floating>),
    PB19: (pb19, 19, Input<Floating>),
    PB20: (pb20, 20, Input<Floating>),
    PB21: (pb21, 21, Input<Floating>),
    PB22: (pb22, 22, Input<Floating>),
    PB23: (pb23, 23, Input<Floating>),
    PB24: (pb24, 24, Input<Floating>),
    PB25: (pb25, 25, Input<Floating>),
    PB26: (pb26, 26, Input<Floating>),
    PB27: (pb27, 27, Input<Floating>),
    PB28: (pb28, 28, Input<Floating>),
    PB29: (pb29, 29, Input<Floating>),
    PB30: (pb30, 30, Input<Floating>),
    PB31: (pb31, 31, Input<Floating>),
]);
