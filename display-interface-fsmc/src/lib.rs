#![no_std]

use core::ops::Deref;
use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use stm32f1xx_hal::pac::{FSMC, GPIOD, GPIOE};

/// FSMC NORSRAM Configuration Structure definition
pub struct FsmcNorsramInitTypeDef {
    /// Specifies the NORSRAM memory device that will be used.
    /// This parameter can be a value of @ref FSMC_NORSRAM_Bank
    pub ns_bank: u32,
    /// Specifies whether the address and data values are
    /// multiplexed on the data bus or not.
    /// This parameter can be a value of @ref FSMC_Data_Address_Bus_Multiplexing
    pub data_address_mux: u32,
    /// Specifies the type of external memory attached to
    /// the corresponding memory device.
    /// This parameter can be a value of @ref FSMC_Memory_Type
    pub memory_type: u32,
    /// Specifies the external memory device width.
    /// This parameter can be a value of @ref FSMC_NORSRAM_Data_Width
    pub memory_data_width: u32,
    /// Enables or disables the burst access mode for Flash memory,
    /// valid only with synchronous burst Flash memories.
    /// This parameter can be a value of @ref FSMC_Burst_Access_Mode
    pub burst_access_mode: u32,
    /// Specifies the wait signal polarity, valid only when accessing
    /// the Flash memory in burst mode.
    /// This parameter can be a value of @ref FSMC_Wait_Signal_Polarity
    pub wait_signal_polarity: u32,
    /// Enables or disables the Wrapped burst access mode for Flash
    /// memory, valid only when accessing Flash memories in burst mode.
    /// This parameter can be a value of @ref FSMC_Wrap_Mode
    pub wrap_mode: u32,
    /// Specifies if the wait signal is asserted by the memory one
    /// clock cycle before the wait state or during the wait state,
    /// valid only when accessing memories in burst mode.
    /// This parameter can be a value of @ref FSMC_Wait_Timing
    pub wait_signal_active: u32,
    /// Enables or disables the write operation in the selected device by the FSMC.
    /// This parameter can be a value of @ref FSMC_Write_Operation
    pub write_operation: u32,
    /// Enables or disables the wait state insertion via wait
    /// signal, valid for Flash memory access in burst mode.
    /// This parameter can be a value of @ref FSMC_Wait_Signal
    pub wait_signal: u32,
    /// Enables or disables the extended mode.
    /// This parameter can be a value of @ref FSMC_Extended_Mode
    pub extended_mode: u32,
    /// Enables or disables wait signal during asynchronous transfers,
    /// valid only with asynchronous Flash memories.
    /// This parameter can be a value of @ref FSMC_AsynchronousWait
    pub asynchronous_wait: u32,
    /// Enables or disables the write burst operation.
    /// This parameter can be a value of @ref FSMC_Write_Burst
    pub write_burst: u32,
    /// Specifies the memory page size.
    /// This parameter can be a value of @ref FSMC_Page_Size
    pub page_size: u32,
}

/// FSMC NORSRAM Timing parameters structure definition
pub struct FsmcNorsramTimingTypeDef {
    /// Defines the number of HCLK cycles to configure
    /// the duration of the address setup time.
    /// This parameter can be a value between Min_Data = 0 and Max_Data = 15.
    /// @note This parameter is not used with synchronous NOR Flash memories.
    pub address_setup_time: u32,
    /// Defines the number of HCLK cycles to configure
    /// the duration of the address hold time.
    /// This parameter can be a value between Min_Data = 1 and Max_Data = 15.
    /// @note This parameter is not used with synchronous NOR Flash memories.
    pub address_hold_time: u32,
    /// Defines the number of HCLK cycles to configure
    /// the duration of the data setup time.
    /// This parameter can be a value between Min_Data = 1 and Max_Data = 255.
    /// @note This parameter is used for SRAMs, ROMs and asynchronous multiplexed
    /// NOR Flash memories.
    pub data_setup_time: u32,
    /// Defines the number of HCLK cycles to configure
    /// the duration of the bus turnaround.
    /// This parameter can be a value between Min_Data = 0 and Max_Data = 15.
    /// @note This parameter is only used for multiplexed NOR Flash memories.
    pub bus_turn_around_duration: u32,
    /// Defines the period of CLK clock output signal, expressed in number of
    /// HCLK cycles. This parameter can be a value between Min_Data = 2 and
    /// Max_Data = 16.
    /// @note This parameter is not used for asynchronous NOR Flash, SRAM or ROM
    /// accesses.
    pub clk_division: u32,
    /// Defines the number of memory clock cycles to issue
    /// to the memory before getting the first data.
    /// The parameter value depends on the memory type as shown below:
    /// - It must be set to 0 in case of a CRAM
    /// - It is don't care in asynchronous NOR, SRAM or ROM
    /// accesses
    /// - It may assume a value between Min_Data = 2 and Max_Data = 17
    /// in NOR Flash memories with synchronous burst mode enable
    pub data_latency: u32,
    /// Specifies the asynchronous access mode.
    /// This parameter can be a value of @ref FSMC_Access_Mode
    pub access_mode: u32,
}

pub struct SramHandleTypeDef<'a> {
    pub device: &'a FSMC,
    pub init: FsmcNorsramInitTypeDef,
    pub timing: FsmcNorsramTimingTypeDef,
    pub ext_timing: FsmcNorsramTimingTypeDef,
}

pub fn fsmc_norsram_init(device: &FSMC, init: &FsmcNorsramInitTypeDef) {
    let bcr1 = device.deref().bcr1.read().bits();
    device.deref().bcr1.modify(|_, w| w.mbken().clear_bit());
    let mask = 0xFFF7Fu32;
    let btcr_reg = init.data_address_mux
        | init.memory_type
        | init.memory_data_width
        | init.burst_access_mode
        | init.wait_signal_polarity
        | init.wait_signal_active
        | init.write_operation
        | init.wait_signal
        | init.extended_mode
        | init.asynchronous_wait
        | init.write_burst;
    // let btcr_reg = 0x5010;
    device.deref().bcr1.write(|w| unsafe {
        w.bits((bcr1 & mask) | btcr_reg)
        // FIXME: NOT CORRECT FIELDS
        // w.mtyp().bits(init.memory_type as u8);
        // w.mwid().bits(init.memory_data_width as u8);
        // w.bursten().bit(init.burst_access_mode != 0);
        // w.waitpol().bit(init.wait_signal_polarity != 0);
        // w.waiten().bit(init.wait_signal_active != 0);
        // w.wren().bit(init.write_operation != 0);
        // w.waiten().bit(init.wait_signal != 0);
        // w.extmod().bit(init.extended_mode != 0);
        // w.asyncwait().bit(init.asynchronous_wait != 0);
        // w.cburstrw().bit(init.write_burst != 0)
    });
}

pub fn fsmc_norsram_timing_init(device: &FSMC, timing: &FsmcNorsramTimingTypeDef) {
    device.deref().btr1.write(|w| unsafe {
        w.addset().bits(timing.address_setup_time as u8);
        w.addhld().bits(timing.address_hold_time as u8);
        w.datast().bits(timing.data_setup_time as u8);
        w.busturn().bits(timing.bus_turn_around_duration as u8);
        w.clkdiv().bits((timing.clk_division - 1) as u8);
        w.datlat().bits((timing.data_latency - 2) as u8);
        w.accmod().bits(timing.access_mode as u8)
        // w.bits(0x0ff00ff0u32)
    });
}

pub fn fsmc_norsram_extended_timing_init(device: &FSMC, timing: &FsmcNorsramTimingTypeDef) {
    device.deref().bwtr1.write(|w| unsafe {
        w.addset().bits(timing.address_setup_time as u8);
        w.addhld().bits(timing.address_hold_time as u8);
        w.datast().bits(timing.data_setup_time as u8);
        w.busturn().bits(timing.bus_turn_around_duration as u8);
        w.accmod().bits(timing.access_mode as u8)
        // w.bits(0x0ff001f0u32)
    });
}

pub fn hal_sram_init(
    hsram: &SramHandleTypeDef,
    timing: &FsmcNorsramTimingTypeDef,
    ext_timing: &FsmcNorsramTimingTypeDef,
) {
    fsmc_norsram_init(hsram.device, &hsram.init);
    fsmc_norsram_timing_init(hsram.device, timing);
    fsmc_norsram_extended_timing_init(hsram.device, ext_timing);
    // enable device
    hsram.device.deref().bcr1.modify(|_, w| w.mbken().set_bit());
}

pub fn hal_fsmc_msp_init(gpioe: GPIOE, gpiod: GPIOD) {
    /*
        FSMC GPIO Configuration
        PE7    ------> FSMC_D4
        PE8    ------> FSMC_D5
        PE9    ------> FSMC_D6
        PE10   ------> FSMC_D7
        PE11   ------> FSMC_D8
        PE12   ------> FSMC_D9
        PE13   ------> FSMC_D10
        PE14   ------> FSMC_D11
        PE15   ------> FSMC_D12
        PD8    ------> FSMC_D13
        PD9    ------> FSMC_D14
        PD10   ------> FSMC_D15
        PD11   ------> FSMC_A16
        PD14   ------> FSMC_D0
        PD15   ------> FSMC_D1
        PD0    ------> FSMC_D2
        PD1    ------> FSMC_D3
        PD4    ------> FSMC_NOE
        PD5    ------> FSMC_NWE
        PD7    ------> FSMC_NE1
    */
    // FIXME: not working
    // let mut gpioe = gpioe.split();
    // let mut gpiod = gpiod.split();
    // let _data_pins = (
    //     gpiod
    //         .pd14
    //         .into_push_pull_output(&mut gpiod.crh)
    //         .set_speed(&mut gpiod.crh, IOPinSpeed::Mhz50),
    //     gpiod
    //         .pd15
    //         .into_push_pull_output(&mut gpiod.crh)
    //         .set_speed(&mut gpiod.crh, IOPinSpeed::Mhz50),
    //     gpiod
    //         .pd0
    //         .into_push_pull_output(&mut gpiod.crl)
    //         .set_speed(&mut gpiod.crl, IOPinSpeed::Mhz50),
    //     gpiod
    //         .pd1
    //         .into_push_pull_output(&mut gpiod.crl)
    //         .set_speed(&mut gpiod.crl, IOPinSpeed::Mhz50),
    //     gpioe
    //         .pe7
    //         .into_push_pull_output(&mut gpioe.crl)
    //         .set_speed(&mut gpioe.crl, IOPinSpeed::Mhz50),
    //     gpioe
    //         .pe8
    //         .into_push_pull_output(&mut gpioe.crh)
    //         .set_speed(&mut gpioe.crh, IOPinSpeed::Mhz50),
    //     gpioe
    //         .pe9
    //         .into_push_pull_output(&mut gpioe.crh)
    //         .set_speed(&mut gpioe.crh, IOPinSpeed::Mhz50),
    //     gpioe
    //         .pe10
    //         .into_push_pull_output(&mut gpioe.crh)
    //         .set_speed(&mut gpioe.crh, IOPinSpeed::Mhz50),
    //     gpioe
    //         .pe11
    //         .into_push_pull_output(&mut gpioe.crh)
    //         .set_speed(&mut gpioe.crh, IOPinSpeed::Mhz50),
    //     gpioe
    //         .pe12
    //         .into_push_pull_output(&mut gpioe.crh)
    //         .set_speed(&mut gpioe.crh, IOPinSpeed::Mhz50),
    //     gpioe
    //         .pe13
    //         .into_push_pull_output(&mut gpioe.crh)
    //         .set_speed(&mut gpioe.crh, IOPinSpeed::Mhz50),
    //     gpioe
    //         .pe14
    //         .into_push_pull_output(&mut gpioe.crh)
    //         .set_speed(&mut gpioe.crh, IOPinSpeed::Mhz50),
    //     gpioe
    //         .pe15
    //         .into_push_pull_output(&mut gpioe.crh)
    //         .set_speed(&mut gpioe.crh, IOPinSpeed::Mhz50),
    //     gpiod
    //         .pd8
    //         .into_push_pull_output(&mut gpiod.crh)
    //         .set_speed(&mut gpiod.crh, IOPinSpeed::Mhz50),
    //     gpiod
    //         .pd9
    //         .into_push_pull_output(&mut gpiod.crh)
    //         .set_speed(&mut gpiod.crh, IOPinSpeed::Mhz50),
    //     gpiod
    //         .pd10
    //         .into_push_pull_output(&mut gpiod.crh)
    //         .set_speed(&mut gpiod.crh, IOPinSpeed::Mhz50),
    // );
    // let _rs = gpiod
    //     .pd11
    //     .into_push_pull_output(&mut gpiod.crh)
    //     .set_speed(&mut gpiod.crh, IOPinSpeed::Mhz50);
    // let _rd = gpiod
    //     .pd4
    //     .into_push_pull_output(&mut gpiod.crl)
    //     .set_speed(&mut gpiod.crl, IOPinSpeed::Mhz50);
    // let _wr = gpiod
    //     .pd5
    //     .into_push_pull_output(&mut gpiod.crl)
    //     .set_speed(&mut gpiod.crl, IOPinSpeed::Mhz50);
    // let _cs = gpiod
    //     .pd7
    //     .into_push_pull_output(&mut gpiod.crl)
    //     .set_speed(&mut gpiod.crl, IOPinSpeed::Mhz50);

    // copied from CubeMX
    gpiod.crl.write(|w| unsafe { w.bits(0xB4BB44BB) });
    gpiod.crh.write(|w| unsafe { w.bits(0xBB44BBBB) });
    // gpiod.idr.write(|w| unsafe { w.bits(0x0000F7FF) });
    gpioe.crl.write(|w| unsafe { w.bits(0xB4444444) });
    gpioe.crh.write(|w| unsafe { w.bits(0xBBBBBBBB) });
    // gpioe.idr.write(|w| unsafe { w.bits(0x0000FFFC) });
}

pub struct FsmcInterface<'a> {
    hsram: SramHandleTypeDef<'a>,
    reg: *mut u16,
    ram: *mut u16,
}

impl<'a> FsmcInterface<'a> {
    pub fn new(hsram: SramHandleTypeDef<'a>, gpioe: GPIOE, gpiod: GPIOD) -> Self {
        const LCD_FSMC_NEX: u32 = 1;
        const LCD_FSMC_AX: u32 = 16;
        const LCD_BASE: u32 =
            (0x60000000u32 + (0x4000000u32 * (LCD_FSMC_NEX - 1))) | (((1 << LCD_FSMC_AX) * 2) - 2);
        let s = Self {
            hsram,
            reg: LCD_BASE as *mut u16,
            ram: (LCD_BASE + 2) as *mut u16,
        };
        hal_fsmc_msp_init(gpioe, gpiod);
        hal_sram_init(&s.hsram, &s.hsram.timing, &s.hsram.ext_timing);
        s
    }
}

type Result = core::result::Result<(), DisplayError>;

#[inline(always)]
fn small_delay<T>(ptr: *const T) {
    for _ in 0..1 {
        unsafe {
            core::ptr::read_volatile(ptr);
        }
    }
}

impl<'a> WriteOnlyDataCommand for FsmcInterface<'a> {
    #[inline(always)]
    fn send_commands(&mut self, cmds: DataFormat<'_>) -> Result {
        match cmds {
            DataFormat::U8Iter(iter) => {
                for cmd in iter {
                    unsafe {
                        *self.reg = cmd as u16;
                    }
                }
            }
            _ => return Err(DisplayError::InvalidFormatError),
        }
        small_delay(&self);
        Ok(())
    }

    #[inline(always)]
    fn send_data(&mut self, data: DataFormat<'_>) -> Result {
        match data {
            DataFormat::U8Iter(iter) => {
                for d in iter {
                    unsafe {
                        *self.ram = d as u16;
                    }
                }
            }
            DataFormat::U16BEIter(iter) => {
                // let buf = unsafe { core::slice::from_raw_parts_mut(self.ram, 1) };
                // for d in iter {
                //     // better performance...why?
                //     buf[0] = d;
                // }
                for d in iter {
                    unsafe {
                        *self.ram = d;
                    }
                }
            }
            _ => return Err(DisplayError::InvalidFormatError),
        }
        small_delay(&self);
        Ok(())
    }
}
