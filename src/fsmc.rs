use core::ops::Deref;

use display_interface::{DataFormat, WriteOnlyDataCommand};
use ili9341::DisplayError;
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
    let p: *mut u32 = device.deref().bcr1.as_ptr();
    // Disable NORSRAM Device
    unsafe {
        *p = *p & !0x1u32;
    }
    let mask = 0xFFF7Fu32;
    let btcr_reg = init.data_address_mux
        | init.burst_access_mode
        | init.wait_signal_polarity
        | init.wrap_mode
        | init.wait_signal_active
        | init.write_operation
        | init.wait_signal
        | init.extended_mode
        | init.asynchronous_wait
        | init.write_burst
        | init.page_size;
    unsafe {
        *p = (*p & mask) | btcr_reg;
    }
}

pub fn fsmc_norsram_timing_init(device: &FSMC, timing: &FsmcNorsramTimingTypeDef) {
    let p: *mut u32 = device.btr1.as_ptr();
    let mask = 0x3FFFFFFFu32;
    let btcr_reg = timing.address_setup_time
        | (timing.address_hold_time << 4)
        | (timing.data_setup_time << 8)
        | (timing.bus_turn_around_duration << 16)
        | ((timing.clk_division - 1) << 20)
        | ((timing.data_latency - 2) << 24)
        | (timing.access_mode);
    unsafe {
        *p = (*p & mask) | btcr_reg;
    }
}

pub fn fsmc_norsram_extended_timing_init(device: &FSMC, timing: &FsmcNorsramTimingTypeDef) {
    let p: *mut u32 = (device.deref().bcr1.as_ptr() as u32 + 0x00000104u32) as _;
    let mask = 0x3FFFFFFFu32;
    let btcr_reg = timing.address_setup_time
        | (timing.address_hold_time << 4)
        | (timing.data_setup_time << 8)
        | (timing.bus_turn_around_duration << 16)
        | (timing.access_mode);
    unsafe {
        *p = (*p & mask) | btcr_reg;
    }
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
    let p: *mut u32 = hsram.device.bcr1.as_ptr();
    unsafe {
        *p = *p | 0x1u32;
    }
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
    // TODO: implement this function
    panic!("hal_fsmc_msp_init not implemented");
    // let gpioe = &peripherals.GPIOE;
    // gpioe.odr
}

pub struct FsmcInterface<'a> {
    hsram: SramHandleTypeDef<'a>,
    // gpioe: GE,
    // gpiod: GD,
    reg: *mut u16,
    ram: *mut u16,
}

impl<'a> FsmcInterface<'a> {
    pub fn new(hsram: SramHandleTypeDef<'a>, gpioe: GPIOE, gpiod: GPIOD) -> Self {
        const LCD_FSMC_NEX: u32 = 1;
        const LCD_FSMC_AX: u32 = 16;
        const lcd_base: u32 =
            (0x60000000u32 + (0x4000000u32 * (LCD_FSMC_NEX - 1))) | (((1 << LCD_FSMC_AX) * 2) - 2);
        let mut s = Self {
            hsram,
            // gpioe,
            // gpiod,
            reg: lcd_base as *mut u16,
            ram: (lcd_base + 2) as *mut u16,
        };
        hal_fsmc_msp_init(gpioe, gpiod);
        hal_sram_init(&s.hsram, &s.hsram.timing, &s.hsram.ext_timing);
        s
    }
}

type Result = core::result::Result<(), DisplayError>;

impl<'a> WriteOnlyDataCommand for FsmcInterface<'a> {
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
        Ok(())
    }

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
                for d in iter {
                    unsafe {
                        *self.ram = d;
                    }
                }
            }
            _ => return Err(DisplayError::InvalidFormatError),
        }
        Ok(())
    }
}
