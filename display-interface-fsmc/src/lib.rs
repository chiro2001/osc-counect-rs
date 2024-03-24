#![no_std]

use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embassy_stm32::{
    dma::TransferOptions,
    pac::{
        self,
        bdma::vals::{Dir, Pl, Size},
    },
    peripherals::DMA2_CH4,
};

#[doc = r"FSMC Register block"]
#[repr(C)]
pub struct FSMC {
    #[doc = "0x00 - SRAM/NOR-Flash chip-select control register 1"]
    pub bcr1: u32,
    #[doc = "0x04 - SRAM/NOR-Flash chip-select timing register 1"]
    pub btr1: u32,
    #[doc = "0x08 - SRAM/NOR-Flash chip-select control register 2"]
    pub bcr2: u32,
    #[doc = "0x0c - SRAM/NOR-Flash chip-select timing register 1"]
    pub btr2: u32,
    #[doc = "0x10 - SRAM/NOR-Flash chip-select control register 2"]
    pub bcr3: u32,
    #[doc = "0x14 - SRAM/NOR-Flash chip-select timing register 1"]
    pub btr3: u32,
    #[doc = "0x18 - SRAM/NOR-Flash chip-select control register 2"]
    pub bcr4: u32,
    #[doc = "0x1c - SRAM/NOR-Flash chip-select timing register 1"]
    pub btr4: u32,
    _reserved8: [u8; 0x40],
    #[doc = "0x60 - PC Card/NAND Flash control register 2"]
    pub pcr2: u32,
    #[doc = "0x64 - FIFO status and interrupt register 2"]
    pub sr2: u32,
    #[doc = "0x68 - Common memory space timing register 2"]
    pub pmem2: u32,
    #[doc = "0x6c - Attribute memory space timing register 2"]
    pub patt2: u32,
    _reserved12: [u8; 0x04],
    #[doc = "0x74 - ECC result register 2"]
    pub eccr2: u32,
    _reserved13: [u8; 0x08],
    #[doc = "0x80 - PC Card/NAND Flash control register 3"]
    pub pcr3: u32,
    #[doc = "0x84 - FIFO status and interrupt register 3"]
    pub sr3: u32,
    #[doc = "0x88 - Common memory space timing register 3"]
    pub pmem3: u32,
    #[doc = "0x8c - Attribute memory space timing register 3"]
    pub patt3: u32,
    _reserved17: [u8; 0x04],
    #[doc = "0x94 - ECC result register 3"]
    pub eccr3: u32,
    _reserved18: [u8; 0x08],
    #[doc = "0xa0 - PC Card/NAND Flash control register 4"]
    pub pcr4: u32,
    #[doc = "0xa4 - FIFO status and interrupt register 4"]
    pub sr4: u32,
    #[doc = "0xa8 - Common memory space timing register 4"]
    pub pmem4: u32,
    #[doc = "0xac - Attribute memory space timing register 4"]
    pub patt4: u32,
    #[doc = "0xb0 - I/O space timing register 4"]
    pub pio4: u32,
    _reserved23: [u8; 0x50],
    #[doc = "0x104 - SRAM/NOR-Flash write timing registers 1"]
    pub bwtr1: u32,
    _reserved24: [u8; 0x04],
    #[doc = "0x10c - SRAM/NOR-Flash write timing registers 1"]
    pub bwtr2: u32,
    _reserved25: [u8; 0x04],
    #[doc = "0x114 - SRAM/NOR-Flash write timing registers 1"]
    pub bwtr3: u32,
    _reserved26: [u8; 0x04],
    #[doc = "0x11c - SRAM/NOR-Flash write timing registers 1"]
    pub bwtr4: u32,
}

const _FSMC_SZ: [u8; 0x11c + 4] = [0; core::mem::size_of::<FSMC>()];

const FSMC_DEV_ADDR: usize = 0xa000_0000u32 as usize;

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

pub struct SramHandleTypeDef {
    pub device: &'static mut FSMC,
    pub init: FsmcNorsramInitTypeDef,
    pub timing: FsmcNorsramTimingTypeDef,
    pub ext_timing: FsmcNorsramTimingTypeDef,
}

impl SramHandleTypeDef {
    pub fn new(
        init: FsmcNorsramInitTypeDef,
        timing: FsmcNorsramTimingTypeDef,
        ext_timing: FsmcNorsramTimingTypeDef,
    ) -> Self {
        Self {
            device: unsafe { &mut *(FSMC_DEV_ADDR as *mut FSMC) },
            init,
            timing,
            ext_timing,
        }
    }
}

pub fn fsmc_norsram_init(device: &mut FSMC, init: &FsmcNorsramInitTypeDef) {
    let p_bcr1 = &mut device.bcr1;
    let bcr1 = *p_bcr1;
    *p_bcr1 = bcr1 & !0x1u32;
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
    *p_bcr1 = (bcr1 & mask) | btcr_reg;
}

pub fn fsmc_norsram_timing_init(device: &mut FSMC, _timing: &FsmcNorsramTimingTypeDef) {
    // device.deref().btr1.write(|w| unsafe {
    //     w.addset().bits(timing.address_setup_time as u8);
    //     w.addhld().bits(timing.address_hold_time as u8);
    //     w.datast().bits(timing.data_setup_time as u8);
    //     w.busturn().bits(timing.bus_turn_around_duration as u8);
    //     w.clkdiv().bits((timing.clk_division - 1) as u8);
    //     w.datlat().bits((timing.data_latency - 2) as u8);
    //     w.accmod().bits(timing.access_mode as u8)
    //     // w.bits(0x0ff00ff0u32)
    // });
    device.btr1 = 0x0ff00ff0u32;
}

pub fn fsmc_norsram_extended_timing_init(device: &mut FSMC, _timing: &FsmcNorsramTimingTypeDef) {
    // device.deref().bwtr1.write(|w| unsafe {
    //     w.addset().bits(timing.address_setup_time as u8);
    //     w.addhld().bits(timing.address_hold_time as u8);
    //     w.datast().bits(timing.data_setup_time as u8);
    //     w.busturn().bits(timing.bus_turn_around_duration as u8);
    //     w.accmod().bits(timing.access_mode as u8)
    //     // w.bits(0x0ff001f0u32)
    // });
    device.bwtr1 = 0x0ff001f0u32;
}

pub fn hal_sram_init(hsram: &mut SramHandleTypeDef) {
    // fsmc_norsram_init(hsram.device, &hsram.init);
    // fsmc_norsram_timing_init(hsram.device, &hsram.timing);
    // fsmc_norsram_extended_timing_init(hsram.device, &hsram.ext_timing);

    /*
        Peripheral, Register, Value, Fields
        FSMC, BCR1, 0x0000705B, MBKEN: 0b1; MUXEN: 0b1; MTYP: 0b10; MWID: 0b01; FACCEN: 0b1; BURSTEN: 0b0; WAITPOL: 0b0; WAITCFG: 0b0; WREN: 0b1; WAITEN: 0b1; EXTMOD: 0b1; ASYNCWAIT: 0b0; CBURSTRW: 0b0
        FSMC, BTR1, 0x00000110, ADDSET: 0b0000; ADDHLD: 0b0001; DATAST: 0b00000001; BUSTURN: 0b0000; CLKDIV: 0b0000; DATLAT: 0b0000; ACCMOD: 0b00
        FSMC, BCR2, 0x000030D2, MBKEN: 0b0; MUXEN: 0b1; MTYP: 0b00; MWID: 0b01; FACCEN: 0b1; BURSTEN: 0b0; WAITPOL: 0b0; WRAPMOD: 0b0; WAITCFG: 0b0; WREN: 0b1; WAITEN: 0b1; EXTMOD: 0b0; ASYNCWAIT: 0b0; CBURSTRW: 0b0
        FSMC, BTR2, 0x0FFFFFFF, ADDSET: 0b1111; ADDHLD: 0b1111; DATAST: 0b11111111; BUSTURN: 0b1111; CLKDIV: 0b1111; DATLAT: 0b1111; ACCMOD: 0b00
        FSMC, BCR3, 0x000030D2, MBKEN: 0b0; MUXEN: 0b1; MTYP: 0b00; MWID: 0b01; FACCEN: 0b1; BURSTEN: 0b0; WAITPOL: 0b0; WRAPMOD: 0b0; WAITCFG: 0b0; WREN: 0b1; WAITEN: 0b1; EXTMOD: 0b0; ASYNCWAIT: 0b0; CBURSTRW: 0b0
        FSMC, BTR3, 0x0FFFFFFF, ADDSET: 0b1111; ADDHLD: 0b1111; DATAST: 0b11111111; BUSTURN: 0b1111; CLKDIV: 0b1111; DATLAT: 0b1111; ACCMOD: 0b00
        FSMC, BCR4, 0x000030D2, MBKEN: 0b0; MUXEN: 0b1; MTYP: 0b00; MWID: 0b01; FACCEN: 0b1; BURSTEN: 0b0; WAITPOL: 0b0; WRAPMOD: 0b0; WAITCFG: 0b0; WREN: 0b1; WAITEN: 0b1; EXTMOD: 0b0; ASYNCWAIT: 0b0; CBURSTRW: 0b0
        FSMC, BTR4, 0x0FFFFFFF, ADDSET: 0b1111; ADDHLD: 0b1111; DATAST: 0b11111111; BUSTURN: 0b1111; CLKDIV: 0b1111; DATLAT: 0b1111; ACCMOD: 0b00
        FSMC, PCR2, 0x00000018, PWAITEN: 0b0; PBKEN: 0b0; PTYP: 0b1; PWID: 0b01; ECCEN: 0b0; TCLR: 0b0000; TAR: 0b0000; ECCPS: 0b000
        FSMC, SR2, 0x00000040, IRS: 0b0; ILS: 0b0; IFS: 0b0; IREN: 0b0; ILEN: 0b0; IFEN: 0b0; FEMPT: 0b1
        FSMC, PMEM2, 0xFCFCFCFC, MEMSETx: 0b11111100; MEMWAITx: 0b11111100; MEMHOLDx: 0b11111100; MEMHIZx: 0b11111100
        FSMC, PATT2, 0xFCFCFCFC, ATTSETx: 0b11111100; ATTWAITx: 0b11111100; ATTHOLDx: 0b11111100; ATTHIZx: 0b11111100
        FSMC, ECCR2, 0x00000000, ECCx: 0b00000000000000000000000000000000
        FSMC, PCR3, 0x00000018, PWAITEN: 0b0; PBKEN: 0b0; PTYP: 0b1; PWID: 0b01; ECCEN: 0b0; TCLR: 0b0000; TAR: 0b0000; ECCPS: 0b000
        FSMC, SR3, 0x00000040, IRS: 0b0; ILS: 0b0; IFS: 0b0; IREN: 0b0; ILEN: 0b0; IFEN: 0b0; FEMPT: 0b1
        FSMC, PMEM3, 0xFCFCFCFC, MEMSETx: 0b11111100; MEMWAITx: 0b11111100; MEMHOLDx: 0b11111100; MEMHIZx: 0b11111100
        FSMC, PATT3, 0xFCFCFCFC, ATTSETx: 0b11111100; ATTWAITx: 0b11111100; ATTHOLDx: 0b11111100; ATTHIZx: 0b11111100
        FSMC, ECCR3, 0x00000000, ECCx: 0b00000000000000000000000000000000
        FSMC, PCR4, 0x00000018, PWAITEN: 0b0; PBKEN: 0b0; PTYP: 0b1; PWID: 0b01; ECCEN: 0b0; TCLR: 0b0000; TAR: 0b0000; ECCPS: 0b000
        FSMC, SR4, 0x00000040, IRS: 0b0; ILS: 0b0; IFS: 0b0; IREN: 0b0; ILEN: 0b0; IFEN: 0b0; FEMPT: 0b1
        FSMC, PMEM4, 0xFCFCFCFC, MEMSETx: 0b11111100; MEMWAITx: 0b11111100; MEMHOLDx: 0b11111100; MEMHIZx: 0b11111100
        FSMC, PATT4, 0xFCFCFCFC, ATTSETx: 0b11111100; ATTWAITx: 0b11111100; ATTHOLDx: 0b11111100; ATTHIZx: 0b11111100
        FSMC, PIO4, 0xFCFCFCFC, IOSETx: 0b11111100; IOWAITx: 0b11111100; IOHOLDx: 0b11111100; IOHIZx: 0b11111100
        FSMC, BWTR1, 0x0FF00110, ADDSET: 0b0000; ADDHLD: 0b0001; DATAST: 0b00000001; CLKDIV: 0b1111; DATLAT: 0b1111; ACCMOD: 0b00
        FSMC, BWTR2, 0x0FFFFFFF, ADDSET: 0b1111; ADDHLD: 0b1111; DATAST: 0b11111111; CLKDIV: 0b1111; DATLAT: 0b1111; ACCMOD: 0b00
        FSMC, BWTR3, 0x0FFFFFFF, ADDSET: 0b1111; ADDHLD: 0b1111; DATAST: 0b11111111; CLKDIV: 0b1111; DATLAT: 0b1111; ACCMOD: 0b00
        FSMC, BWTR4, 0x0FFFFFFF, ADDSET: 0b1111; ADDHLD: 0b1111; DATAST: 0b11111111; CLKDIV: 0b1111; DATLAT: 0b1111; ACCMOD: 0b00
    */
    let d = &mut hsram.device;
    d.bcr1 = 0x0000705B & !0x1u32;
    d.btr1 = 0x00000110;
    d.bcr2 = 0x000030d2;
    d.btr2 = 0x0fffffff;
    d.bcr3 = 0x000030d2;
    d.btr3 = 0x0fffffff;
    d.bcr4 = 0x000030d2;
    d.btr4 = 0x0fffffff;
    d.pcr2 = 0x00000018;
    d.sr2 = 0x00000040;
    d.pmem2 = 0xfcfcfcfc;
    d.patt2 = 0xfcfcfcfc;
    d.eccr2 = 0x00000000;
    d.pcr3 = 0x00000018;
    d.sr3 = 0x00000040;
    d.pmem3 = 0xfcfcfcfc;
    d.patt3 = 0xfcfcfcfc;
    d.eccr3 = 0x00000000;
    d.pcr4 = 0x00000018;
    d.sr4 = 0x00000040;
    d.pmem4 = 0xfcfcfcfc;
    d.patt4 = 0xfcfcfcfc;
    d.pio4 = 0xfcfcfcfc;
    d.bwtr1 = 0x0ff00110;
    d.bwtr2 = 0x0fffffff;
    d.bwtr3 = 0x0fffffff;
    d.bwtr4 = 0x0fffffff;

    // enable device
    d.bcr1 = d.bcr1 | 1u32;
}

pub fn hal_fsmc_msp_init() {
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
    // // copied from CubeMX
    // gpiod.crl.write(|w| unsafe { w.bits(0xB4BB44BB) });
    // gpiod.crh.write(|w| unsafe { w.bits(0xBB44BBBB) });
    // // gpiod.idr.write(|w| unsafe { w.bits(0x0000F7FF) });
    // gpioe.crl.write(|w| unsafe { w.bits(0xB4444444) });
    // gpioe.crh.write(|w| unsafe { w.bits(0xBBBBBBBB) });
    // // gpioe.idr.write(|w| unsafe { w.bits(0x0000FFFC) });
    // FIXME: Init GPIO outside plz.
}

pub struct FsmcInterface {
    hsram: SramHandleTypeDef,
    reg: *mut u16,
    ram: *mut u16,
}

impl FsmcInterface {
    pub fn new(hsram: SramHandleTypeDef) -> Self {
        const LCD_FSMC_NEX: u32 = 1;
        const LCD_FSMC_AX: u32 = 16;
        let lcd_base: u32 =
            (0x60000000u32 + (0x4000000u32 * (LCD_FSMC_NEX - 1))) | (((1 << LCD_FSMC_AX) * 2) - 2);
        let mut s = Self {
            hsram,
            reg: lcd_base as *mut u16,
            ram: (lcd_base + 2) as *mut u16,
        };
        hal_fsmc_msp_init();
        hal_sram_init(&mut s.hsram);
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

impl WriteOnlyDataCommand for FsmcInterface {
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

    #[inline(never)]
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
                let buf = unsafe { core::slice::from_raw_parts_mut(self.ram, 1) };
                for d in iter {
                    // better performance...why?
                    buf[0] = d;
                }
            }
            DataFormat::U16BE(value) => {
                // send with async dma
                // pac::DMA2.ch(4).mar().write_value(value.as_ptr() as u32);
                // pac::DMA2.ch(4).par().write_value(self.ram as u32);
                // pac::DMA2.ch(4).ndtr().write_value(value.len() as u32);
                // pac::DMA2.ch(4).cr().modify(|w| {
                //     w.set_mem2mem(true);
                //     w.set_dir(Dir::FROMMEMORY);
                //     w.set_minc(true);
                //     w.set_pinc(false);
                //     w.set_pl(Pl::HIGH);
                //     w.set_msize(Size::BITS16);
                //     w.set_psize(Size::BITS16);
                //     w.set_circ(false);
                //     w.set_en(true);
                // });

                // let transfer = unsafe {
                //     embassy_stm32::dma::Transfer::new_write(
                //         DMA2_CH4::steal(),
                //         (),
                //         value,
                //         self.ram,
                //         TransferOptions::default(),
                //     )
                // };
                // transfer.blocking_wait();

                let buf = unsafe { core::slice::from_raw_parts_mut(self.ram, 1) };
                for d in value.into_iter() {
                    buf[0] = *d;
                }
            }
            _ => return Err(DisplayError::InvalidFormatError),
        }
        small_delay(&self);
        Ok(())
    }
}
