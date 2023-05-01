use core::convert::Infallible;
use core::mem::size_of;
use core::ptr::null;

use embedded_hal::adc::{self as adc, nb::OneShot, nb::Channel};
use embedded_hal::delay::blocking::{DelayUs, DelayMs};
use pac::Peripherals;
use crate::clock;
use crate::clock::Clocks;
use crate::delay;
use crate::delay::*;

use crate::{pac, gpio::Adc};

#[derive(Debug, PartialEq, Eq)]
pub enum AdcError {
    ErrFifoEmpty,
    ErrRead,
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum AdcChannel {
    Ch0, // Gpio 12
    Ch1, // Gpio 4
    Ch2, // Gpio 14
    Ch3, // Gpio 13
    Ch4, // Gpio 5
    Ch5, // Gpio 6
    Ch6, 
    Ch7, // Gpio 9
    Ch8, 
    Ch9, // Gpio 10
    Ch10, // Gpio 11
    Ch11, // Gpio 15
    ChDacOutA,
    ChDacOutB,
    ChTempSensorPos,
    ChTempSensorNeg,
    ChVref,
    ChDcTest,
    ChVBatHalf,
    ChSenP3,
    ChSenP2,
    ChSenP1,
    ChSenP0,
    ChGnd,
    None,
}
impl From<u8> for AdcChannel {
    fn from(val: u8) -> Self {
        match val {
            0 => AdcChannel::Ch0,
            1 => AdcChannel::Ch1,
            2 => AdcChannel::Ch2,
            3 => AdcChannel::Ch3,
            4 => AdcChannel::Ch4,
            5 => AdcChannel::Ch5,
            6 => AdcChannel::Ch6,
            7 => AdcChannel::Ch7,
            8 => AdcChannel::Ch8,
            9 => AdcChannel::Ch9,
            10 => AdcChannel::Ch10,
            11 => AdcChannel::Ch11,
            _ => panic!("Unhandled channel {}", val),
        }
    }
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AdcClockDiv {
    Div0, // 32M
    Div4, // 8M
    Div8, // 4M
    Div12, // 2.666M
    Div16, // 2M
    Div20, // 1.6M
    Div24, // 1.333M
    Div32, // 1M
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AdcClockType {
    Clk96M,
    ClkX
}
impl Into<bool> for AdcClockType {
    fn into(self) -> bool {
        match self {
            AdcClockType::Clk96M => false,
            AdcClockType::ClkX => true,
        }
    }
}
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AdcV18Sel {
    Sel1p62v,
    Sel1p72v,
    Sel1p82v,
    Sel1p92v,
}
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AdcV11Sel {
    Sel1p0v,
    Sel1p1v,
    Sel1p18v,
    Sel1p26v,
}
#[derive(Clone, Copy)]
pub enum AdcVRefType {
    VRef3p2v,
    VRef2v
}
impl Into<bool> for AdcVRefType {
    fn into(self) -> bool {
        match self {
            AdcVRefType::VRef3p2v => false,
            AdcVRefType::VRef2v => true,
        }
    }
}
#[derive(Clone, Copy)]
pub enum AdcInputMode {
    SingleEnd,
    Diff,
}
impl Into<bool> for AdcInputMode {
    fn into(self) -> bool {
        match self {
            AdcInputMode::SingleEnd => false,
            AdcInputMode::Diff => true,
        }
    }
}
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AdcDataWidth {
    Bits12,
    Bits14Avg16,
    Bits16Avg64,
    Bits16Avg128,
    Bits16Avg256,
}
#[repr(u8)]
#[derive(Eq, PartialEq, Clone, Copy)]
pub enum AdcPgaGain {
    PgaGainNone,
    PgaGain1,
    PgaGain2,
    PgaGain4,
    PgaGain8,
    PgaGain16,
    PgaGain32,
}
#[derive(Clone, Copy)]
pub enum AdcBiasBandgap {
    BiasSelMainBandgap,   // ADC current from main bandgap
    BiasSelAonBandgap,    // ADC current from aon bandgap for HBN mode
}
impl Into<bool> for AdcBiasBandgap {
    fn into(self) -> bool {
        match self {
            AdcBiasBandgap::BiasSelMainBandgap => false,
            AdcBiasBandgap::BiasSelAonBandgap => true,
        }
    }
}
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AdcPgaVcm {
    PgaVcm1V,
    PgaVcm1P2V,
    PgaVcm1P4V,
    PgaVcm1P6V,
}
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AdcFifoThreshold {
    FifoThreshold1,
    FifoThreshold4,
    FifoThreshold8,
    FifoThreshold16,
}
#[repr(u8)]
pub enum DmaTransferDir {
    MemoryToMemory,
    MemoryToPeripheral,
    PeripheralToMemory,
    PeripheralToPeripheral,
}

const EF_CTRL_DFT_TIMEOUT_VAL: u32 = 160 * 1000;
const ADC_CHANNEL_COUNT: usize = 12;
const ADC_DMA_BUF_SIZE: usize = 1000 * 2 * size_of::<u32>();

// ADC_CFG_Type
pub struct Adc1 {
    sys_clock: u32,
    v18_sel: AdcV18Sel,
    v11_sel: AdcV11Sel,
    clk_div: AdcClockDiv,
    gain1: AdcPgaGain,
    gain2: AdcPgaGain,
    vref: AdcVRefType,
    bias_sel: AdcBiasBandgap,
    vcm: AdcPgaVcm,
    input_mode: AdcInputMode,
    data_width: AdcDataWidth,
    offset_calibration: bool,
    offset_calibration_value: u16,
    pos_channels: [AdcChannel; ADC_CHANNEL_COUNT],
    neg_channels: [AdcChannel; ADC_CHANNEL_COUNT],
    scan_len: u8,
    fifo_enable_dma: bool,
    fifo_threshold: AdcFifoThreshold,
    gain_coeff_enable: bool,
    gain_coefficient: u16,
    gain_coe: f32,
    dma_ctx: Option<AdcContext>,
}

// adc_ctx_t
struct AdcContext {
    channel_data: u32,
    chan_init_table: u32,
    data_size: u32,
    data_buf: [u8; ADC_DMA_BUF_SIZE],
    lli0: DmaLliCtrl,
    lli1: DmaLliCtrl,
}

struct DmaLliCtrl {
    src_dma_addr: u32,
    dst_dma_addr: *const u8,
    next_lli: *const DmaLliCtrl,
}

impl adc::nb::Channel<Adc1> for crate::gpio::Pin4<Adc> {
    type ID = AdcChannel;
    fn channel(&self) -> Self::ID {
        AdcChannel::Ch1
    }
}
impl adc::nb::Channel<Adc1> for crate::gpio::Pin5<Adc> {
    type ID = AdcChannel;
    fn channel(&self) -> Self::ID {
        AdcChannel::Ch4
    }
}
impl adc::nb::Channel<Adc1> for crate::gpio::Pin6<Adc> {
    type ID = AdcChannel;
    fn channel(&self) -> Self::ID {
        AdcChannel::Ch5
    }
}
impl adc::nb::Channel<Adc1> for crate::gpio::Pin9<Adc> {
    type ID = AdcChannel;
    fn channel(&self) -> Self::ID {
        AdcChannel::Ch6 // 6/7??
    }
}
impl adc::nb::Channel<Adc1> for crate::gpio::Pin10<Adc> {
    type ID = AdcChannel;
    fn channel(&self) -> Self::ID {
        AdcChannel::Ch9 // CH8/9 ??
    }
}
impl adc::nb::Channel<Adc1> for crate::gpio::Pin11<Adc> {
    type ID = AdcChannel;
    fn channel(&self) -> Self::ID {
        AdcChannel::Ch10
    }
}
impl adc::nb::Channel<Adc1> for crate::gpio::Pin12<Adc> {
    type ID = AdcChannel;
    fn channel(&self) -> Self::ID {
        AdcChannel::Ch0
    }
}
impl adc::nb::Channel<Adc1> for crate::gpio::Pin13<Adc> {
    type ID = AdcChannel;
    fn channel(&self) -> Self::ID {
        AdcChannel::Ch3 // CH3
    }
}
impl adc::nb::Channel<Adc1> for crate::gpio::Pin14<Adc> {
    type ID = AdcChannel;
    fn channel(&self) -> Self::ID {
        AdcChannel::Ch2
    }
}
impl adc::nb::Channel<Adc1> for crate::gpio::Pin15<Adc> {
    type ID = AdcChannel;
    fn channel(&self) -> Self::ID {
        AdcChannel::Ch11 // CH11
    }
}

// 12 bit ADC
impl Adc1
{
    pub fn new(adc_channel: AdcChannel) -> Self 
    {
        let mut scan_len = 0;
        let mut neg_channels = [AdcChannel::None; ADC_CHANNEL_COUNT];
        let mut pos_channels = [AdcChannel::None; ADC_CHANNEL_COUNT];

        for i in 0..ADC_CHANNEL_COUNT {
            let ch_at_i = AdcChannel::from(i as u8);
            if adc_channel == ch_at_i {
                pos_channels[i] = ch_at_i;
                neg_channels[i] = AdcChannel::ChGnd;
                scan_len += 1;
            }
        }

        Self {
            sys_clock: 0, // set on init()
            clk_div: AdcClockDiv::Div24,
            v18_sel: AdcV18Sel::Sel1p82v,
            v11_sel: AdcV11Sel::Sel1p1v,
            gain1: AdcPgaGain::PgaGainNone,
            gain2: AdcPgaGain::PgaGainNone,
            vref: AdcVRefType::VRef3p2v,
            bias_sel: AdcBiasBandgap::BiasSelMainBandgap,
            vcm: AdcPgaVcm::PgaVcm1V,
            input_mode: AdcInputMode::SingleEnd,
            data_width: AdcDataWidth::Bits12,
            offset_calibration: false,
            offset_calibration_value: 0,
            fifo_enable_dma: false,
            fifo_threshold: AdcFifoThreshold::FifoThreshold1,
            gain_coeff_enable: false,
            gain_coefficient: 0,
            gain_coe: 0f32,
            pos_channels,
            neg_channels,
            scan_len,
            dma_ctx: None,
        }
    }

    pub fn init(&mut self, clocks: &clock::Clocks) {
        self.sys_clock = clocks.sysclk().0;

        adc_enable(false);
        adc_enable(true);
        soft_reset();

        // config1
        {
            let aon = unsafe { &*pac::AON::ptr() };
            aon.gpadc_reg_config1.modify(|_,w| unsafe {
                w.gpadc_v18_sel().bits(self.v18_sel as u8);
                w.gpadc_v11_sel().bits(self.v11_sel as u8);
                w.gpadc_dither_en().bit(false);
                w.gpadc_scan_en().bit(false);
                w.gpadc_scan_length().bits(0u8);
                w.gpadc_clk_div_ratio().bits(self.clk_div as u8);
                w.gpadc_clk_ana_inv().bit(false);
                w.gpadc_cal_os_en().bit(self.offset_calibration);
                w.gpadc_res_sel().bits(self.data_width as u8);
                w
            });
        }
        McycleDelay::delay_cycles(8);

        // config2
        {
            let aon = unsafe { &*pac::AON::ptr() };
            aon.gpadc_reg_config2.modify(|_, w| unsafe {
                w.gpadc_dly_sel().bits(0u8);
                w.gpadc_pga1_gain().bits(self.gain1 as u8);
                w.gpadc_pga2_gain().bits(self.gain2 as u8);
                w.gpadc_bias_sel().bit(self.bias_sel.into());

                // chopmode
                let has_gain = self.gain1 != AdcPgaGain::PgaGainNone || self.gain2 != AdcPgaGain::PgaGainNone;
                if has_gain {
                    w.gpadc_chop_mode().bits(2u8);
                } else {
                    w.gpadc_chop_mode().bits(1u8);
                }
                
                // pga_vcmi = mic
                w.gpadc_pga_vcmi_en().bit(false);
                w.gpadc_pga_en().bit(has_gain);

                // pga_os_cal = mic
                w.gpadc_pga_os_cal().bits(8u8);
                w.gpadc_pga_vcm().bits(self.vcm as u8);
                w.gpadc_vref_sel().bit(self.vref.into());
                w.gpadc_diff_mode().bit(self.input_mode.into());

                w
            });

            // calibration offset
            aon.gpadc_reg_define.modify(|_,w| unsafe {
                w.gpadc_os_cal_data().bits(self.offset_calibration_value)
            });
        }

        self.set_gain_trim();

        self.dma_init(1000);

        self.config_channels(true);

        self.set_fifo_config(self.fifo_threshold, true);
    }

    pub fn deinit(&mut self) {
        self.stop();
    }

    // ADC_Scan_Channel_Config
    fn config_channels(&self, enable_continuous_dma: bool) {
        let scan_len = self.scan_len as usize;

        // mode != 0
        let deal_len = if scan_len < 6 {
            scan_len
        } else {
            6
        };

        // set first 6 ch 
        let peripherals = get_peripherals();
        peripherals.AON.gpadc_reg_scn_pos1.modify(|r,w| unsafe {
            let mut reg = r.bits();
            for i in 0..deal_len {
                reg = reg & (!(0x1F << (i * 5)));
                // TODO: Multi channel
                reg |= ((self.pos_channels[i] as u8) << (i * 5)) as u32;
            }
            w.bits(reg)
        });
        peripherals.AON.gpadc_reg_scn_neg1.modify(|r,w| unsafe {
            let mut reg = r.bits();
            for i in 0..deal_len {
                reg = reg & (!(0x1F << (i * 5)));
                // TODO: Multi channel
                reg |= ((self.neg_channels[i] as u8) << (i * 5)) as u32;
            }
            w.bits(reg)
        });

        // remaining channels
        if scan_len > deal_len {
            peripherals.AON.gpadc_reg_scn_pos2.modify(|r,w| {
                let mut reg = r.bits();
                for i in 0..scan_len - deal_len {
                    reg = reg & (!(0x1F << (i * 5)));
                    reg |= ((self.pos_channels[i] as u8) << (i * 5)) as u32;
                }  
                w
            });
            peripherals.AON.gpadc_reg_scn_neg2.modify(|r,w| {
                let mut reg = r.bits();
                for i in 0..scan_len - deal_len {
                    reg = reg & (!(0x1F << (i * 5)));
                    reg |= ((self.neg_channels[i] as u8) << (i * 5)) as u32;
                }  
                w
            });
        }

        // scan mode
        peripherals.AON.gpadc_reg_config1.modify(|_,w| unsafe {
            w.gpadc_scan_length().bits(scan_len as u8 - 1u8)
             .gpadc_cont_conv_en().bit(enable_continuous_dma)
             .gpadc_scan_en().bit(true)
        });
    }

    // ADC_FIFO_Cfg
    fn set_fifo_config(&mut self, fifo_thresh: AdcFifoThreshold, enable_dma: bool) {
        let peripherals = get_peripherals();

        // enable = Fifo data is exceeded fifoThreshold DMA request occurs
        // disable = Threshold determines how much data to cause FIFO ready interrupt
        peripherals.GPIP.gpadc_config.modify(|_,w| unsafe {
            w.gpadc_fifo_thl().bits(fifo_thresh as u8)
            .gpadc_dma_en().bit(enable_dma)
        });
        // clear fifo
        peripherals.GPIP.gpadc_config.modify(|_,w| {
            w.gpadc_fifo_clr().clear_bit()
        });

        self.fifo_threshold = fifo_thresh;
        self.fifo_enable_dma = enable_dma;
    }

    // ADC_Gain_Trim
    fn set_gain_trim(&mut self) { 
        fn get_trim_parity(val: u32, len: u8) -> u8 {
            let mut c: u8 = 0;

            for i in 0..len {
                if val & (1 << i) != 0 {
                    c += 1;
                }
            }

            c & 0x01
        }
        fn is_ctrl_busy(peripherals: &Peripherals) -> bool {
            peripherals.EF_CTRL.ef_if_ctrl_0.read()
              .ef_if_0_busy().bit_is_set()
        }
        fn use_ahb_clk_0(peripherals: &Peripherals) {
            let mut timeout: u32 = EF_CTRL_DFT_TIMEOUT_VAL;

            while is_ctrl_busy(&peripherals) {
                timeout -= 1;
                if timeout == 0 {
                    break;
                }
            }

            peripherals.EF_CTRL.ef_if_ctrl_0.write(|w| {
                // auto = false, manual = true
                w.ef_if_0_manual_en().bit(false)
                // default = false, manual = true
                .ef_if_cyc_modify_lock().bit(false)
                // efuse clock = false, SAHB clock = true
                .ef_clk_sahb_data_sel().bit(true)
                .ef_if_auto_rd_en().bit(true)
                .ef_if_por_dig().bit(false)
                .ef_if_0_int_clr().bit(true)
                .ef_if_0_rw().bit(false)
                .ef_if_0_trig().bit(false)
            });
        }

        let peripherals = get_peripherals();
        use_ahb_clk_0(&peripherals);

        let gain_trim_reg = peripherals.EF_DATA_0.ef_key_slot_5_w3.read().bits();
        let adc_gain_coeff = (gain_trim_reg >> 1) & 0xfff;
        let adc_gain_coeff_parity = (gain_trim_reg >> 13) & 0x01;
        let adc_gain_coeff_en = (gain_trim_reg >> 14) & 0x01 != 0;

        if adc_gain_coeff_en {
            if adc_gain_coeff as u8 == get_trim_parity(adc_gain_coeff_parity, 12) {
                self.gain_coeff_enable = true; 
                self.gain_coefficient = adc_gain_coeff as u16;

                let mut tmp = self.gain_coefficient;
                if tmp & 0x800 != 0 {
                    tmp = !tmp;
                    tmp += 1;
                    tmp = tmp & 0xfff;

                    self.gain_coe = 1.0 + (tmp as f32 / 2048.0);
                } else {
                    self.gain_coe = 1.0 - (tmp as f32 / 2048.0);
                }
            }
        }
    }

    pub fn start(&self) {
        if self.fifo_enable_dma {
            let dma = get_peripherals().DMA;

            // disable DMA_INT_ALL
            dma.dma_c1config.modify(|_,w| w.itc().set_bit().ie().set_bit());
            dma.dma_c1control.modify(|_,w| w.i().clear_bit());

            // enable DMA_INT_TCOMPLETED
            dma.dma_c1config.modify(|_,w| w.itc().clear_bit());
            dma.dma_c1control.modify(|_,w| w.i().set_bit());

            // enable DMA_INT_ERR
            dma.dma_c1config.modify(|_,w| w.ie().set_bit());
        }
        {
            let aon = get_peripherals().AON;
            aon.gpadc_reg_cmd.modify(|_,w| w.gpadc_conv_start().clear_bit());
            dummy_wait();
            aon.gpadc_reg_cmd.modify(|_,w| w.gpadc_conv_start().set_bit());
        }

        if self.fifo_enable_dma {
            // Enable DMA
            let dma = get_peripherals().DMA;
            dma.dma_top_config.modify(|_,w| w.e().set_bit());
            // DMA channel 1 = ADC_CHANNEL
            dma.dma_c1config.modify(|_,w| w.e().set_bit());
        }
    }

    pub fn stop(&self) {
        get_peripherals().AON.gpadc_reg_cmd.modify(|_,w| w.gpadc_conv_start().set_bit());

        if self.fifo_enable_dma {
            // DMA channel 1 = ADC_CHANNEL
            get_peripherals().DMA.dma_top_config.modify(|_,w| w.e().clear_bit());
        }
    }

    pub fn fifo_count(&self) -> u8 {
        get_peripherals().GPIP.gpadc_config.read().gpadc_fifo_data_count().bits()
    }

    pub fn fifo_full(&self) -> bool {
        get_peripherals().GPIP.gpadc_config.read().gpadc_fifo_full().bit()
    }

    pub fn fifo_empty(&self) -> bool {
        get_peripherals().GPIP.gpadc_config.read().gpadc_fifo_ne().bit() == false
    }

    #[allow(unused)]
    pub fn dma_init(&mut self, sample_len: u32) {
        let mut ctx = AdcContext {
            channel_data: 0,
            chan_init_table: 0,
            data_size: sample_len,
            data_buf: [0u8; ADC_DMA_BUF_SIZE],
            lli0: DmaLliCtrl {
                src_dma_addr: 0x40002000+0x4,
                dst_dma_addr: null(),
                next_lli: null(),
            },
            lli1: DmaLliCtrl {
                src_dma_addr: 0x40002000+0x4,
                dst_dma_addr: null(),
                next_lli: null(),
            },
        };
        ctx.lli0.next_lli = &ctx.lli1;
        ctx.lli1.next_lli = &ctx.lli0;
        ctx.lli0.dst_dma_addr = core::ptr::addr_of!(ctx.data_buf[0]);
        ctx.lli1.dst_dma_addr = core::ptr::addr_of!(ctx.data_buf[ADC_CHANNEL_COUNT]);

        // ADC_DMA_CHANNEL = 1
        dma_channel_disable(1);

        {
            let dma = get_peripherals().DMA;
            dma.dma_c1config.modify(|_,w| unsafe {
                w.flow_cntrl().bits(DmaTransferDir::PeripheralToMemory as u8)
                 .dst_peripheral().bits(0u8)  // 0 = DMA_REQ_NONE
                 .src_peripheral().bits(22u8) // 22 = DMA_REQ_GPADC0
            });
            dma.dma_c1control.modify(|_,w| unsafe {
                w.transfer_size().bits(sample_len as u16)
                // 1 = DMA_BURST_SIZE_1, 2 = 4, 3 = 8, 4 = 16
                .sbsize().bits(1) 
                .dbsize().bits(1)
                .swidth().bits(2u8) // 0 = 8bit, 1 = 16, 32 = 2
                .dwidth().bits(2u8)
                .si().clear_bit()
                .di().set_bit()
                .i().set_bit()
                .prot().bits(0u8)
            });
        }
    }
}

impl<WORD, PIN> OneShot<Adc1, WORD, PIN> for Adc1
where WORD: From<u16>,
      PIN: Channel<Adc1, ID=AdcChannel>,
{
    type Error = AdcError;

    fn read(&mut self, pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        let channel = pin.channel();

        self.start();
        while self.fifo_count() == 0 {
            return Err(nb::Error::Other(AdcError::ErrFifoEmpty));
        }

        let adc_val = get_peripherals().GPIP.gpadc_dma_rdata.read().bits() as u16;
        if adc_val == 0 {
            return Err(nb::Error::Other(AdcError::ErrRead));
        }

        return Ok(adc_val.into())
    }
}

#[inline]
fn get_peripherals() -> Peripherals {
    // TODO: take is locking for some reason, resort to more nefarious programming
    unsafe { pac::Peripherals::steal() }
}


// DMA_Channel_Disable
fn dma_channel_disable(channel: u8)
{
    match channel {
        0 => get_peripherals().DMA.dma_c0config.modify(|_,w| w.e().clear_bit()),
        1 => get_peripherals().DMA.dma_c1config.modify(|_,w| w.e().clear_bit()),
        2 => get_peripherals().DMA.dma_c2config.modify(|_,w| w.e().clear_bit()),
        3 => get_peripherals().DMA.dma_c3config.modify(|_,w| w.e().clear_bit()),
        _ => panic!(),
    }
}

fn soft_reset()
{
    get_peripherals().AON.gpadc_reg_cmd.modify(|_,w| { w.gpadc_soft_rst().bit(true)});    
    dummy_wait();
    get_peripherals().AON.gpadc_reg_cmd.modify(|_,w| { w.gpadc_soft_rst().bit(false)});
}

fn adc_enable(enable: bool)
{
    get_peripherals().AON.gpadc_reg_cmd.modify(|_,w| { w.gpadc_global_en().bit(enable) });
}

#[inline]
fn dummy_wait() {
    unsafe {
        riscv::asm::nop();
        riscv::asm::nop();
        riscv::asm::nop();
        riscv::asm::nop();
        riscv::asm::nop();
        riscv::asm::nop();
        riscv::asm::nop();
        riscv::asm::nop();
    }
}
