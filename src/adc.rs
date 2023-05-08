
use embedded_hal::adc::{self as adc, nb::OneShot, nb::Channel};
use pac::Peripherals;
use crate::clock;
use crate::delay::*;
use crate::dma;
use crate::dma::single_buffer;
use crate::dma::single_channel::SingleChannel;
use crate::gpio::AdcPin;
use crate::pac;

#[derive(Debug, PartialEq, Eq)]
pub enum AdcError {
    ErrFifoEmpty,
    ErrRead,
    ErrInt,
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

const EF_CTRL_DFT_TIMEOUT_VAL: u32 = 160 * 1000;
const ADC_CHANNEL_COUNT: usize = 12;

// ADC_CFG_Type
pub struct Config {
    pub v18_sel: AdcV18Sel,
    pub v11_sel: AdcV11Sel,
    pub clk_div: AdcClockDiv,
    pub gain1: AdcPgaGain,
    pub gain2: AdcPgaGain,
    pub vref: AdcVRefType,
    pub bias_sel: AdcBiasBandgap,
    pub vcm: AdcPgaVcm,
    pub input_mode: AdcInputMode,
    pub data_width: AdcDataWidth,
    pub offset_calibration: bool,
    pub offset_calibration_value: u16,
    pos_channels: [AdcChannel; ADC_CHANNEL_COUNT],
    neg_channels: [AdcChannel; ADC_CHANNEL_COUNT],
    scan_len: u8,
    pub fifo_enable_dma: bool,
    pub fifo_threshold: AdcFifoThreshold,
}

pub struct Adc {
    gpip: pac::GPIP,
    dma_ch: dma::Channel<dma::CH1>,
    pub buf: [u16; 64],
}

macro_rules! impl_adc_pin {
    ($Pini: ident, $ch: ident) => {
        impl adc::nb::Channel<Adc> for crate::gpio::$Pini<crate::gpio::Adc> {
            type ID = AdcChannel;
            fn channel(&self) -> Self::ID {
                AdcChannel::$ch
            }
        }
    };
}

impl_adc_pin!(Pin4, Ch1);
impl_adc_pin!(Pin5, Ch4);
impl_adc_pin!(Pin6, Ch5);
impl_adc_pin!(Pin9, Ch6);
impl_adc_pin!(Pin10, Ch9);
impl_adc_pin!(Pin11, Ch10);
impl_adc_pin!(Pin12, Ch0);
impl_adc_pin!(Pin13, Ch3);
impl_adc_pin!(Pin14, Ch2);
impl_adc_pin!(Pin15, Ch11);

impl Default for Config {
    fn default() -> Self 
    {
        Self {
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
            pos_channels: [AdcChannel::None; ADC_CHANNEL_COUNT],
            neg_channels: [AdcChannel::None; ADC_CHANNEL_COUNT],
            scan_len: 0,
        }
    }
}

// 12 bit ADC
impl Config {
    pub fn use_channel(mut self, channel: AdcChannel) -> Self 
    {
        if self.scan_len + 1 > ADC_CHANNEL_COUNT as u8 {
            panic!("Only 12 channels available")
        }
        self.pos_channels[self.scan_len as usize] = channel;
        self.neg_channels[self.scan_len as usize] = AdcChannel::ChGnd;
        self.scan_len += 1;
        self
    }
}

impl Adc {
    pub fn new(aon: pac::AON, gpip: pac::GPIP, dma_ch: dma::Channel<dma::CH1>, config: Config, clocks: &clock::Clocks) -> Self {
        adc_enable(false);
        adc_enable(true);
        soft_reset();

        // config1
        {
            let dp = unsafe { Peripherals::steal() };
            dp.AON.gpadc_reg_config1.modify(|_,w| unsafe {
                w.gpadc_v18_sel().bits(config.v18_sel as u8);
                w.gpadc_v11_sel().bits(config.v11_sel as u8);
                w.gpadc_dither_en().bit(false);
                w.gpadc_scan_en().bit(false);
                w.gpadc_scan_length().bits(0u8);
                w.gpadc_clk_div_ratio().bits(config.clk_div as u8);
                w.gpadc_clk_ana_inv().bit(false);
                w.gpadc_cal_os_en().bit(config.offset_calibration);
                w.gpadc_res_sel().bits(config.data_width as u8);
                w
            });
        }
        McycleDelay::delay_cycles(8);

        // config2
        Self::set_gain(&config);

        let mut coef_enable = false;
        let mut coef: u16 = 0;
        let mut coe: f32 = 0f32;
        Self::set_gain_trim(&mut coef_enable, &mut coef, &mut coe);

        Self::config_channels(&aon, &config, false);

        // ADC_FIFO_Cfg
        // enable = Fifo data is exceeded fifoThreshold DMA request occurs
        // disable = Threshold determines how much data to cause FIFO ready interrupt
        gpip.gpadc_config.modify(|_,w| unsafe {
            w.gpadc_fifo_thl().bits(config.fifo_threshold as u8)
            .gpadc_dma_en().bit(config.fifo_enable_dma)
        });
        // clear fifo
        gpip.gpadc_config.modify(|_,w| {
            w.gpadc_fifo_clr().clear_bit()
        });

        Self {
            gpip,
            dma_ch,
            buf: [0u16; 64],
        }
    }

    pub fn deinit(&mut self) {
        self.stop();
    }

    // ADC_Scan_Channel_Config
    fn config_channels(aon: &pac::AON, config: &Config, enable_continuous_dma: bool) {
        let scan_len = config.scan_len as usize;

        // mode != 0
        let deal_len = if scan_len < 6 {
            scan_len
        } else {
            6
        };

        // set first 6 ch 
        aon.gpadc_reg_scn_pos1.modify(|r,w| unsafe {
            let mut reg = r.bits();
            for i in 0..deal_len {
                reg = reg & (!(0x1F << (i * 5)));
                // TODO: Multi channel
                reg |= ((config.pos_channels[i] as u8) << (i * 5)) as u32;
            }
            w.bits(reg)
        });
        aon.gpadc_reg_scn_neg1.modify(|r,w| unsafe {
            let mut reg = r.bits();
            for i in 0..deal_len {
                reg = reg & (!(0x1F << (i * 5)));
                // TODO: Multi channel
                reg |= ((config.neg_channels[i] as u8) << (i * 5)) as u32;
            }
            w.bits(reg)
        });

        // remaining channels
        if scan_len > deal_len {
            aon.gpadc_reg_scn_pos2.modify(|r,w| {
                let mut reg = r.bits();
                for i in 0..scan_len - deal_len {
                    reg = reg & (!(0x1F << (i * 5)));
                    reg |= ((config.pos_channels[i] as u8) << (i * 5)) as u32;
                }  
                w
            });
            aon.gpadc_reg_scn_neg2.modify(|r,w| {
                let mut reg = r.bits();
                for i in 0..scan_len - deal_len {
                    reg = reg & (!(0x1F << (i * 5)));
                    reg |= ((config.neg_channels[i] as u8) << (i * 5)) as u32;
                }  
                w
            });
        }

        // scan mode
        aon.gpadc_reg_config1.modify(|_,w| unsafe {
            w.gpadc_scan_length().bits(scan_len as u8 - 1u8)
             .gpadc_cont_conv_en().bit(enable_continuous_dma)
             .gpadc_scan_en().bit(true)
        });
    }

    // ADC_Gain_Trim
    fn set_gain_trim(coef_enable: &mut bool, coef: &mut u16, coe: &mut f32) { 
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
                *coef_enable = true; 
                *coef = adc_gain_coeff as u16;

                let mut tmp = *coef;
                if tmp & 0x800 != 0 {
                    tmp = !tmp;
                    tmp += 1;
                    tmp = tmp & 0xfff;

                    *coe = 1.0 + (tmp as f32 / 2048.0);
                } else {
                    *coe = 1.0 - (tmp as f32 / 2048.0);
                }
            }
        }
    }

    pub fn start(&self) {
        // disable DMA_INT_ALL
        let ch1 = self.dma_ch.ch();
        ch1.ch_config.modify(|_,w| w.itc().set_bit().ie().set_bit());
        ch1.ch_control.modify(|_,w| w.i().clear_bit());

        // enable DMA_INT_TCOMPLETED
        ch1.ch_config.modify(|_,w| w.itc().clear_bit());
        ch1.ch_control.modify(|_,w| w.i().set_bit());

        // enable DMA_INT_ERR
        ch1.ch_config.modify(|_,w| w.ie().set_bit());

        {
            let aon = get_peripherals().AON;
            aon.gpadc_reg_cmd.modify(|_,w| w.gpadc_conv_start().clear_bit());
            dummy_wait();
            aon.gpadc_reg_cmd.modify(|_,w| w.gpadc_conv_start().set_bit());
        }

        adc_enable(true);

        {
            let dma = get_peripherals().DMA;
            dma.dma_top_config.modify(|_,w| w.e().set_bit());

        }

        // DMA channel 1 = ADC_CHANNEL
        ch1.ch_config.modify(|_,w| w.e().set_bit());
    }

    pub fn stop(&self) {
        get_peripherals().AON.gpadc_reg_cmd.modify(|_,w| w.gpadc_conv_start().set_bit());
        get_peripherals().DMA.dma_top_config.modify(|_,w| w.e().clear_bit());
    }

    pub fn fifo_count(&self) -> u8 {
        self.gpip.gpadc_config.read().gpadc_fifo_data_count().bits()
    }

    pub fn fifo_full(&self) -> bool {
        self.gpip.gpadc_config.read().gpadc_fifo_full().bit()
    }

    pub fn fifo_empty(&self) -> bool {
        self.gpip.gpadc_config.read().gpadc_fifo_ne().bit() == false
    }

    fn set_gain(config: &Config) {
        let aon = unsafe { &*pac::AON::ptr() };
        aon.gpadc_reg_config2.modify(|_, w| unsafe {
            w.gpadc_dly_sel().bits(0u8);
            w.gpadc_pga1_gain().bits(config.gain1 as u8);
            w.gpadc_pga2_gain().bits(config.gain2 as u8);
            w.gpadc_bias_sel().bit(config.bias_sel.into());

            // chopmode
            let has_gain = config.gain1 != AdcPgaGain::PgaGainNone || config.gain2 != AdcPgaGain::PgaGainNone;
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
            w.gpadc_pga_vcm().bits(config.vcm as u8);
            w.gpadc_vref_sel().bit(config.vref.into());
            w.gpadc_diff_mode().bit(config.input_mode.into());

            w
        });

        // calibration offset
        aon.gpadc_reg_define.modify(|_,w| unsafe {
            w.gpadc_os_cal_data().bits(config.offset_calibration_value)
        });
    }

}

impl<WORD, PIN> OneShot<Adc, WORD, PIN> for Adc
where WORD: From<u16>,
      PIN: Channel<Adc, ID=AdcChannel>,
{
    type Error = AdcError;

    fn read(&mut self, pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        let _adc_channel = pin.channel();

        self.start();

        let adc_val = self.gpip.gpadc_dma_rdata.read().bits() as u16;
        if adc_val == 0 {
            return Err(nb::Error::Other(AdcError::ErrRead));
        }

        return Ok(adc_val.into())
    }
}

impl dma::ReadTarget for Adc {
    type ReceivedWord = u16;

    fn rx_address_count(&self) -> (u32, u32) {
        (self.gpip.gpadc_dma_rdata.as_ptr() as u32, u32::MAX)
    }
    fn rx_treq() -> Option<u8> {
        // ??
        Some(0)
    }
    fn rx_increment(&self) -> bool {
        false
    }
}

#[inline]
fn get_peripherals() -> Peripherals {
    // TODO: Don't do this
    unsafe { pac::Peripherals::steal() }
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
