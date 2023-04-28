use core::convert::Infallible;

use embedded_hal::adc::{self as adc, nb::OneShot, nb::Channel};
use embedded_time::rate::Hertz;
use pac::Peripherals;

use crate::{pac, gpio};

#[derive(PartialEq, Eq)]
pub enum AdcClockType {
    Clk96M,
    ClkX,
}

#[repr(u8)]
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
impl Into<bool> for AdcClockType {
    fn into(self) -> bool {
        match self {
            AdcClockType::Clk96M => false,
            AdcClockType::ClkX => true,
        }
    }
}
#[repr(u8)]
pub enum AdcV18Sel {
    Sel1p62v,
    Sel1p72v,
    Sel1p82v,
    Sel1p92v,
}
#[repr(u8)]
pub enum AdcV11Sel {
    Sel1p0v,
    Sel1p1v,
    Sel1p18v,
    Sel1p26v,
}
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
pub enum AdcDataWidth {
    Bits12,
    Bits14Avg16,
    Bits16Avg64,
    Bits16Avg128,
    Bits16Avg256,
}
#[repr(u8)]
#[derive(Eq, PartialEq)]
pub enum AdcPgaGain {
    PgaGainNone,
    PgaGain1,
    PgaGain2,
    PgaGain4,
    PgaGain8,
    PgaGain16,
    PgaGain32,
}
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
pub enum AdcPgaVcm {
    PgaVcm1V,
    PgaVcm1P2V,
    PgaVcm1P4V,
    PgaVcm1P6V,
}

const ERR_NO_PERIPHERALS: &str = "Unable to take hal bl602 peripherals";
const EF_CTRL_DFT_TIMEOUT_VAL: u32 = 160 * 1000;

// ADC_CFG_Type
struct Adc1 {
    enable: bool,
    v18_sel: AdcV18Sel,
    v11_sel: AdcV11Sel,
    clk_ty: AdcClockType,
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
}

impl<MODE> adc::nb::Channel<Adc1> for crate::gpio::Pin4<MODE> {
    type ID = u8;
    fn channel(&self) -> Self::ID {
        1_u8 // CH1
    }
}
impl<MODE> adc::nb::Channel<Adc1> for crate::gpio::Pin5<MODE> {
    type ID = u8;
    fn channel(&self) -> Self::ID {
        4_u8 // CH4
    }
}
impl<MODE> adc::nb::Channel<Adc1> for crate::gpio::Pin6<MODE> {
    type ID = u8;
    fn channel(&self) -> Self::ID {
        5_u8 // CH5
    }
}
impl<MODE> adc::nb::Channel<Adc1> for crate::gpio::Pin9<MODE> {
    type ID = u8;
    fn channel(&self) -> Self::ID {
        6_u8 // CH6/7 ??
    }
}
impl<MODE> adc::nb::Channel<Adc1> for crate::gpio::Pin10<MODE> {
    type ID = u8;
    fn channel(&self) -> Self::ID {
        8_u8 // CH8/9 ??
    }
}
impl<MODE> adc::nb::Channel<Adc1> for crate::gpio::Pin11<MODE> {
    type ID = u8;
    fn channel(&self) -> Self::ID {
        10 // CH10
    }
}
impl<MODE> adc::nb::Channel<Adc1> for crate::gpio::Pin12<MODE> {
    type ID = u8;
    fn channel(&self) -> Self::ID {
        0_u8 // CH0
    }
}
impl<MODE> adc::nb::Channel<Adc1> for crate::gpio::Pin13<MODE> {
    type ID = u8;
    fn channel(&self) -> Self::ID {
        3_u8 // CH3
    }
}
impl<MODE> adc::nb::Channel<Adc1> for crate::gpio::Pin14<MODE> {
    type ID = u8;
    fn channel(&self) -> Self::ID {
        2_u8 // CH2
    }
}
impl<MODE> adc::nb::Channel<Adc1> for crate::gpio::Pin15<MODE> {
    type ID = u8;
    fn channel(&self) -> Self::ID {
        11_u8 // CH11
    }
}

// 12 bit ADC
impl Adc1 { 
    pub fn new() -> Self {
        Self {
            enable: false,
            clk_ty: AdcClockType::Clk96M,
            v18_sel: AdcV18Sel::Sel1p82v,
            v11_sel: AdcV11Sel::Sel1p1v,
            clk_div: AdcClockDiv::Div24,
            gain1: AdcPgaGain::PgaGainNone,
            gain2: AdcPgaGain::PgaGainNone,
            vref: AdcVRefType::VRef3p2v,
            bias_sel: AdcBiasBandgap::BiasSelMainBandgap,
            vcm: AdcPgaVcm::PgaVcm1V,
            input_mode: AdcInputMode::SingleEnd,
            data_width: AdcDataWidth::Bits16Avg128,
            offset_calibration: false,
            offset_calibration_value: 0,
        }
    }
    pub fn init(&mut self) {
        self.enable(false);
        self.enable(true);
        self.reset();

        /// bl_adc_freq_init


        /// bl_adc_init

        //// ADC_Init

        // config1
        let peripherals = get_peripherals(); 
        {
            peripherals.AON.gpadc_reg_config1.modify(|r, w| unsafe {
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
        unsafe { set_dummy_wait(); }

        // config2
        let peripherals = get_peripherals();
        {
            peripherals.AON.gpadc_reg_config2.modify(|r, w| unsafe {
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
            peripherals.AON.gpadc_reg_define.modify(|r,w| unsafe {
                w.gpadc_os_cal_data().bits(self.offset_calibration_value)
            });
        }

        let mut coeff_en: bool;
        let mut coeff_val: u16;
        let mut coeff_coe: f32;
        Self::get_gain_trim(&mut coeff_en, &mut coeff_val, &mut coeff_coe);

        // TODO: SINGLE CHANNEL ONLY
        /*
        if (mode == 0) {
            for (i = 0; i < ADC_CHANNEL_MAX; i++) {
                pos_chlist_single[i] = i;
                neg_chlist_single[i] = ADC_CHAN_GND;
            }

            channel_num = ADC_CHANNEL_MAX;
        } else {
        */
        
        // TODO: DETERMINE CHANNEL SOMEHOW, AND INIT
    }

    pub fn deinit(&mut self) {}
    pub fn do_conversion(&mut self, _chan: u8) -> u16 { 0xFFFF }

    // ADC_Clock_Init >> GLB_Set_ADC_CLK
    fn glb_set_adc_clk(&self, ty: AdcClockType, clk: AdcClockDiv)
    {
        let peripherals = get_peripherals();

        // disable first
        peripherals.GLB.gpadc_32m_src_ctrl.modify(|r, w| unsafe {
            w.gpadc_32m_div_en().bit(false)
        });
        peripherals.GLB.gpadc_32m_src_ctrl.modify(|r,w| unsafe {
            w.gpadc_32m_clk_div().bits(self.clk_div as u8);
            w.gpadc_32m_clk_sel().bit(self.clk_ty.into());
            w
        });
        peripherals.GLB.gpadc_32m_src_ctrl.modify(|r,w| unsafe {
            w.gpadc_32m_div_en().bit(self.enable)
        });
    }

    fn enable(&mut self, enable: bool)
    {
        get_peripherals()
            .AON.gpadc_reg_cmd.modify(|r,w| unsafe { w.gpadc_global_en().bit(enable) });

        self.enable = enable;
    }

    fn reset(&self)
    {
        let peripherals = get_peripherals();
        peripherals.AON.gpadc_reg_cmd.modify(|r,w| unsafe { w.gpadc_soft_rst().bit(true)});

        unsafe { set_dummy_wait(); }

        peripherals.AON.gpadc_reg_cmd.modify(|r,w| unsafe { w.gpadc_soft_rst().bit(false)});
    }

    fn use_ahb_clk_0() {
        let mut timeout: u32 = EF_CTRL_DFT_TIMEOUT_VAL;
        let peripherals = get_peripherals();

        while Self::is_ctrl_busy(&peripherals) {
            timeout -= 1;
            if timeout == 0 {
                break;
            }
        }

        peripherals.EF_CTRL.ef_if_ctrl_0.write(|w| unsafe {
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

    fn get_gain_trim(coef_en: &mut bool, coef_val: &mut u16, coe: &mut f32) {

        Self::use_ahb_clk_0();
        
        let peripherals = get_peripherals();
        {
            let gain_trim_reg = peripherals.EF_DATA_0.ef_key_slot_5_w3.read().bits();
            let adc_gain_coeff = (gain_trim_reg >> 1) & 0xfff;
            let adc_gain_coeff_parity = (gain_trim_reg >> 13) & 0x01;
            let adc_gain_coeff_en = (gain_trim_reg >> 14) & 0x01 != 0;

            if adc_gain_coeff_en {
                if adc_gain_coeff as u8 == get_trim_parity(adc_gain_coeff_parity, 12) {
                    *coef_en = true; 
                    *coef_val = adc_gain_coeff as u16;

                    let mut tmp = *coef_val;
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
    }

    fn is_ctrl_busy(peripherals: &Peripherals) -> bool {
        peripherals.EF_CTRL.ef_if_ctrl_0.read()
          .ef_if_0_busy().bit_is_set()
    }
}

impl<WORD, PIN> OneShot<Adc1, WORD, PIN> for Adc1
where WORD: From<u16>,
      PIN: Channel<Adc1, ID=u8>,
{
    type Error = Infallible;

    fn read(&mut self, pin: &mut PIN) -> nb::Result<WORD, Self::Error> {

    }
}

unsafe fn nop(cycles: usize) {
    for i in 0..cycles {
        riscv::asm::nop();
    }
}

// AON_CLK_SET_DUMMY_WAIT
unsafe fn set_dummy_wait() {
    nop(8)
}

fn get_peripherals() -> Peripherals {
    pac::Peripherals::take().expect(ERR_NO_PERIPHERALS)
}

fn get_trim_parity(val: u32, len: u8) -> u8 {
    let c: u8 = 0;

    for i in 0..len {
        if val & (1 << i) != 0 {
            c += 1;
        }
    }

    c & 0x01
}
