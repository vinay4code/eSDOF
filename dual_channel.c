#include "F28x_Project.h"
#include <math.h>

// ============================================================================
// SYSTEM DEFINITIONS & CONSTANTS
// ============================================================================
#define PI              3.141592653589793f
#define FS              12800.0f            // Sampling frequency (Hz)
#define TS              (1.0f / FS)         // Sampling period (sec)

// ADC & DAC Voltage Scaling Definitions
#define ADC_VREF        3.3f                // LaunchPad ADC reference voltage
#define DAC_VREF        3.3f                // LaunchPad DAC full scale voltage
#define ADC_OFFSET      (ADC_VREF / 2.0f)   // Offset to center AC signal at 1.5V



// ============================================================================
// STRUCTURE DEFINITIONS
// ============================================================================
// Biquad Filter Coefficients and States (Direct Form II Transposed)
typedef struct {
    float32 b0;
    float32 b1;
    float32 b2;
    float32 a1;
    float32 a2;
    float32 d1; // Delay state 1
    float32 d2; // Delay state 2
} Biquad_t;


// ============================================================================
// GLOBAL VARIABLES
// ============================================================================


Biquad_t g_H11,g_H12,g_H13;
Biquad_t g_H21,g_H22,g_H23;
Biquad_t g_H31,g_H32,g_H33;


// Tuning parameters (Modify live via expressions window)
volatile float32 g_f0 = 100.0f;             // Target Natural Frequency (Hz)
volatile float32 g_Q  = 10.0f;              // Quality Factor
float32 g_f0_active = 0.0f;
float32 g_Q_active  = 0.0f;



// Execution benchmarks
volatile Uint32 g_IsrCount = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void System_Init(void);
void ePWM1_Init(void);
void ADC_Init_Hardware(void);
void DAC_Init_Hardware(void);

void Filter_UpdateCoefficients(Biquad_t *f, float32 f0, float32 Q);
__interrupt void adca1_isr(void);

static inline float32 Biquad_Run(Biquad_t *f, float32 x);

// ============================================================================
// MAIN EXECUTION ENGINE
// ============================================================================
void main(void)
{
    System_Init();


    DINT; // Disable Global Interrupts during peripheral mapping

    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    // Map Interrupt vector to our real-time ADC conversion complete handler
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;
    EDIS;

    // Initialize Peripherals
    DAC_Init_Hardware();
    ADC_Init_Hardware();
    ePWM1_Init();

    // Initialize DSP Blocks
    Filter_UpdateCoefficients(&g_H11,60.0f,10.0f);
    Filter_UpdateCoefficients(&g_H12,60.0f,10.0f);
    Filter_UpdateCoefficients(&g_H13,60.0f,10.0f);

    Filter_UpdateCoefficients(&g_H21,150.0f,8.0f);
    Filter_UpdateCoefficients(&g_H22,150.0f,8.0f);
    Filter_UpdateCoefficients(&g_H23,150.0f,8.0f);

    Filter_UpdateCoefficients(&g_H31,350.0f,5.0f);
    Filter_UpdateCoefficients(&g_H32,350.0f,5.0f);
    Filter_UpdateCoefficients(&g_H33,350.0f,5.0f);

    g_f0_active = g_f0;
    g_Q_active = g_Q;
    Chirp_Init(&g_ChirpGenerator);

    // Enable PIE and Core Interrupts
    IER |= M_INT1;                      // Enable Core Group 1 Interrupts
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable ADCA1 Interrupt in PIE
    EINT;                               // Enable Global Interrupt (INTM)
    ERTM;                               // Enable Real-Time Debug Interrupt (DBGM)

    // Start Time Base Counter for ePWM1 to launch real-time execution
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

}

// ============================================================================
// INTERRUPT SERVICE ROUTINE (ISR) - Driven by Hardware ADC Conversion EOC
// ============================================================================
#pragma CODE_SECTION(adca1_isr, ".TI.ramfunc");
__interrupt void adca1_isr(void)
{
    Uint16 raw_adc1,raw_adc2,raw_adc3;

    float32 x1,x2,x3;
    float32 y1,y2,y3;

    float32 h11,h12,h13;
    float32 h21,h22,h23;
    float32 h31,h32,h33;

    raw_adc1 = AdcaResultRegs.ADCRESULT0;
    raw_adc2 = AdcbResultRegs.ADCRESULT0;
    raw_adc3 = AdccResultRegs.ADCRESULT0;

    x1 = ((float32)raw_adc1*(ADC_VREF/4095.0f))-ADC_OFFSET;
    x2 = ((float32)raw_adc2*(ADC_VREF/4095.0f))-ADC_OFFSET;
    x3 = ((float32)raw_adc3*(ADC_VREF/4095.0f))-ADC_OFFSET;

    h11 = Biquad_Run(&g_H11,x1);
    h12 = 0.20f*Biquad_Run(&g_H12,x2);
    h13 = 0.10f*Biquad_Run(&g_H13,x3);

    h21 = 0.25f*Biquad_Run(&g_H21,x1);
    h22 = Biquad_Run(&g_H22,x2);
    h23 = 0.15f*Biquad_Run(&g_H23,x3);

    h31 = 0.10f*Biquad_Run(&g_H31,x1);
    h32 = 0.20f*Biquad_Run(&g_H32,x2);
    h33 = Biquad_Run(&g_H33,x3);

    y1 = (h11+h12+h13)*0.5f;
    y2 = (h21+h22+h23)*0.5f;
    y3 = (h31+h32+h33)*0.5f;

    float32 v1 = y1 + ADC_OFFSET;
    float32 v2 = y2 + ADC_OFFSET;
    float32 v3 = y3 + ADC_OFFSET;

    if(v1>DAC_VREF) v1=DAC_VREF;
    if(v1<0.0f) v1=0.0f;

    if(v2>DAC_VREF) v2=DAC_VREF;
    if(v2<0.0f) v2=0.0f;

    if(v3>DAC_VREF) v3=DAC_VREF;
    if(v3<0.0f) v3=0.0f;

    Uint16 dac1,dac2,dac3;

    dac1 = (Uint16)((v1/DAC_VREF)*4095.0f);
    dac2 = (Uint16)((v2/DAC_VREF)*4095.0f);
    dac3 = (Uint16)((v3/DAC_VREF)*4095.0f);

    DacaRegs.DACVALS.bit.DACVALS = dac1;
    DacbRegs.DACVALS.bit.DACVALS = dac2;
    DaccRegs.DACVALS.bit.DACVALS = dac3;

    g_IsrCount++;

    // 6. Clear Peripheral Interrupt Request flags
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Clear ADC Flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge PIE group

}

// ============================================================================
// SIGNAL PROCESSING & CALCULATIONS REFERENCE
// ============================================================================
void Filter_UpdateCoefficients(Biquad_t *f, float32 f0, float32 Q)
{
    // Bound checks to safeguard against numerical division by zero
    if (f0 < 1.0f)   f0 = 1.0f;
    if (f0 > 5000.0f) f0 = 5000.0f;
    if (Q < 0.05f)   Q = 0.05f;

    float32 omega0 = 2.0f * PI * f0;
    float32 zeta = 1.0f / (2.0f * Q);
    float32 K = 2.0f * FS;

    // Intermediary computational denominators
    float32 K2 = K * K;
    float32 omega02 = omega0 * omega0;
    float32 Denom = K2 + (2.0f * zeta * omega0 * K) + omega02;

    // Normalize and load coefficients
    f->b0 = omega02 / Denom;
    f->b1 = (2.0f * omega02) / Denom;
    f->b2 = omega02 / Denom;
    f->a1 = (2.0f * omega02 - 2.0f * K2) / Denom;
    f->a2 = (K2 - (2.0f * zeta * omega0 * K) + omega02) / Denom;


    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

static inline float32 Biquad_Run(Biquad_t *f, float32 x)
{
    float32 y;

    y = (x * f->b0) + f->d1;

    f->d1 =
        (x * f->b1)
      - (y * f->a1)
      + f->d2;

    f->d2 =
        (x * f->b2)
      - (y * f->a2);

    return y;
}

// ============================================================================
// HARDWARE DRIVER CONFIGURATIONS (REGISTER LEVEL)
// ============================================================================
void System_Init(void)
{
    InitSysCtrl(); // Initializes Core PLL, Watchdog, Peripheral Clocks

    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR16.bit.DAC_A = 1;
    CpuSysRegs.PCLKCR16.bit.DAC_B = 1;
    CpuSysRegs.PCLKCR16.bit.DAC_C = 1;
    EDIS;
}

void ePWM1_Init(void)
{
    EALLOW;
    // Freeze counter during configurations
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_FREEZE;

    // Set Period for 12.8 kHz sampling rate
    // EPWMCLK = SYSCLK / 2 = 200 MHz / 2 = 100 MHz
    // TBPRD = EPWMCLK / FS = 100,000,000 / 12,800 = 7812.5 -> Choose 7812
    EPwm1Regs.TBPRD = 3906.25;
    EPwm1Regs.TBPHS.bit.TBPHS = 0;
    EPwm1Regs.TBCTR = 0;

    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Pre-scaler division = 1
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;

    // Setup ADC Start-of-Conversion Trigger on Time-base period match
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;        // Enable SOCA event generation
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // Trigger when CTR == PRD
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;   // Trigger on the 1st matching event
    EDIS;
}

void ADC_Init_Hardware(void)
{
    EALLOW;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;  // Power up analog circuitry
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Interrupt pulse at end of conversion
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;  // 0: 12-bit Resolution mode
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;  // 0: Single-Ended signal input

    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL2.bit.RESOLUTION = 0;
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;

    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL2.bit.RESOLUTION = 0;
    AdccRegs.ADCCTL2.bit.SIGNALMODE = 0;

    // Delay for internal power-up initialization
    DELAY_US(1000);

    // Map SOC0 to sample ADCINA0 pin
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 4;    // Input Channel: ADCINA0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;   // Sample Window duration = 15 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;  // Triggered by ePWM1 SOCA hardware line

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 3;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

    AdccRegs.ADCSOC0CTL.bit.CHSEL = 3;
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 14;
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

    // Setup Interrupt Response
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 triggers ADCINT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable ADCINT1 flag signal
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 triggers ADCINT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable ADCINT1 flag signal
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 triggers ADCINT1 flag
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable ADCINT1 flag signal
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;


    EDIS;
}

void DAC_Init_Hardware(void)
{
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1; // Use VREFHI/VDDA reference voltage
    DacaRegs.DACCTL.bit.LOADMODE = 0;  // Load immediately on DACVALS write
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // Enable internal output buffer analog pin
    DacaRegs.DACVALS.bit.DACVALS = 2048; // Set default mid-scale output

    DacbRegs.DACCTL.bit.DACREFSEL = 1;
    DacbRegs.DACCTL.bit.LOADMODE = 0;
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
    DacbRegs.DACVALS.bit.DACVALS = 2048;

    DaccRegs.DACCTL.bit.DACREFSEL = 1;
    DaccRegs.DACCTL.bit.LOADMODE = 0;
    DaccRegs.DACOUTEN.bit.DACOUTEN = 1;
    DaccRegs.DACVALS.bit.DACVALS = 2048;
    EDIS;
}