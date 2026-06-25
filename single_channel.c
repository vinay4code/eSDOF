#include "F28x_Project.h"
#include <math.h>

// ============================================================================
// SYSTEM DEFINITIONS & CONSTANTS
// ============================================================================
#define PI              3.141592653589793f
#define FS              12800.0f// Sampling frequency (Hz)
#define TS              (1.0f / FS)         // Sampling period (sec)

// ADC & DAC Voltage Scaling Definitions
#define ADC_VREF        3.3f                // LaunchPad ADC reference voltage
#define DAC_VREF        3.3f                // LaunchPad DAC full scale voltage
#define ADC_OFFSET      (ADC_VREF/2.0f)   // Offset to center AC signal at 1.5V


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

Biquad_t g_ResonanceFilter;

// Tuning parameters (Modify live via expressions window)
volatile float32 g_f0 = 60.0f;             // Target Natural Frequency (Hz)
volatile float32 g_Q  = 10.0f;              // Quality Factor
float32 g_f0_active = 0.0f;
float32 g_Q_active  = 0.0f;

volatile float32 x_t = 0.0f;
volatile float32 y_t = 0.0f;
volatile Uint16 raw_adc = 0;
volatile Uint16 raw_dac = 0;


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
    Filter_UpdateCoefficients(&g_ResonanceFilter, g_f0, g_Q);
    g_f0_active = g_f0;
    g_Q_active = g_Q;

    // Enable PIE and Core Interrupts
    IER |= M_INT1;                      // Enable Core Group 1 Interrupts
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;  // FIX: Changed PieCtrlRegisters to PieCtrlRegs
    EINT;                               // Enable Global Interrupt (INTM)
    ERTM;                               // Enable Real-Time Debug Interrupt (DBGM)

    // Start Time Base Counter for ePWM1 to launch real-time execution
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    // Background Processing Loop
    while(1)
    {
        // Real-time parameter update safe check
        if ((g_f0 != g_f0_active) || (g_Q != g_Q_active))
        {
            DINT; // Critical section to ensure atomicity during calculation
            Filter_UpdateCoefficients(&g_ResonanceFilter, g_f0, g_Q);
            g_f0_active = g_f0;
            g_Q_active = g_Q;
            EINT;
        }
    }
}

// ============================================================================
// INTERRUPT SERVICE ROUTINE (ISR)
// ============================================================================
//#pragma CODE_SECTION(adca1_isr, ".TI.ramfunc");
__interrupt void adca1_isr(void)
{


    // 1. Read raw digitized signal from hardware register
    raw_adc = AdcaResultRegs.ADCRESULT0;

    
    // Translate unipolar 12-bit ADC data to zero-mean bipolar voltage
    x_t = ((float32)raw_adc * (ADC_VREF / 4095.0f)) - ADC_OFFSET;
    

    // 3. Digital Processing: Direct Form II Transposed Biquad Equation Execution
    y_t = (x_t * g_ResonanceFilter.b0) + g_ResonanceFilter.d1;
    g_ResonanceFilter.d1 = (x_t * g_ResonanceFilter.b1) - (y_t * g_ResonanceFilter.a1) + g_ResonanceFilter.d2;
    g_ResonanceFilter.d2 = (x_t * g_ResonanceFilter.b2) - (y_t * g_ResonanceFilter.a2);

    // 4. Output Scaling & Hard Anti-Clipping Saturation
    // Re-bias from zero-mean voltage to unipolar DAC space
    float32 v_out = y_t + ADC_OFFSET;

    if(v_out > ADC_VREF)  v_out = ADC_VREF;   // Clamp to ADC range to avoid saturating input stage
    if(v_out < 0.0f)      v_out = 0.0f;


    // Convert filtered voltage to DAC Digital Integer Code
    raw_dac = (Uint16)((v_out / DAC_VREF) * 4095.0f);
    DacaRegs.DACVALS.bit.DACVALS = raw_dac;

    g_IsrCount++;

    // 6. Clear Peripheral Interrupt Request flags
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Clear ADC Flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

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
}


// ============================================================================
// HARDWARE DRIVER CONFIGURATIONS (REGISTER LEVEL)
// ============================================================================
void System_Init(void)
{
    InitSysCtrl(); // Initializes Core PLL, Watchdog, Peripheral Clocks

    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    CpuSysRegs.PCLKCR16.bit.DAC_A = 1;
    EDIS;
}


void ePWM1_Init(void)
{
    EALLOW;
    // Freeze counter during configurations
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_FREEZE;

    // Set Period for 12.8 kHz sampling rate
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

    // Delay for internal power-up initialization
    DELAY_US(1000);

    // Map SOC0 to sample ADCINA1 pin
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 1;    // Input Channel: ADCINA1
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;   // Sample Window duration = 15 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;  // Triggered by ePWM1 SOCA hardware line

    // Setup Interrupt Response
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 triggers ADCINT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable ADCINT1 flag signal
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    EDIS;
}

void DAC_Init_Hardware(void)
{
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1;
    DacaRegs.DACCTL.bit.LOADMODE = 0;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // Enable internal output buffer analog pin
    DacaRegs.DACVALS.bit.DACVALS = 2048; // Set default mid-scale output
    EDIS;
}
