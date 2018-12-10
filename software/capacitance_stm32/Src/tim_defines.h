#define  TIM_SMCR_TS__ITR0                   0  // slave -- see below for individual timers
#define  TIM_SMCR_TS__ITR1                   1
#define  TIM_SMCR_TS__ITR2                   2
#define  TIM_SMCR_TS__ITR3                   3
#define  TIM_SMCR_TS__TI1F_ED                4  // both edges on CH1 input
#define  TIM_SMCR_TS__TI1FP1                 5  // TI1FP1 with the same polarity as if CH1 used for capturing in CC1
#define  TIM_SMCR_TS__TI2FP2                 6  // TI2FP2 with the same polarity as if CH2 used for capturing in CC2
#define  TIM_SMCR_TS__ETRF                   7  // external trigger input

#define  TIM_SMCR_SMS__NO                    0  // slave mode disabled, timer clocked from internal clock
#define  TIM_SMCR_SMS__ENCODER1              1  // TI2FP2=CLK, TI1FP1=dir
#define  TIM_SMCR_SMS__ENCODER2              2  // TI1FP1=CLK, TI2FP2=dir
#define  TIM_SMCR_SMS__ENCODER3              3  // quad mode, counts on both signal's edges
#define  TIM_SMCR_SMS__RESET                 4
#define  TIM_SMCR_SMS__GATED                 5
#define  TIM_SMCR_SMS__TRIGGER               6
#define  TIM_SMCR_SMS__EXT_CLK_1             7

// note - CCyS bits are writable only if TIMx_CCER.CCyE = 0
#define  TIM_CCMR_CCS__OUTPUT                0  // CC unit in compare mode
#define  TIM_CCMR_CCS__INPUT_TI1             1  // CC unit in capture mode, input from "own" pin (i.e. TI1 for CC1, TI2 for CC2, TI3 for CC3, TI4 for CC4)
#define  TIM_CCMR_CCS__INPUT_TI2             2  // CC unit in capture mode, input from "neighbour's" pin (i.e. TI2 for CC1, TI1 for CC2, TI4 for CC3, TI3 for CC4)
#define  TIM_CCMR_CCS__INPUT_TRC             3  // CC unit in capture mode, input from TRGI (this works only if internal trigger input is used, i.e. from other timer)
