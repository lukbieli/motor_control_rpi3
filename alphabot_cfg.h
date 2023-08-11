#ifndef E8EF0C6B_B704_4DB3_A43B_913D6B4E8FDD
#define E8EF0C6B_B704_4DB3_A43B_913D6B4E8FDD

// #define HW_ORIGINAL
#define HW_N20D

#define PIN_MOTORL_EN (6)
#define PIN_MOTORR_EN (26)
#define PIN_HBRIDGE_IN1 (12)
#define PIN_HBRIDGE_IN2 (13)
#define PIN_HBRIDGE_IN3 (20)
#define PIN_HBRIDGE_IN4 (21)

#define PIN_ENC_R1 (8)
#define PIN_ENC_L1 (7)

#if defined(HW_N20D)
#define PIN_ENC_R2 (16)
#define PIN_ENC_L2 (19)
#endif

#define PIN_ADC_IOCLK (25)
#define PIN_ADC_ADDR (24)
#define PIN_ADC_DOUT (23)
#define PIN_ADC_CS (5)

#endif /* E8EF0C6B_B704_4DB3_A43B_913D6B4E8FDD */
