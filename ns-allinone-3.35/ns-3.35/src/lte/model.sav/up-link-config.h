#define MAX_UL_CQI_16Q 11
#define UL_NUM_CQI 16
#define UL_16QAM_MAX_ITBS 21
#define UL_16QAM_TARGET_SINR 166
#define UL_64QAM_TARGET_SINR 176
#define P_O_NOMINAL_PUSCH -96
#define COOL_OFF_PERIOD_DURATION 4
#define GET_MIN(x, y) ((x) <= (y) ? (x) : (y))
#define GET_MAX(x, y) ((x) >= (y) ? (x) : (y))
#define MAX_UE_PWR 23
#define MIN_POWER_CORRECTION_FOR_PUSCH -63
#define MAX_POWER_CORRECTION_FOR_PUSCH  63
#define ALFA_HIGH 150
#define ALFA_LOW 150
#define FACTOR 1000
#define UP_64_QAM_PRECENTAGE 0
#define TARGET_BLER 10
#define MARGIN_STEP_DOWN 3
#define MIN_MAX_RBS 3
#define MIN_MCS 3
#define CHECK_AND_UPDATE_POWER_CORRECTION_FOR_PUSCH(powerCorrectionValue)\
{\
    if(MIN_POWER_CORRECTION_FOR_PUSCH > powerCorrectionValue) \
    {\
        powerCorrectionValue = MIN_POWER_CORRECTION_FOR_PUSCH;\
    }\
    else if(MAX_POWER_CORRECTION_FOR_PUSCH < powerCorrectionValue)\
    {\
        powerCorrectionValue = MAX_POWER_CORRECTION_FOR_PUSCH;\
    } \
}

enum State{UE_EFF,SPECTRAL_EFF,NEW_SPECTRAL_EFF};
