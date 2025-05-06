#ifdef __cplusplus
extern "C" {
#endif

#if defined CONFIG_LV_USE_DEMO_WIDGETS
    #include "lv_examples/src/lv_demo_widgets/lv_demo_widgets.h"
#elif defined CONFIG_LV_USE_DEMO_KEYPAD_AND_ENCODER
    #include "lv_examples/src/lv_demo_keypad_encoder/lv_demo_keypad_encoder.h"
#elif defined CONFIG_LV_USE_DEMO_BENCHMARK
    #include "lv_examples/src/lv_demo_benchmark/lv_demo_benchmark.h"
#elif defined CONFIG_LV_USE_DEMO_STRESS
    #include "lv_examples/src/lv_demo_stress/lv_demo_stress.h"
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
