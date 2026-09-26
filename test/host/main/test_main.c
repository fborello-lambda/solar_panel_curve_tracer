#include <stdlib.h>

#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

// adaptive arc-length sweep
void test_sweep_adapt_reaches_isc_across_range(void);
void test_sweep_adapt_no_large_normalized_gaps(void);
void test_sweep_adapt_tiny_isc_terminates_within_budget(void);
void test_sweep_adapt_prints_isc_50ma_and_5ma_tables(void);

// sweep_range state machine
void test_sweep_range_probes_strictly_ascending_and_bounded(void);
void test_sweep_range_knee_estimate_tracks_isc(void);
void test_sweep_range_knee_times_headroom_clamped_to_hard_max(void);
void test_sweep_range_low_voc_is_no_panel(void);
void test_sweep_range_strong_panel_hits_hard_max(void);
void test_sweep_range_collapse_on_first_probe_is_sane(void);

// json_builder
void test_json_builder_zero_points(void);
void test_json_builder_one_point(void);
void test_json_builder_n_points(void);
void test_json_builder_truncates_small_buffer(void);
void test_json_builder_cap_fits_zero_elements(void);
void test_json_builder_cap_fits_one_element(void);
void test_json_builder_cap_fits_two_elements(void);
void test_json_builder_cap_too_small_for_brackets(void);

// quadrature decoder
void test_quadrature_cw_sequence_emits_one_step_per_cycle(void);
void test_quadrature_ccw_sequence_emits_negative_one_step_per_cycle(void);
void test_quadrature_bounce_emits_nothing(void);
void test_quadrature_invalid_jump_ignored(void);

void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_sweep_adapt_reaches_isc_across_range);
    RUN_TEST(test_sweep_adapt_no_large_normalized_gaps);
    RUN_TEST(test_sweep_adapt_tiny_isc_terminates_within_budget);
    RUN_TEST(test_sweep_adapt_prints_isc_50ma_and_5ma_tables);

    RUN_TEST(test_sweep_range_probes_strictly_ascending_and_bounded);
    RUN_TEST(test_sweep_range_knee_estimate_tracks_isc);
    RUN_TEST(test_sweep_range_knee_times_headroom_clamped_to_hard_max);
    RUN_TEST(test_sweep_range_low_voc_is_no_panel);
    RUN_TEST(test_sweep_range_strong_panel_hits_hard_max);
    RUN_TEST(test_sweep_range_collapse_on_first_probe_is_sane);

    RUN_TEST(test_json_builder_zero_points);
    RUN_TEST(test_json_builder_one_point);
    RUN_TEST(test_json_builder_n_points);
    RUN_TEST(test_json_builder_truncates_small_buffer);
    RUN_TEST(test_json_builder_cap_fits_zero_elements);
    RUN_TEST(test_json_builder_cap_fits_one_element);
    RUN_TEST(test_json_builder_cap_fits_two_elements);
    RUN_TEST(test_json_builder_cap_too_small_for_brackets);

    RUN_TEST(test_quadrature_cw_sequence_emits_one_step_per_cycle);
    RUN_TEST(test_quadrature_ccw_sequence_emits_negative_one_step_per_cycle);
    RUN_TEST(test_quadrature_bounce_emits_nothing);
    RUN_TEST(test_quadrature_invalid_jump_ignored);

    int failures = UNITY_END();

    // The linux target's app_main never "returns" to a shell in the usual
    // sense (FreeRTOS keeps the process alive), so exit explicitly with
    // Unity's failure count as the process exit code.
    exit(failures);
}
