#include <stdlib.h>

#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

// sweep_plan_build
void test_sweep_plan_build_shape_across_isc_range(void);
void test_sweep_plan_build_never_collapsed_case(void);
void test_sweep_plan_build_knee_clustering(void);
void test_sweep_duty_for_step_last_point_always_equals_top_by_construction(void);

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

void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_sweep_plan_build_shape_across_isc_range);
    RUN_TEST(test_sweep_plan_build_never_collapsed_case);
    RUN_TEST(test_sweep_plan_build_knee_clustering);
    RUN_TEST(test_sweep_duty_for_step_last_point_always_equals_top_by_construction);

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

    int failures = UNITY_END();

    // The linux target's app_main never "returns" to a shell in the usual
    // sense (FreeRTOS keeps the process alive), so exit explicitly with
    // Unity's failure count as the process exit code.
    exit(failures);
}
