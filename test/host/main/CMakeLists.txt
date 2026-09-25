idf_component_register(SRCS "test_main.c"
                             "test_sweep_plan.c"
                             "test_sweep_range.c"
                             "test_json_builder.c"
                        PRIV_REQUIRES unity sweep_plan json_builder)

# test_sweep_range.c's panel simulation uses exp()/fabsf() from libm, which
# isn't linked in automatically on the host build.
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
