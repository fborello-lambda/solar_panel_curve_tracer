idf_component_register(SRCS "test_main.c"
                             "test_sweep_adapt.c"
                             "test_sweep_range.c"
                             "test_json_builder.c"
                             "test_quadrature.c"
                             "test_chart_math.c"
                        PRIV_REQUIRES unity sweep_plan json_builder quadrature chart_math)

# test_sweep_range.c's and test_sweep_adapt.c's panel simulations use
# exp()/fabsf()/sqrtf() from libm, which isn't linked in automatically on
# the host build.
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
