# Assuming lcm_FOUND

drake_add_cc_test(pose_t_translator_test)
target_link_libraries(pose_t_translator_test
    robotlocomotionTranslators)
