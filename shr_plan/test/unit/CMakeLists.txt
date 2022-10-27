ament_add_gtest(medicine_reminder_tests medicine_reminder_tests.cpp)
ament_target_dependencies(medicine_reminder_tests ${dependencies})
target_link_libraries(medicine_reminder_tests
        shr_plan_parameters
        )
