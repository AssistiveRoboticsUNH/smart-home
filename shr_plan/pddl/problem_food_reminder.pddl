(define (problem food_reminder)
(:domain shr_domain)
(:objects
    bedroom inside_not_bedroom outside - LandmarkPerson
    designated_space home - LandmarkRobot
    nathan - Person
    t1 t2 t3 t4 t5 - Time
    reminder_1_msg reminder_2_msg - Msg
    first_reminder second_reminder - ReminderAction
    w1 w2 w3 w4 w5 - WaitAction
    na1 na2 na3 - NoAction
)
(:init
    ;;(person_at t1 nathan inside_not_bedroom)
    ;;(robot_at home)
    ;;(robot_at_time t1 home)

    (DetectPerson_enabled)
    (GiveReminder_enabled)
    (DetectEatingFood_enabled)

    (current_time t1)
    (next_time t1 t2)
    (next_time t2 t3)
    (next_time t3 t4)
    (next_time t4 t5)

    (oneof (person_at t2 nathan bedroom) (person_at t2 nathan inside_not_bedroom)  (person_at t2 nathan outside) )
    (oneof (person_at t3 nathan bedroom) (person_at t3 nathan inside_not_bedroom)  (person_at t3 nathan outside) )
    (oneof (person_at t4 nathan bedroom) (person_at t4 nathan inside_not_bedroom)  (person_at t4 nathan outside) )
    (oneof (person_at t5 nathan bedroom) (person_at t5 nathan inside_not_bedroom)  (person_at t5 nathan outside) )

    (traversable designated_space home)
    (traversable home designated_space)

    ;;success states
    (message_given_success reminder_2_msg)
    (food_eaten_success)


    ;; specify which actions must come before others
    (reminder_blocks_reminder first_reminder second_reminder)

    ;; specify valid input argument combinations for all actions
    (valid_reminder_message first_reminder reminder_1_msg)
    (valid_reminder_message second_reminder reminder_2_msg)

    ;; specify world state constraints for all actions
    (reminder_person_location_constraint first_reminder nathan inside_not_bedroom)
    (reminder_robot_location_constraint first_reminder designated_space)
    (reminder_person_location_constraint second_reminder nathan inside_not_bedroom)
    (reminder_robot_location_constraint second_reminder designated_space)

    (reminder_person_not_eating_food_constraint first_reminder nathan)
    (reminder_person_not_eating_food_constraint second_reminder nathan)

    (wait_person_location_constraint t1 nathan inside_not_bedroom)
    (wait_person_location_constraint t2 nathan inside_not_bedroom)
    (wait_person_location_constraint t3 nathan inside_not_bedroom)
    (wait_person_location_constraint t4 nathan inside_not_bedroom)
    (wait_person_location_constraint t5 nathan inside_not_bedroom)

    (noaction_not_person_location_constraint na1 nathan inside_not_bedroom)
    (noaction_not_person_location_constraint na2 nathan inside_not_bedroom)
    (noaction_not_person_location_constraint na3 nathan inside_not_bedroom)

    (wait_robot_location_constraint t1 home)
    (wait_robot_location_constraint t2 home)
    (wait_robot_location_constraint t3 home)
    (wait_robot_location_constraint t4 home)
)
(:goal (and
        (success)) 
        )
)