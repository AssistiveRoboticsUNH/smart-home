(define (problem gym_reminder)
(:domain shr_domain)
(:objects
    bedroom visible_area outside bathroom - LandmarkPerson
    designated_space home - LandmarkRobot
    nathan - Person
    t1 t2 t3 t4 t5 - Time
    reminder_1_msg reminder_2_msg reminder_3_msg - Msg
    first_reminder second_reminder third_reminder - ReminderAction
    w1 w2 w3 w4 w5 - WaitAction
    na1 na2 na3 - NoAction
)
(:init
    ;;(person_at t1 nathan visible_area)
    ;;(robot_at home)
    ;;(robot_at_time t1 home)

    ;;(person_at t1 nathan visible_area)
    ;;(robot_at home)
    ;;(robot_at_time t1 home)

    ;;(no_action)
    (DetectPerson_enabled)
    (GiveReminder_enabled)

    (current_time t1)
    (next_time t1 t2)
    (next_time t2 t3)
    (next_time t3 t4)
    (next_time t4 t5)

    (oneof (person_at t2 nathan bedroom) (person_at t2 nathan visible_area)  (person_at t2 nathan outside) (person_at t2 nathan bathroom) )
    (oneof (person_at t3 nathan bedroom) (person_at t3 nathan visible_area)  (person_at t3 nathan outside) (person_at t3 nathan bathroom) )
    (oneof (person_at t4 nathan bedroom) (person_at t4 nathan visible_area)  (person_at t4 nathan outside) (person_at t4 nathan bathroom) )
    (oneof (person_at t5 nathan bedroom) (person_at t5 nathan visible_area)  (person_at t5 nathan outside) (person_at t5 nathan bathroom) )

    (traversable designated_space home)
    (traversable home designated_space)

    ;;success states
    (message_given_success reminder_3_msg)
    (person_at_success nathan outside)

    ;; specify which actions must come before others
    (reminder_blocks_reminder first_reminder second_reminder )
    (reminder_blocks_reminder second_reminder third_reminder )

    ;; specify valid input argument combinations for all actions
    (valid_reminder_message first_reminder reminder_1_msg)
    (valid_reminder_message second_reminder reminder_2_msg)
    (valid_reminder_message third_reminder reminder_3_msg)

    ;; specify world state constraints for all actions
    (reminder_person_location_constraint first_reminder nathan visible_area)
    (reminder_robot_location_constraint first_reminder designated_space)

    (reminder_person_location_constraint second_reminder nathan visible_area)
    (reminder_robot_location_constraint second_reminder designated_space)

    (reminder_person_location_constraint third_reminder nathan visible_area)
    (reminder_robot_location_constraint third_reminder designated_space)

    (wait_person_location_constraint t1 nathan visible_area)
    (wait_person_location_constraint t2 nathan visible_area)
    (wait_person_location_constraint t3 nathan visible_area)
    (wait_person_location_constraint t4 nathan visible_area)
    (wait_person_location_constraint t5 nathan visible_area)

     ;;(noaction_person_location_constraint na1 nathan bedroom)
     ;;(noaction_person_location_constraint na2 nathan bedroom)
     ;;(noaction_person_location_constraint na3 nathan bedroom)
     ;;(noaction_person_location_constraint na1 nathan bathroom)
     ;;(noaction_person_location_constraint na2 nathan bathroom)
     ;;(noaction_person_location_constraint na3 nathan bathroom)

    ;; if person is not in visible_area then wait
    (noaction_not_person_location_constraint na1 nathan visible_area)
    (noaction_not_person_location_constraint na2 nathan visible_area)
    (noaction_not_person_location_constraint na3 nathan visible_area)

    (wait_robot_location_constraint t1 home)
    (wait_robot_location_constraint t2 home)
    (wait_robot_location_constraint t3 home)
    (wait_robot_location_constraint t4 home)
)
(:goal (and
        (success)) 
        )
)