(define (problem medicine_reminder)
(:domain shr_domain)
(:objects
    living_room home outside bedroom - Landmark
    nathan - Person
    t1 t2 t3 t4 t5 - Time
    reminder_1_msg reminder_2_msg - Msg
    first_reminder second_reminder - ReminderAction
    w1 w2 w3 w4 w5 - WaitAction
    na1 na2 na3 - NoAction
)
(:init
    ;;(person_at t1 nathan visible_area)
    ;;(robot_at designated_space)
    ;;(robot_at_time t1 designated_space)
    
    (DetectPerson_enabled)
    (GiveReminder_enabled)
    (DetectTakingMedicine_enabled)

    (current_time t1)
    (next_time t1 t2)
    (next_time t2 t3)
    (next_time t3 t4)
    (next_time t4 t5)

    (oneof (person_at t2 nathan living_room) (person_at t2 nathan bedroom)  (person_at t2 nathan outside) )
    (oneof (person_at t3 nathan living_room) (person_at t3 nathan bedroom)  (person_at t3 nathan outside) )
    (oneof (person_at t4 nathan living_room) (person_at t4 nathan bedroom)  (person_at t4 nathan outside) )
    (oneof (person_at t5 nathan living_room) (person_at t5 nathan bedroom)  (person_at t5 nathan outside))

    (traversable living_room home)
    (traversable home living_room)
    (traversable living_room outside)
    (traversable outside living_room)

    (traversable bedroom home)
    (traversable home bedroom)

    (traversable outside home)
    (traversable home outside)

    ;;success states
    (message_given_success reminder_2_msg)
    (medicine_taken_success)


    ;; uncomment this when you want same location constraint to be active
    ;; not sure why same same_location_constraint needs to be true for the other condition to work else it doesnt find a solution
    ;;(not_same_location_constraint)
    (same_location_constraint)

    ;; specify which actions must come before others
    (reminder_blocks_reminder first_reminder second_reminder)

    ;; specify valid input argument combinations for all actions
    (valid_reminder_message first_reminder reminder_1_msg)
    (valid_reminder_message second_reminder reminder_2_msg)

    ;; specify world state constraints for all actions
    (reminder_person_location_constraint first_reminder nathan living_room)
    (reminder_robot_location_constraint first_reminder living_room)
    (reminder_person_location_constraint second_reminder nathan living_room)
    (reminder_robot_location_constraint second_reminder living_room)

    (reminder_person_not_taking_medicine_constraint first_reminder nathan)
    (reminder_person_not_taking_medicine_constraint second_reminder nathan)

    ;;(wait_person_location_constraint t1 nathan visible_area)
    ;;(wait_person_location_constraint t2 nathan visible_area)
    ;;(wait_person_location_constraint t3 nathan visible_area)
    ;;(wait_person_location_constraint t4 nathan visible_area)
    ;;(wait_person_location_constraint t5 nathan visible_area)

    (wait_not_person_location_constraint t1 nathan outside)
    (wait_not_person_location_constraint t2 nathan outside)
    (wait_not_person_location_constraint t3 nathan outside)
    (wait_not_person_location_constraint t4 nathan outside)
    (wait_not_person_location_constraint t5 nathan outside)

    ;; if person is not in visible_area then wait
    ;;(noaction_not_person_location_constraint na1 nathan visible_area)
    ;;(noaction_not_person_location_constraint na2 nathan visible_area)
    ;;(noaction_not_person_location_constraint na3 nathan visible_area)

    ;; outside or no action will be used
    (noaction_person_location_constraint na1 nathan outside)
    (noaction_person_location_constraint na2 nathan outside)
    (noaction_person_location_constraint na3 nathan outside)

    (wait_robot_location_constraint t1 home)
    (wait_robot_location_constraint t2 home)
    (wait_robot_location_constraint t3 home)
    (wait_robot_location_constraint t4 home)
)
(:goal (and
        (success))
        )
)