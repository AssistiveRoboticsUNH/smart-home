(define (problem walking_reminder)
(:domain shr_domain)
(:objects
    current_loc dest_loc home outside - Landmark
    nathan - Person
    t1 t2 t3 t4 t5 - Time
    reminder_1_msg - Msg
    first_reminder - ReminderAction
    w1 w2 w3 w4 w5 - WaitAction
    na1 na2 na3 - NoAction
)
(:init
    ;; Initial person and robot locations
    ;;(person_at t1 nathan dest_loc)
    ;;(robot_at current_loc)

    ;; Enabled actions
    (DetectPerson_enabled)
    (GiveReminder_enabled)
    ;;(DetectTakingMedicine_enabled)

    ;; Time progression
    (current_time t1)
    (next_time t1 t2)
    (next_time t2 t3)
    (next_time t3 t4)
    (next_time t4 t5)



    ;; Person can be at different locations at future times
    (oneof (person_at t2 nathan current_loc) (person_at t2 nathan dest_loc) (person_at t2 nathan outside))
    (oneof (person_at t3 nathan current_loc) (person_at t3 nathan dest_loc) (person_at t3 nathan outside))
    (oneof (person_at t4 nathan current_loc) (person_at t4 nathan dest_loc) (person_at t4 nathan outside))
    (oneof (person_at t5 nathan current_loc) (person_at t5 nathan dest_loc) (person_at t5 nathan outside))

    (home_location home)


    ;; Allow traversal between locations if needed
    (traversable dest_loc current_loc)
    (traversable current_loc dest_loc)
    (traversable home current_loc)
    (traversable current_loc home)
    (traversable dest_loc home)
    (traversable home dest_loc)

    ;; Define success states
    (message_given_success reminder_1_msg)
    (person_at_success nathan outside)

    ;; Enforce same location constraint for interactions
    (same_location_constraint)

    ;; Specify required action order
    ;;(reminder_blocks_reminder first_reminder second_reminder)

    ;; Define valid messages for reminders
    (valid_reminder_message first_reminder reminder_1_msg)
    ;;(valid_reminder_message second_reminder reminder_2_msg)

    ;; Constraints: Reminders should not be given if Nathan is taking medicine
    ;;(reminder_person_not_taking_medicine_constraint first_reminder nathan)
    ;;(reminder_person_not_taking_medicine_constraint second_reminder nathan)

    ;; Ensure robot waits only when not outside
    (wait_not_person_location_constraint t1 nathan outside)
    (wait_not_person_location_constraint t2 nathan outside)
    (wait_not_person_location_constraint t3 nathan outside)
    (wait_not_person_location_constraint t4 nathan outside)
    (wait_not_person_location_constraint t5 nathan outside)

    ;; If person is outside, enforce no action
    (noaction_person_location_constraint na1 nathan outside)
    (noaction_person_location_constraint na2 nathan outside)
    (noaction_person_location_constraint na3 nathan outside)

    ;; Ensure the robot can wait only if at home
    (wait_robot_location_constraint t1 home)
    (wait_robot_location_constraint t2 home)
    (wait_robot_location_constraint t3 home)
    (wait_robot_location_constraint t4 home)
)
(:goal (and
        (success))
)
)
