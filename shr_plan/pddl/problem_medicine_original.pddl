(define (problem medicine_reminder)
(:domain shr_domain)
(:objects
    current_loc dest_loc - Landmark
    nathan - Person
    t1 t2 t3 t4 t5 - Time
    guide_1_msg guide_2_msg automated_msg recorded_msg call_caregiver_guide_msg call_caregiver_msg - Msg
    automated_reminder recorded_reminder guide_reminder_1 guide_reminder_2 - ReminderAction
    caregiver_call caregiver_call_guide - CallAction
)
(:init

	  ;; needed actions
    (DetectPerson_enabled)
    (GiveReminder_enabled)
    (MakeCall_enabled)
    (DetectTakingMedicine_enabled)

    (current_time t1)
    (next_time t1 t2)
    (next_time t2 t3)
    (next_time t3 t4)
    (next_time t4 t5)

    (oneof (person_at t2 nathan current_loc) (person_at t2 nathan dest_loc) )
    (oneof (person_at t3 nathan current_loc) (person_at t3 nathan dest_loc) )
    (oneof (person_at t4 nathan current_loc) (person_at t4 nathan dest_loc) )
    (oneof (person_at t5 nathan current_loc) (person_at t5 nathan dest_loc) )

    (traversable dest_loc current_loc)
    (traversable current_loc dest_loc)

    ;;success states
    (message_given_success call_caregiver_msg)
    (message_given_success call_caregiver_guide_msg)
    (medicine_taken_success)

    ;; specify which actions must come before others
    (reminder_blocks_reminder automated_reminder recorded_reminder)
    (reminder_blocks_call recorded_reminder caregiver_call)
    (reminder_blocks_reminder guide_reminder_1 guide_reminder_2)
    (reminder_blocks_call guide_reminder_2 caregiver_call_guide)

    ;; specify valid input argument combinations for all actions
    (valid_call_message caregiver_call call_caregiver_msg)
    (valid_call_message caregiver_call_guide call_caregiver_guide_msg)
    (valid_reminder_message automated_reminder automated_msg)
    (valid_reminder_message recorded_reminder recorded_msg)
    (valid_reminder_message guide_reminder_1 guide_1_msg)
    (valid_reminder_message guide_reminder_2 guide_2_msg)

    ;; specify world state constraints for all actions
    (reminder_person_location_constraint automated_reminder nathan dest_loc)
    (reminder_person_not_taking_medicine_constraint automated_reminder nathan)
    (reminder_person_location_constraint recorded_reminder nathan dest_loc)
    (reminder_person_not_taking_medicine_constraint recorded_reminder nathan)
    (reminder_not_person_location_constraint guide_reminder_1 nathan dest_loc)
    (reminder_person_not_taking_medicine_constraint guide_reminder_1 nathan)
    (reminder_not_person_location_constraint guide_reminder_2 nathan dest_loc)
    (reminder_person_not_taking_medicine_constraint guide_reminder_2 nathan)
    (call_not_person_location_constraint caregiver_call_guide nathan dest_loc)
    (call_person_not_taking_medicine_constraint caregiver_call nathan)

)
(:goal (and (success)  ) )

)