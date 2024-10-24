(define (domain shr_domain)

(:requirements :strips :typing)

(:types
    Landmark
	Person
	Time
	Msg
	ReminderAction
    WaitAction
    NoAction
  )

(:predicates
    ;; physical modeling
    (robot_at ?lmr - Landmark)
    (robot_at_time ?t - Time ?lmr - Landmark)
    (person_at ?t - Time ?p - Person ?lmp - Landmark)
    ;;(person_currently_at ?p - Person ?lmp - Landmark)
    (person_at_success ?p - Person ?lmp - Landmark)


    (person_taking_medicine ?t - Time)
    (person_eating_food ?t - Time)

    (no_action)
    (move_to_home_enabled)

    ;; physical constants
    (traversable ?from ?to - Landmark)

    ;; control flow
    (abort)

    ;; effects of actions
    (message_given ?m - Msg)

    ;; success conditions
    (message_given_success ?m - Msg)
    (success_location ?lmp - Landmark)
    (medicine_taken_success)
    (food_eaten_success)


    ;; enable/disable actions
    (GiveReminder_enabled)
    (DetectPerson_enabled)
    (DetectEatingFood_enabled)
    (DetectTakingMedicine_enabled)

    ;; enforce action sequence dependencies
    (reminder_blocks_reminder ?a1 ?a2 - ReminderAction)
    (executed_reminder ?a - ReminderAction)
    (executed_wait ?t - Time)

    ;; enforce that actions are called with valid object instances
    (valid_reminder_message ?a - ReminderAction ?m - Msg)

    (same_location_constraint)
    (not_same_location_constraint)

    ;; time management predicates
    (time_critical)
    (used_move ?tc - Time ?lmr - Landmark)
    (used_reminder ?tc - Time)

    (current_time ?tc - Time)
    (next_time ?tc ?tn - Time)

    ;; constraints on the state of the world. object instances here refer to non-input instances
    (reminder_robot_location_constraint ?a - ReminderAction ?lmr - Landmark)
    (reminder_person_location_constraint ?a - ReminderAction ?p - Person ?lmp - Landmark)
    (reminder_person_not_location_constraint ?a - ReminderAction ?p - Person ?lmp - Landmark)
    (wait_not_person_location_constraint ?t - Time ?p - Person ?lmp - Landmark )
    (wait_person_location_constraint ?t - Time ?p - Person ?lmp - Landmark )
    (noaction_not_person_location_constraint ?na - NoAction ?p - Person ?lmp - Landmark)
    (noaction_person_location_constraint ?na - NoAction ?p - Person ?lmp - Landmark)

    (reminder_person_not_taking_medicine_constraint ?a - ReminderAction ?p - Person)
    (reminder_person_not_eating_food_constraint ?a - ReminderAction ?p - Person)
    (wait_robot_location_constraint ?t - Time ?lmp - Landmark )

    (success)
    (na_used ?na - NoAction)

)

;; detect if person is at location
(:action DetectTakingMedicine
    :parameters (?t - Time)
    :precondition (and
                    (DetectTakingMedicine_enabled)
                    (current_time ?t)
                    (not (abort))
	                )
    :observe (person_taking_medicine ?t)
)

;; detect if person is at location
(:action DetectEatingFood
    :parameters (?t - Time)
    :precondition (and
                    (DetectEatingFood_enabled)
                    (current_time ?t)
                    (not (abort))
	                )
    :observe (person_eating_food ?t)
)

;; detect if person is at location outside or inside or bedroom
(:action DetectPersonLocation
    :parameters (?t - Time ?p - Person ?lmp - Landmark)
    :precondition (and
                    (current_time ?t)
                    (DetectPerson_enabled)
                    (not (abort))
	                )
    :observe (person_at ?t ?p ?lmp)
)

;; Move to any landmark, avoiding terrain
(:action MoveToLandmark
	:parameters (?t - Time ?from - Landmark ?to - Landmark)
	:precondition (and
	                (current_time ?t)
	                (not (used_move ?t ?to))
	                (robot_at ?from)
	                (traversable ?from ?to)
	                (not (abort))
	          )
	:effect (and (robot_at ?to) (not (robot_at ?from)) (used_move ?t ?to)
	          (when (time_critical)
              (forall (?tn - Time)
                (when (next_time ?t ?tn) (and (not (current_time ?t)) (current_time ?tn) (robot_at_time ?tn ?to) ) )
              )
	          )
            (when (not (time_critical))
              (robot_at_time ?t ?to)
            )
	        )
)

;;give reminder
(:action GiveReminder
    :parameters (?a - ReminderAction ?t - Time ?p - Person ?m - Msg)
    :precondition (and
            ;;(not move_to_home_enabled)
            (GiveReminder_enabled)
            (current_time ?t)

            (not (used_reminder ?t))
            (valid_reminder_message ?a ?m)

            (not (executed_reminder ?a))

            ;; enforce that the person didn't take medicine constraint
            (not (and (reminder_person_not_taking_medicine_constraint ?a ?p)  (not (not (person_taking_medicine ?t)) ) ) )
            ;; enforce that the person didn't eat food constraint
            (not (and (reminder_person_not_eating_food_constraint ?a ?p)  (not (not (person_eating_food ?t)) ) ) )


            ;; certain action instances block others, for example, we must call caregiver before calling emergency
            (forall (?ai - ReminderAction)
              (not (and (reminder_blocks_reminder ?ai ?a)  (not (executed_reminder ?ai) ) ) )
            )

            ;; Either robot and person have to be in same location or in designated locations
            ;; !(a || b) is equivalent to !a && !b
            ;; !!(a || b) = (a || b)  is equivalent to ! (!a && !b)


            (same_location_constraint)

            ;; the robot and person must be at the same location
            ;; gives true when robot and person are at the same location
            (not
                (forall (?loc - Landmark)
                    (not (and (person_at ?t ?p ?loc) (robot_at ?loc)) )
                )
            )

            ;; this condition enforces that the person is not at the location specified in not_person_location_constraint
            ;;(forall (?lmp - Landmark)
            ;;  (not (and (person_at ?t ?p ?lmp) (reminder_person_not_location_constraint ?a ?p ?lmp) ) )
            ;;)
            (not (abort))
		)
    :effect (and (message_given ?m)  (executed_reminder ?a)
              ;;(forall (?tn - Time)
              ;;  (when (next_time ?t ?tn) (and (not (current_time ?t)) (current_time ?tn)) )
              ;;)
              (used_reminder ?t)

            )
)


;; Wait for timestep
(:action Wait
	:parameters (?t - Time ?p - Person)
	:precondition (and
                  ;; this condition enforces that the robot is at the location specified in person_location_constraint
                  (forall (?lmr - Landmark)
                    (not (and (not (robot_at ?lmr)) (wait_robot_location_constraint ?t ?lmr) ) )
                  )

                  (current_time ?t)
	              (not (executed_wait ?t))
                  (not (abort))
                  ;;(forall (?lmp - Landmark)
                  ;;  (not (and (not (person_at ?t ?p ?lmp)) (wait_person_location_constraint ?t ?p ?lmp) ) )
                  ;;)
                  (forall (?lmp - Landmark)
                    (not (and (not (person_at ?t ?p ?lmp)) (wait_person_location_constraint ?t ?p ?lmp) ) )
                  )
                  (forall (?lmp - Landmark)
                    (not (and (person_at ?t ?p ?lmp) (wait_not_person_location_constraint ?t ?p ?lmp) ) )
                  )
	             )
	:effect (and (executed_wait ?t)
            (forall (?tn - Time)
              (when (next_time ?t ?tn) (and (not (current_time ?t)) (current_time ?tn)) )
            )
	)
)


;; Update success status
(:action MessageGivenSuccess
	:parameters ()
	:precondition (and
	                (not
                        (forall (?m - Msg)
                          (not (and (message_given_success ?m) (message_given ?m) ) )
                        )
                    )
                    (not (abort))
                  )
    :effect (success)
)

(:action PersonAtSuccess
	:parameters (?p - Person ?t - Time ?lmp - Landmark)
	:precondition (and
	                ;;(person_currently_at ?p - Person ?lmp - Landmark)
	                (person_at_success ?p ?lmp)
	                (success_location ?lmp)
	                (not (abort))
                  )
    :effect (success)
)


;; check if person Left
(:action NoActionUsed
	:parameters (?t - Time ?p - Person ?na - NoAction)
	:precondition (and

	              (not (no_action))
                  ;; this condition enforces that the person is at the location specified in person_location_constraint
                  (forall (?loc - Landmark)
                    (not (and (not (person_at ?t ?p ?loc)) (noaction_person_location_constraint ?na ?p ?loc) ) )
                  )

                  ;; this condition enforces that the robot is at the location specified in person_location_constraint
                    (forall (?lmr - Landmark)
                      (not (and (not (robot_at ?lmr)) (wait_robot_location_constraint ?t ?lmr) ) )
                    )

                  (not (na_used ?na))
                  ;; this condition enforces that the person is not at the location specified in not_person_location_constraint
                  (forall (?loc - Landmark)
                    (not (and (person_at ?t ?p ?loc) (noaction_not_person_location_constraint ?na ?p ?loc) ) )
                  )
                  (current_time ?t)
	                (not (abort))
                )
    :effect (and (na_used ?na)
            (forall (?tn - Time)
              (when (next_time ?t ?tn) (and (not (current_time ?t)) (current_time ?tn)) )
            )
            )
)


;; morgans law there exists with forall instead of using when
(:action TimeOut
	:parameters ()
	:precondition (and
                  (forall (?na - NoAction)
                    (na_used ?na)
                  )
                  (not (abort))
                )
    :effect (success)
)

;; taking medicine
(:action MedicineTakenSuccess
	:parameters ()
	:precondition (and
	                (not (forall (?t - Time)
                          (not (and (medicine_taken_success) (person_taking_medicine ?t) ) )
                       )
	                )
	                (not (abort))
                )
    :effect (success)
)

;; eating food
(:action FoodEatenSuccess
	:parameters ()
	:precondition (and
	                (not (forall (?t - Time)
                          (not (and (food_eaten_success) (person_eating_food ?t) ) )
                       )
	                )
	                (not (abort))
                )
    :effect (success)
)

)