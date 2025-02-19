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
    CallAction
  )

(:predicates
    ;; physical modeling
    (robot_at ?lmr - Landmark)
    (robot_at_time ?t - Time ?lmr - Landmark)
    (person_at ?t - Time ?p - Person ?lmp - Landmark)
    ;;(person_currently_at ?p - Person ?lmp - Landmark)
    (person_at_success ?p - Person ?lmp - Landmark)
    (same_location ?l1 ?l2 - Landmark)
    (home_location ?l - Landmark)
    (moved_after_landmark ?t - Time)
    (wait_completed ?t - Time)
    (waiting_required ?t - Time)


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
    (MakeCall_enabled)


    ;; enforce action sequence dependencies
    (call_blocks_call ?a1 ?a2 - CallAction)
    (reminder_blocks_call ?a1 - ReminderAction ?a2 - CallAction)
    (reminder_blocks_reminder ?a1 ?a2 - ReminderAction)
    (executed_reminder ?a - ReminderAction)
    (executed_call ?c - CallAction)
    (executed_wait ?a - WaitAction)
    (wait_blocks_wait ?a1 - WaitAction ?a2 - WaitAction)

    ;; enforce that actions are called with valid object instances
    (valid_reminder_message ?a - ReminderAction ?m - Msg)
    (valid_call_message ?a - CallAction ?m - Msg)

    (same_location_constraint)
    (not_same_location_constraint)

    ;; time management predicates
    (time_critical)
    (used_move ?tc - Time ?lmr - Landmark)
    (used_reminder ?tc - Time)
    (used_call ?tc - Time)

    (current_time ?tc - Time)
    (next_time ?tc ?tn - Time)

    ;; constraints on the state of the world. object instances here refer to non-input instances
    ;;(reminder_robot_location_constraint ?a - ReminderAction ?lmr - Landmark)
    (reminder_person_location_constraint ?a - ReminderAction ?p - Person ?lmp - Landmark)
    (reminder_person_not_location_constraint ?a - ReminderAction ?p - Person ?lmp - Landmark)
    (wait_not_person_location_constraint ?t - Time ?p - Person ?lmp - Landmark )
    (wait_person_location_constraint ?t - Time ?p - Person ?lmp - Landmark )
    (noaction_not_person_location_constraint ?na - NoAction ?p - Person ?lmp - Landmark)
    (noaction_person_location_constraint ?na - NoAction ?p - Person ?lmp - Landmark)
    (call_person_location_constraint ?a - CallAction ?p - Person ?loc - Landmark)
    (call_not_person_location_constraint ?a - CallAction ?p - Person ?loc - Landmark)
    (call_person_not_taking_medicine_constraint ?a - CallAction ?p - Person)
    (call_person_not_eating_food_constraint ?a - CallAction ?p - Person)

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
    :parameters (?t - Time ?from - Landmark ?to - Landmark ?tn - Time)
    :precondition (and
        (current_time ?t)
        (next_time ?t ?tn)
        (robot_at ?from)
        (traversable ?from ?to)
        (not (abort))
        (not (wait_completed ?t)) ;; Ensure Wait must follow Move
    )
    :effect (and
        (robot_at ?to)
        (not (robot_at ?from))
        (moved_after_landmark ?tn)
        (not (wait_completed ?t)) ;; Block other actions until Wait executes

        ;; ✅ NEW: Enforce waiting after movement
        (waiting_required ?t)

        (forall (?tn - Time)
            (when (next_time ?t ?tn)
                (and (not (current_time ?t)) (current_time ?tn) (robot_at_time ?tn ?to))
            )
        )
    )
)








;; Make a call action with enforced waiting
(:action MakeCall
    :parameters (?a - CallAction ?t - Time ?p - Person ?m - Msg)
    :precondition (and
        (MakeCall_enabled)
        (current_time ?t)

        (not (used_reminder ?t))
        (not (executed_call ?a))
        (valid_call_message ?a ?m)
        (wait_completed ?t)

        ;; Enforce that the person didn't take medicine
        (not (and (call_person_not_taking_medicine_constraint ?a ?p)
                  (not (not (person_taking_medicine ?t)) ) ) )

        ;; Enforce that the person didn't eat food
        (not (and (call_person_not_eating_food_constraint ?a ?p)
                  (not (not (person_eating_food ?t)) ) ) )

        ;; Ensure blocked calls must be executed first
        (forall (?ai - CallAction)
            (not (and (call_blocks_call ?ai ?a)
                      (not (executed_call ?ai)) ) ) )

        (forall (?ai - ReminderAction)
            (not (and (reminder_blocks_call ?ai ?a)
                      (not (executed_reminder ?ai)) ) ) )

        ;; Ensure robot and person are at the same location
        (same_location_constraint)
        (not
            (forall (?loc - Landmark)
                (not (and (person_at ?t ?p ?loc) (robot_at ?loc)) )
            )
        )

        (not (abort))
    )
    :effect (and
        (message_given ?m)
        (executed_call ?a)

        ;; ✅ Advances time
        (forall (?tn - Time)
            (when (next_time ?t ?tn)
                (and (not (current_time ?t)) (current_time ?tn)) ) )

        (used_reminder ?t)
    )
)



;;give reminder
(:action GiveReminder
    :parameters (?a - ReminderAction ?t - Time ?p - Person ?m - Msg)
    :precondition (and
        (GiveReminder_enabled)
        (current_time ?t)
        (not (used_reminder ?t))
        (valid_reminder_message ?a ?m)
        (not (executed_reminder ?a))
        (wait_completed ?t) ;; Ensure Wait must have executed

        ;; ✅ NEW: Ensure Wait has fully completed before executing GiveReminder
        (not (waiting_required ?t))

        ;; Ensure person is not taking medicine
        (not (and (reminder_person_not_taking_medicine_constraint ?a ?p)
                  (not (not (person_taking_medicine ?t))) ) )

        ;; Ensure person is not eating food
        (not (and (reminder_person_not_eating_food_constraint ?a ?p)
                  (not (not (person_eating_food ?t))) ) )

        ;; Block reminders if required
        (forall (?ai - ReminderAction)
            (not (and (reminder_blocks_reminder ?ai ?a)
                      (not (executed_reminder ?ai)) ) ) )

        ;; Ensure robot and person are at the same location
        (same_location_constraint)
        (not
            (forall (?loc - Landmark)
                (not (and (person_at ?t ?p ?loc) (robot_at ?loc))) )
        )

        (not (abort))
    )
    :effect (and
        (message_given ?m)
        (executed_reminder ?a)

        ;; ✅ Advances time
        (forall (?tn - Time)
            (when (next_time ?t ?tn)
                (and (not (current_time ?t)) (current_time ?tn)) ) )

        (used_reminder ?t)
    )
)




;; Wait for timestep
(:action Wait
  :parameters (?a - WaitAction ?t - Time ?tn - Time)
  :precondition (and
    (current_time ?t)
    (next_time ?t ?tn) ;; Ensure time progression
    (moved_after_landmark ?t)  ;; Ensures Wait is only considered after movement
    (not (executed_wait ?a))
    (not (abort))

    ;; ✅ NEW: Enforce that Wait must be the next action
    (waiting_required ?t)

    ;; Enforce that nothing else (reminder/call) happens before waiting
    (forall (?r - ReminderAction)
        (not (executed_reminder ?r))
    )
    (forall (?c - CallAction)
        (not (executed_call ?c))
    )
  )
  :effect (and
    (executed_wait ?a)
    (wait_completed ?t)

    ;; ✅ NEW: Mark that waiting is done, allowing further actions
    (not (waiting_required ?t))

    ;; Ensure time advances after waiting
    (forall (?tn - Time)
      (when (next_time ?t ?tn)
        (and (not (current_time ?t)) (current_time ?tn))
      )
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