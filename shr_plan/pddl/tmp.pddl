(define (domain shr_domain)

(:requirements :strips :typing)

(:types
	Person
	LandmarkPerson
    LandmarkRobot
	Msg
	Time
	ReminderAction
)

(:predicates
    ;; physical modeling
    (robot_at ?lmr - LandmarkRobot)
    (person_at ?t - Time ?p - Person ?lmp - LandmarkPerson)
    
    ;; physical constants
    (traversable ?from ?to - LandmarkRobot)

    ;; control flow
    (abort)

    ;; effects of actions
    (message_given ?m - Msg)

    ;; success conditions
    (message_given_success ?m - Msg)

    ;; enable/disable actions
    (GiveReminder_enabled)
    (DetectPerson_enabled)

    ;; enforce action sequence dependencies
    (executed_reminder ?a - ReminderAction)

    ;; enforce that actions are called with valid object instances
    (valid_reminder_message ?a - ReminderAction ?m - Msg)

    ;; time management predicates
    (time_critical)
    (used_move ?tc - Time)
    (current_time ?tc - Time)
    (next_time ?tc ?tn - Time)

    ;; constraints on the state of the world. object instances here refer to non-input instances
    (reminder_robot_location_constraint ?a - ReminderAction ?lmr - LandmarkRobot)
    (reminder_person_location_constraint ?a - ReminderAction ?p - Person ?lmp - LandmarkPerson)

    (success)
)

;; detect if person is at location outside or inside or bedroom
(:action DetectPersonLocation
    :parameters (?t - Time ?p - Person ??lmp - LandmarkPerson)
    :precondition (and
                    (current_time ?t)
                    (DetectPerson_enabled)
                    (not (abort))
	                )
    :observe (person_at ?t ?p ?lmp)
)

;;give reminder
(:action GiveReminder
    :parameters (?a - ReminderAction ?t - Time ?p - Person ?m - Msg ?lmr - LandmarkRobot ?lmp - LandmarkPerson)
    :precondition (and
            (GiveReminder_enabled)
            (current_time ?t)
            (valid_reminder_message ?a ?m)
            (not (executed_reminder ?a))

            ;; certain things must be true about the world state for the specific action instance
            ;; this condition enforces that the robot is at the location specified in person_location_constraint
            (forall (?lmr - LandmarkRobot)
             (not (and (not (robot_at ?lmr)) (reminder_robot_location_constraint ?a ?lmr) ) )
            )
            ;; this condition enforces that the person is at the location specified in person_location_constraint
            (forall (?lmp - LandmarkPerson)
              (not (and (not (person_at ?t ?p ?lmp)) (reminder_person_location_constraint ?a ?p ?lmp) ) )
            )
            
            ;; this condition enforces that the person is not at the location specified in not_person_location_constraint
            ;;(forall (?lmr - LandmarkRobot)
            ;;  (not (and (robot_at ?lmr) (reminder_not_robot_location_constraint ?a ?lmr) ) )
            )

            (not (abort))
		)
    :effect (and (message_given ?m)  (executed_reminder ?a)
              (forall (?tn - Time)
                (when (next_time ?t ?tn) (and (not (current_time ?t)) (current_time ?tn)) )
              )
            )
)

;; Move to any landmark, avoiding terrain
(:action MoveToLandmark
	:parameters (?t - Time ?from - LandmarkRobot ?to - LandmarkRobot)
	:precondition (and
	                (current_time ?t)
	                (not (used_move ?t))
	                (robot_at ?from)
	                (traversable ?from ?to)
	                (not (abort))
	          )
	:effect (and (robot_at ?to) (not (robot_at ?from)) (used_move ?t)
	          (when (time_critical)
              (forall (?tn - Time)
                (when (next_time ?t ?tn) (and (not (current_time ?t)) (current_time ?tn)) )
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

)