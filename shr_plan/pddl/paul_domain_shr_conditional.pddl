(define (domain paul_shr_conditional)

(:requirements :strips :typing :disjunctive-preconditions)

(:types
	person
	robot
	landmark
)

(:predicates
	(robot_at ?r - robot ?lm - landmark)
	(person_at ?p - person ?lm - landmark)
	(medicine_location ?lm - landmark)

	(asked_caregiver_help ?p - person)
	(robot_updated_1)
	(robot_updated_2)

	(guide_to_succeeded_attempt_1)
	(guide_to_succeeded_attempt_2)
	(notify_automated_succeeded)
	(notify_recorded_succeeded)

	(tried_guide_person_landmark_1)
	(tried_guide_person_landmark_2)

    (enable_check_guide_1)
    (enable_check_guide_2)

	(success)

)

(:action detect_person
    :parameters (?r - robot ?p - person ?loc - landmark)
    :precondition (and (robot_at ?r ?loc))
    :observe (person_at ?p ?loc)
)

;; Move to any landmark, avoiding terrain
(:action moveToLandmark
	:parameters (?r - robot ?from ?to - landmark)
	:precondition (robot_at ?r ?from)
	:effect (and
                (not (robot_at ?r ?from))
                (robot_at ?r ?to)
                (not (enable_check_guide_1))
                (not (enable_check_guide_2))
            )
)


;; Guide person from one landmark to another
(:action guidePersonToLandmarkAttempt1
	:parameters (?r - robot ?p - person ?from ?to - landmark)
	:precondition (and
	                    (not (tried_guide_person_landmark_1))
                        (robot_at ?r ?from)
                        (person_at ?p ?from)
                        (medicine_location ?to)
                   )
    :effect (and
                (not (robot_at ?r ?from))
                (robot_at ?r ?to)
                (tried_guide_person_landmark_1)
                (enable_check_guide_1)
            )
)

;; Guide person from one landmark to another
(:action guidePersonToLandmarkAttempt2
	:parameters (?r - robot ?p - person ?from ?to - landmark)
	:precondition (and
                        (tried_guide_person_landmark_1)
                        (not (tried_guide_person_landmark_2))
                        (robot_at ?r ?from)
                        (person_at ?p ?from)
                        (medicine_location ?to)
                   )
    :effect (and
                (not (robot_at ?r ?from))
                (robot_at ?r ?to)
                (tried_guide_person_landmark_2)
                (enable_check_guide_2)
            )
)

;; Notify message at landmark
(:action checkGuideToSucceeded1
	:parameters ()
	:precondition  (and
	                    (tried_guide_person_landmark_1)
	                    (enable_check_guide_1)
	                )
	:observe (guide_to_succeeded_attempt_1)
)
;; Notify message at landmark
(:action checkGuideToSucceeded2
	:parameters (?r - robot ?loc - landmark)
	:precondition  (and
	                    (tried_guide_person_landmark_2)
	                    (enable_check_guide_2)
	                )
	:observe (guide_to_succeeded_attempt_2)
)



;; Update person location
(:action UpdatePersonLoc1
	:parameters (?p - person ?from ?to - landmark)
	:precondition (and
	                (guide_to_succeeded_attempt_1)
	                (person_at ?p ?from)
	                (medicine_location ?to)
	               )
    :effect ( and
                (not (person_at ?p ?from))
                (person_at ?p ?to)
            )
)

;; Update person location
(:action UpdatePersonLoc2
	:parameters (?p - person ?from ?to - landmark)
	:precondition (and
                    (guide_to_succeeded_attempt_2)
	                (person_at ?p ?from)
	                (medicine_location ?to)
                   )
	:effect ( and
                (not (person_at ?p ?from))
                (person_at ?p ?to)
            )
)

;; Update success status
(:action UpdateSuccess1
	:parameters ()
	:precondition (notify_automated_succeeded)
    :effect (success)
)
;; Update success status
(:action UpdateSuccess2
	:parameters ()
	:precondition (notify_recorded_succeeded)
	:effect (success)
)
;; Update success status
(:action UpdateSuccess3
	:parameters (?p - person)
	:precondition (asked_caregiver_help ?p)
	:effect (success)
)

;; Notify message at landmark
(:action notifyAutomatedMedicineAt
	:parameters (?r - robot ?p - person ?loc - landmark)
	:precondition  (and
                        (robot_at ?r ?loc)
                        (person_at ?p ?loc)
                        (medicine_location ?loc)
               	   )
	:observe (notify_automated_succeeded)
)

;; Notify message at landmark
(:action notifyRecordedMedicineAt
	:parameters (?r - robot ?p - person ?loc - landmark)
	:precondition (and
	                    (not (notify_automated_succeeded))
                        (robot_at ?r ?loc)
                        (person_at ?p ?loc)
                        (medicine_location ?loc)
               	   )
	:observe (notify_recorded_succeeded)
)


;; ask for caregiver to convince person to do something
(:action askCaregiverHelpMedicine1
	:parameters (?r - robot ?p - person ?loc - landmark)
	:precondition (and
                    (not (notify_automated_succeeded))
                    (not (notify_recorded_succeeded))
                    (robot_at ?r ?loc)
                    (person_at ?p ?loc)
                   )
	:effect (asked_caregiver_help ?p)
)

;; ask for caregiver to convince person to do something
(:action askCaregiverHelpMedicine2
	:parameters (?r - robot ?p - person ?loc - landmark)
	:precondition (and
                    (not (guide_to_succeeded_attempt_1))
                    (not (guide_to_succeeded_attempt_2))
                    (robot_at ?r ?loc)
                    (person_at ?p ?loc)
                   )
	:effect (asked_caregiver_help ?p)
)


)
