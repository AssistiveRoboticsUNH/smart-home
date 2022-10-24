(define (domain paul_shr_conditional)

(:requirements :strips :typing :disjunctive-preconditions)

(:types
	person
	robot
	landmark
	automated_message
	recorded_message
)

(:predicates
	(robot_at ?r - robot ?lm - landmark)
	(person_at ?p - person ?lm - landmark)
	(person_at_5minutes ?p - person ?lm - landmark)

	(give_message_location ?lm - landmark)
	(door_location ?lm - landmark)
	(outside_location ?lm - landmark)

	(recorded_message_given ?msg - recorded_message)
	(automated_message_given ?msg - automated_message)
	(asked_caregiver_help ?p - person)
	(alerted_caregiver_wondering ?p - person)
	(alerted_emergency_wondering ?p - person)

	(guide_to_succeeded_attempt_1)
	(guide_to_succeeded_attempt_2)
)


(:action SensePersonLoc
    :parameters (?r - robot ?p - person ?loc - landmark)
    :precondition (robot_at ?r ?loc)
    :observe (person_at ?p)
)


(:action SensePersonLoc
    :parameters (?r - robot ?p - person ?loc - landmark)
    :precondition (robot_at ?r ?loc)
    :observe (person_at ?p)
)

;; wait 5 minutes before taking an action
(:action SensePersonAtLoc5Mins
	:parameters (?p - person ?loc - landmark)
	:precondition (person_at ?p ?loc)
	:observe (person_at_5minutes ?p ?loc)
)

;; Move to any landmark, avoiding terrain
(:action moveToLandmark
	:parameters (?r - robot ?from ?to - landmark)
	:precondition (robot_at ?r ?from)
	:effect (robot_at ?r ?to)
)

;; Guide person from one landmark to another
(:action guidePersonToLandmarkAttempt1
	:parameters (?r - robot ?p - person ?from ?to - landmark)
	:precondition (and
                        (robot_at ?r ?from)
                        (person_at ?p ?from)
                   )
	:effect (robot_at ?r ?to)
    :observe (guide_to_succeeded_attempt_1)
)
;; Guide person from one landmark to another
(:action guidePersonToLandmarkAttempt2
	:parameters (?r - robot ?p - person ?from ?to - landmark)
	:precondition (and
	                    (not (guide_to_succeeded_attempt_1))
                        (robot_at ?r ?from)
                        (person_at ?p ?from)
                   )
	:effect (robot_at ?r ?to)
    :observe (guide_to_succeeded_attempt_2)
)

;; Notify message at landmark
(:action notifyAutomatedAt
	:parameters (?r - robot ?p - person ?loc - landmark ?msg - automated_message)
	:precondition  (and
                        (robot_at ?r ?loc)
                        (person_at ?p ?loc)
                        (give_message_location ?loc)
               	   )
	:effect (automated_message_given ?msg)
)

;; Notify message at landmark
(:action notifyRecordedAt
	:parameters (?r - robot ?p - person ?loc - landmark ?msg1 - automated_message ?msg2 - recorded_message)
	:precondition (and
                        (robot_at ?r ?loc)
                        (person_at ?p ?loc)
                        (give_message_location ?loc)
                        (automated_message_given ?msg1)
               	   )
	:effect (recorded_message_given ?msg2)
)

;; Notify message at landmark
(:action alertEmergencyWondering
	:parameters (?p - person ?loc - landmark)
	:precondition (and
                    (alerted_caregiver ?p)
                    (person_at ?p ?loc)
                    (outside_location ?loc)
                    (person_at_5minutes ?p ?loc)
                  )
	:effect (alerted_emergency_wondering ?p - person)
)

;; alert care giver of wondering
(:action alertCaregiverWondering
	:parameters (?p - person ?msg - recorded_message)
	:precondition (and
                      (person_at ?p ?loc)
                      (outside_location ?loc)
                  )
	:effect (alerted_caregiver_wondering ?p)
)

;; ask for caregiver to convince person to do something
(:action askCaregiverHelp
	:parameters (?p - person ?msg - recorded_message)
	:precondition (recorded_message_given ?msg)
	:effect (asked_caregiver_help ?p)
)

)
