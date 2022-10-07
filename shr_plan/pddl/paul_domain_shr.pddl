(define (domain paul_shr)

(:requirements :strips :typing :fluents :durative-actions)

(:types
	person
	robot
	landmark
	automated_message
	recorded_message
)

(:predicates
	(robot_at ?v - robot ?lm - landmark)
	(person_at ?v - person ?lm - landmark)
	(give_message_location ?lm - landmark)
	(person_outside ?v - person)
	(recorded_message_given)
	(automated_message_given)
	(talked_with_caregiver ?v - person)
	(alerted_caregiver)
	(is_safe)
)

;; Move to any landmark, avoiding terrain
(:durative-action moveto_landmark
	:parameters (?v - robot ?from ?to - landmark)
	:duration ( = ?duration 60)
	:condition (at start (robot_at ?v ?from))
	:effect (and
		(at start (not (robot_at ?v ?from))))
		(at end (robot_at ?v ?to))
)

;; Guide person from one landmark to another
(:durative-action guideto_landmark
	:parameters (?r - robot ?p - person ?from ?to - landmark)
	:duration ( = ?duration 60)
	:condition (at start (and
	    (robot_at ?r ?from)
        (person_at ?p ?from)
	))
	:effect (and
        (at start (and
            (not (robot_at ?v ?from))
            (not (person_at ?v ?from))
        )))
		(at end (and
            (robot_at ?r ?to)
            (person_at ?r ?to)
		))
)

;; Notify message at landmark
(:durative-action notifyAutomatedAt
	:parameters (?r - robot ?p - person ?loc - landmark ?msg - automated_message)
	:duration ( = ?duration 60)
	:condition (at start (and
               	    (robot_at ?r ?loc)
                    (person_at ?p ?loc)
                    (give_message_location ?loc)
               	))
	:effect (at end (automated_message_given))
)

;; Notify message at landmark
(:durative-action notifyRecordedAt
	:parameters (?r - robot ?p - person ?loc - landmark ?msg - recorded_message)
	:duration ( = ?duration 60)
	:condition (at start (and
               	    (robot_at ?r ?loc)
                    (person_at ?p ?loc)
                    (give_message_location ?loc)
                    (automated_message_given)
               	))
	:effect (at end (recorded_message_given)
	)
)

;; Notify message at landmark
(:durative-action callEmergency
	:parameters (?p - person)
	:duration ( = ?duration 60)
	:condition (at start (and
        (alerted_caregiver)
        (person_outside ?p)
	))
	:effect (at end (is_safe))
)

;; Notify message at landmark
(:durative-action alertCaregiver
	:parameters (?r - robot)
	:duration ( = ?duration 60)
	:condition (at start (recorded_message_given))
	:effect (at end (alerted_caregiver ?p))
)

;; Notify message at landmark
(:durative-action callCaregiver
	:parameters (?r - robot ?p - person ?loc - landmark)
	:duration ( = ?duration 60)
	:condition (at start (and
	                (recorded_message_given)
               	    (robot_at ?r ?loc)
                    (person_at ?p ?loc)
               	))
	:effect (at end (talked_with_caregiver ?p))
)

)
