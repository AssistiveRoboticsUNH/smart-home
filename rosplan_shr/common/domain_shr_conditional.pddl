(define (domain shr_contingent)

(:requirements :strips :typing :disjunctive-preconditions)

(:types
	landmark 
	robot
	message
	sensor
)

(:predicates
	(robot_at ?v - robot ?lm - landmark)
	(is_home ?lm - landmark)
	(notified ?msg - message)
	(message_at ?msg - message ?lm -landmark)
	(is_on ?ss - sensor)
	(is_off ?ss - sensor)
	(available_to_check_s ?ss - sensor)
	(sensor_after_notified ?ss -sensor ?msg - message)
	(is_safe)
	(is_not_safe)
)

;; Move to any landmark, avoiding terrain
(:action moveto_landmark
	:parameters (?v - robot ?from ?to - landmark)
	:precondition (robot_at ?v ?from)
	:effect (and
		(robot_at ?v ?to)
		(not (robot_at ?v ?from)))
)

;; Notify message at landmark
(:action notifyAt
	:parameters (?v - robot ?lm - landmark ?msg - message)
	:precondition (and
	        (robot_at ?v ?lm)
	        (message_at ?msg ?lm))
	:effect (and
	       (forall (?ss - sensor) (when (sensor_after_notified ?ss ?msg) (available_to_check_s ?ss)))
	       (notified ?msg))
)

;; check if sensor ss is on
(:action check_sensor_on
	:parameters (?ss - sensor)
	:precondition (available_to_check_s ?ss)
	:observe (is_on ?ss)
)

;; check if sensor ss is off
;; I guest this is the way to do it since 
;; it only allow true precondtion actions
(:action check_sensor_off
	:parameters (?ss - sensor)
	:precondition (available_to_check_s ?ss)
	:observe (is_off ?ss)
)

;; call caregiver and play message msg if sensor ss is on
(:action call_caregiver
	:parameters (?ss - sensor ?msg - message)
	:precondition (is_on ?ss)
	:effect (and
	        (is_safe)
	        (not (is_not_safe))
	        (notified ?msg))
)

;; go home from landmark lm if sensor ss is off
(:action go_home_from
	:parameters (?v - robot ?from - landmark ?ss - sensor)
	:precondition (and 
				(robot_at ?v ?from)
				(is_off ?ss))
	:effect (and
	        (is_safe)
	        (not (is_not_safe))
		    (not (robot_at ?v ?from))
	        (forall (?lm - landmark) (when (is_home ?lm) (robot_at ?v ?lm))))
)

)
