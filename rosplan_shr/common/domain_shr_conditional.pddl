(define (domain shr_contingent)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	landmark 
	robot
	message
	sensor
)

(:predicates
	(robot_at ?v - robot ?lm - landmark)
	(notified ?msg - message)
	(message_at ?msg - message ?lm -landmark)
	(is_on ?ss - sensor)
	(is_off ?ss - sensor)
	(available_to_check_s ?ss - sensor)
	(sensor_after_notified ?ss -sensor ?msg - message)
)

;; Move to any landmark, avoiding terrain
(:durative-action moveto_landmark
	:parameters (?v - robot ?from ?to - landmark)
	:duration ( = ?duration 60)
	:condition (at start (robot_at ?v ?from))
	:effect (and
		(at end (robot_at ?v ?to))
		(at start (not (robot_at ?v ?from))))
)

;; Notify message at landmark
(:durative-action notifyAt
	:parameters (?v - robot ?lm - landmark ?msg - message)
	:duration ( = ?duration 60)
	:condition (and
	        (at start (robot_at ?v ?lm))
	        (at start (message_at ?msg ?lm)))
	:effect (and
	       (at end (forall (?ss - sensor) (when (sensor_after_notified ?ss ?msg) (available_to_check_s ?ss))))
	       (at end (notified ?msg)))
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
	:effect (notified ?msg)
)

)
