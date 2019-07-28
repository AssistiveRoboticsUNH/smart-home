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
	:effect (at end (notified ?msg))
)

;; events triggered by notifying message 
(:event enableSensorAfterPlayedMsg
	:parameters (?ss - sensor ?msg - message)
	:precondition (and
	        (sensor_after_notified ?ss ?msg)
	        (notified ?msg ))
	:effect (available_to_check_s ?ss)
)

;; check sensor 
(:action check_sensor
	:parameters (?ss - sensor)
	:precondition (available_to_check_s ?ss)
	:observe (is_on ?ss)
)

)
