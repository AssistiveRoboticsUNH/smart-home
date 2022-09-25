(define (domain shr)

(:requirements :strips :typing :fluents :durative-actions)

(:types
	landmark 
	robot
	message
)

(:predicates
	(robot_at ?v - robot ?lm - landmark)
	(notified ?msg - message)
	(message_at ?msg - message ?lm -landmark)
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

)
