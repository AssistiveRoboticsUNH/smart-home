(define (domain shr_contingent_wildcard_plugged)

(:requirements :strips :typing :disjunctive-preconditions)

(:types
	landmark 
	message
	phonemessage
	sensor
)

(:predicates
	(robot_at ?lm - landmark)
	(is_home ?lm - landmark)
	(notified ?msg - message)
	(message_at ?msg - message ?lm - landmark)
	(phonemessage_about_sensor ?msg - phonemessage ?ss - sensor)
	(is_on ?ss - sensor)
	(is_off ?ss - sensor)
	(is_safe_when_on ?ss - sensor)
	(is_safe_when_off ?ss - sensor)
	(available_to_check_s ?ss - sensor)
	(sensor_after_notified ?ss -sensor ?msg - message)
	(is_safe)
	(is_not_safe)
)

;; Move to any landmark, avoiding terrain
(:action moveto_landmark
	:parameters (?from ?to - landmark)
	:precondition (robot_at ?from)
	:effect (and
		(robot_at ?to)
		(not (robot_at ?from)))
)

;; Notify message at landmark
(:action notifyAt
	:parameters (?lm - landmark ?msg - message)
	:precondition (and
	        (robot_at ?lm)
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
(:action call_caregiver_when_sensor_on_at
	:parameters (?ss - sensor ?msg - phonemessage ?lm - landmark)
	:precondition  (and
	        (is_on ?ss)
	        (is_safe_when_off ?ss)
	        (robot_at ?lm)
	        (phonemessage_about_sensor ?msg ?ss))
	:effect (and
	        (is_safe)
	        (not (is_not_safe)))
)

;; call caregiver and play message msg if sensor ss is off
(:action call_caregiver_when_sensor_off_at
	:parameters (?ss - sensor ?msg - phonemessage ?lm - landmark)
	:precondition  (and
	        (is_off ?ss)
	        (is_safe_when_on ?ss)
	        (robot_at ?lm)
	        (phonemessage_about_sensor ?msg ?ss))
	:effect (and
	        (is_safe)
	        (not (is_not_safe)))
)

;; go home from landmark lm if sensor ss is off
(:action go_home_from
	:parameters (?from - landmark ?ss - sensor)
	:precondition (and 
				(robot_at ?from)
				(not (is_home ?from))
	            (is_safe_when_off ?ss)
				(is_off ?ss))
	:effect (and
	        (is_safe)
	        (not (is_not_safe))
		    (not (robot_at ?from))
	        (forall (?lm - landmark) (when (is_home ?lm) (robot_at ?lm))))
)

;; go home from landmark lm if sensor ss is on
(:action go_home_from
	:parameters (?from - landmark ?ss - sensor)
	:precondition (and 
				(robot_at ?from)
				(not (is_home ?from))
	            (is_safe_when_on ?ss)
				(is_on ?ss))
	:effect (and
	        (is_safe)
	        (not (is_not_safe))
		    (not (robot_at ?from))
	        (forall (?lm - landmark) (when (is_home ?lm) (robot_at ?lm))))
)

)
