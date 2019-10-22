(define (domain shr_conditional_medication)

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
	(phonemessage_about_sensor ?msg - phonemessage ?ss - sensor)
	(phonemessage_about_missing ?msg)
	(is_on ?ss - sensor)
	(is_off ?ss - sensor)
	(is_safe_when_on ?ss - sensor)
	(is_safe_when_off ?ss - sensor)
	(available_to_check_s ?ss - sensor)
	(sensor_after_notified ?ss -sensor ?msg - message)
	(person_is_approached)
	(person_is_not_approached)
	(is_safe)
	(is_not_safe)
)

;; search and appraoch person success branch
(:action search_and_approach_person_success
	:observe (person_is_approached)
)

;; search and appraoch person fail branch
(:action search_and_approach_person_fail
	:observe (person_is_not_approached)
)

;; Notify message if person is approached
(:action notify
	:parameters (?msg - message)
	:precondition (person_is_approached)
	:effect (and
	       (forall (?ss - sensor) (when 
		           (sensor_after_notified ?ss ?msg) 
				   (available_to_check_s ?ss)))
	       (notified ?msg))
)

;; check if sensor ss is on
(:action check_sensor_on
	:parameters (?ss - sensor)
	:precondition (available_to_check_s ?ss)
	:observe (is_on ?ss)
)

;; check if sensor ss is off
(:action check_sensor_off
	:parameters (?ss - sensor)
	:precondition (available_to_check_s ?ss)
	:observe (is_off ?ss)
)

;; call caregiver and play message msg if sensor ss is off
(:action call_caregiver_when_sensor_off
	:parameters (?ss - sensor ?msg - phonemessage)
	:precondition  (and
	        (is_off ?ss)
	        (is_safe_when_on ?ss)
	        (phonemessage_about_sensor ?msg ?ss))
	:effect (and
	        (is_safe)
	        (not (is_not_safe)))
)

;; call caregiver and play message msg if sensor ss is off
(:action call_caregiver_when_person_is_not_located
	:parameters (?msg - phonemessage)
	:precondition  (and
	        (phonemessage_about_missing ?msg)
			(person_is_not_approached))
	:effect (and
	        (is_safe)
	        (not (is_not_safe)))
)

;; go home from landmark lm if sensor ss is on
(:action go_home_when_sensor_on
	:parameters (?ss - sensor)
	:precondition (and 
	            (is_safe_when_on ?ss)
				(is_on ?ss))
	:effect (and
	        (is_safe)
	        (not (is_not_safe))
	        (forall (?lm - landmark) (when (is_home ?lm) (robot_at ?lm))))
)

)
