(define (domain shr_conditional_medication_enhanced)

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
	(phonemessage_about_bottle ?msg)
	(msg_about_medication ?msg)
	(msg_about_bottle ?msg)
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
	(available_to_find)
	(bottle_is_found)
	(bottle_is_not_found)
)

;; search and appraoch person success branch
(:action search_and_approach_person_success
	:observe (person_is_approached)
)

;; search and appraoch person fail branch
(:action search_and_approach_person_fail
	:observe (person_is_not_approached)
)

;; Notify message if person is approach
(:action notifyMedication
	:parameters (?msg - message)
	:precondition (and
	              (person_is_approached)
				  (not (available_to_find))
	              (msg_about_medication ?msg))
	:effect (and
	       (forall (?ss - sensor) (when (sensor_after_notified ?ss ?msg) (available_to_check_s ?ss)))
	       (notified ?msg))
)

;; Notify message if found medication bottle
(:action notifyBottle
	:parameters (?msg - message)
	:precondition (and
	              (bottle_is_found)
	              (msg_about_bottle ?msg))
	:effect (and 
	         (notified ?msg)
	         (forall (?ss - sensor) (available_to_check_s ?ss)))
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

;; macro action 1 
(:action marco_action_1
	:precondition (forall (?ss - sensor) (is_off ?ss))
    :effect (and
	        (forall (?ss - sensor) (not (is_on ?ss)))
	        (forall (?ss - sensor) (not (is_off ?ss)))
	        (forall (?ss - sensor) (not (available_to_check_s ?ss)))
		    (available_to_find))
)

;; find object
(:action find_bottle_succ
	:precondition (available_to_find)
	:observe (bottle_is_found)
)

(:action find_bottle_fail
	:precondition (available_to_find)
    :observe (bottle_is_not_found)
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

;; call caregiver and play message msg if sensor ss is off
(:action call_caregiver_when_sensor_off
	:parameters (?ss - sensor ?msg - phonemessage)
	:precondition  (and
	        (is_off ?ss)
	        (is_safe_when_on ?ss)
	        (phonemessage_about_sensor ?msg ?ss)
			(bottle_is_found))
	:effect (and
	        (is_safe)
	        (not (is_not_safe)))
)

;; call caregiver and play message msg if bottle is not found
(:action call_caregiver_when_medication_is_not_found
	:parameters (?msg - phonemessage)
	:precondition  (and
	        (phonemessage_about_bottle ?msg)
			(bottle_is_not_found))
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
