(define (domain shr_contigent_medication_enhanced_instant)

(:types
)

(:predicates
	(available_to_find)
	(bottle_is_found)
	(bottle_is_not_found)
)

(:action find_bottle_succ
	:precondition (available_to_find)
	:observe (bottle_is_found)
)

(:action find_bottle_fail
	:precondition (available_to_find)
    :observe (bottle_is_not_found)
)

(:action notifyBottle
	:parameters (?msg - message)
	:precondition (and
	              (bottle_is_found)
	              (msg_about_bottle ?msg))
	:effect (and 
	         (notified ?msg)
	         (forall (?ss - sensor) 
			         (available_to_check_s ?ss)))
)

(:action call_caregiver_when_medication_is_not_found
	:parameters (?msg - phonemessage)
	:precondition  (and
	        (phonemessage_about_bottle ?msg)
			(bottle_is_not_found))
	:effect (and
	        (is_safe)
	        (not (is_not_safe)))
)
)
