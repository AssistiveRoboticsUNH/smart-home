(define (problem task_conditional_medication_enhanced)
(:domain shr_conditional_medication_enhanced)
(:objects
    door kitchen bedroom home - landmark
    medication_msg bottle_msg - message
    medication_not_taken people_missing medication_not_found - phonemessage
    medication_sensor - sensor
)
(:init
    (robot_at home)
    (is_home home)
    (phonemessage_about_sensor medication_not_taken medication_sensor)
    (phonemessage_about_missing people_missing)
    (phonemessage_about_bottle medication_not_found)
    (msg_about_medication medication_msg)
    (msg_about_bottle bottle_msg)
    (sensor_after_notified medication_sensor medication_msg)
    (is_safe_when_on medication_sensor)
    (unknown (is_on medication_sensor))
    (unknown (is_off medication_sensor))
	(oneof
		(is_on medication_sensor)
		(is_off medication_sensor)
	)
    (unknown (person_is_approached))
    (unknown (person_is_not_approached))
	(oneof
		(person_is_approached)
		(person_is_not_approached)
	)
    (unknown (bottle_is_found))
    (unknown (bottle_is_not_found))
	(oneof
		(bottle_is_found)
		(bottle_is_not_found)
	)
    (is_not_safe)
)
(:goal (is_safe)
)
)
