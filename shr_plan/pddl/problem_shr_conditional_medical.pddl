(define (problem task_conditional_medical)
(:domain shr_contingent)
(:objects
    door kitchen bedroom home - landmark
    medicine_robot_msg - message
    medicine_phone_msg - phonemessage
    mediciness - sensor
)
(:init
    (robot_at home)
    (is_home home)
    (message_at medicine_robot_msg kitchen)
    (phonemessage_about_sensor medicine_phone_msg mediciness)
    (sensor_after_notified mediciness medicine_robot_msg)
    (is_safe_when_on mediciness)
    (unknown (is_on mediciness))
    (unknown (is_off mediciness))
	(oneof
		(is_on mediciness)
		(is_off mediciness)
	)
    (is_not_safe)
)
(:goal (is_safe)
)
)
