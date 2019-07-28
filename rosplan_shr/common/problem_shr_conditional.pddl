(define (problem task_conditional)
(:domain shr_contingent)
(:objects
    door home - landmark
    pioneer - robot
    midnight_warning - message
    leaving_house - message
    doorss - sensor
)
(:init
    (robot_at pioneer home)
    (is_home home)
    (message_at midnight_warning door)
    (sensor_after_notified doorss midnight_warning)
    (unknown (is_on doorss))
    (unknown (is_off doorss))
	(oneof
		(is_on doorss)
		(is_off doorss)
	)
    (is_not_safe)
)
(:goal (is_safe)
)
)
