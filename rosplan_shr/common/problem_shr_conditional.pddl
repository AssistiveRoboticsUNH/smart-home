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
    (message_at midnight_warning door)
    (sensor_after_notified doorss midnight_warning )
)
(:goal (or
    (is_off doorss)
    (notified leaving_house)
))
)
