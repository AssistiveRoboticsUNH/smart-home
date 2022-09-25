(define (problem task)
(:domain shr)
(:objects
    door home - landmark
    pioneer - robot
    midnight_warning - message
)
(:init
    (robot_at pioneer home)


    (message_at midnight_warning door)

)
(:goal (and
    (robot_at pioneer door)
    (notified midnight_warning)
))
)
