(define (problem task)
(:domain shr)
(:objects
    door home - landmark
    pioneer - robot
    midnightmsg - message
)
(:init
    (robot_at pioneer home)


    (message_at midnightmsg door)

)
(:goal (and
    (robot_at pioneer door)
    (notified midnightmsg)
))
)
