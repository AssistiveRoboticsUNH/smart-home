(define (problem detection_trial)
(:domain detection_trial)

(:objects
    door unknown home - landmark
    pioneer - robot
    nathan - person
)

(:init
    (robot_at pioneer home)
    (unknown (person_at nathan door))
    (unknown (person_at nathan unknown))
    )

(:goal (success)
)
)

