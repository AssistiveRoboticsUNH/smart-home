(define (problem midnight_wondering)
(:domain midnight_wondering_domain)
(:objects
    kitchen couch home door - landmark
    pioneer - robot
    nathan - person
)
(:init
    (robot_at pioneer home)
    (unknown (person_at nathan door))
    (door_location door)
    (unknown (person_decides_to_go_outside_1))
    (unknown (person_decides_to_go_outside_2))
    (unknown (person_decides_to_return_1))
    (unknown (person_decides_to_return_2))
    (unknown (person_decides_to_go_to_bed_1))
    (unknown (person_decides_to_go_to_bed_2))
)
(:goal (success)
)
)