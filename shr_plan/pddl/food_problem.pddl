(define
	(problem food)
	(:domain food)
	(:objects
		kitchen couch home - landmark
		nathan - person
		pioneer - robot
	)
	(:init 
	(robot_at pioneer home)
	(food_location kitchen)
	(unknown 	(person_at nathan couch))
	(unknown 	(person_at nathan kitchen))
	(unknown 	(person_at nathan home))
	(unknown 	(guide_to_succeeded_attempt_1))
	(unknown 	(guide_to_succeeded_attempt_2))
	(unknown 	(remind_food_succeeded))
	(unknown 	(remind_food_succeeded2))
	(oneof 		(person_at nathan couch) 	(person_at nathan kitchen) 	(person_at nathan home) )
	)

	(:goal (success))
)
