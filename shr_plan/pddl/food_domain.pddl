(define
	(domain food)
	(:requirements :strips :typing)
	(:types
		landmark
		person
		robot
	)
	(:predicates
	    (robot_at ?r - robot ?loc - landmark)
	    (person_at ?p - person ?loc - landmark)
	    (food_location ?loc - landmark)

		(asked_caregiver_help ?p - person)
		(robot_updated_1 )
        (robot_updated_2 )

        (init_guide_person_to_landmark_attempt )
        (init_move_to_landmark )

        (guide_to_succeeded_attempt_1 )
        (guide_to_succeeded_attempt_2 )
        (remind_food_succeeded )
        (remind_food_succeeded2)

        (tried_guide_person_landmark_1 )
        (tried_guide_person_landmark_2 )

		(enable_check_guide_1 )
		(enable_check_guide_2 )

		(success)
)

	(:action InitguidePersonToLandmarkAttempt
		:parameters (?r - robot ?p - person ?to - landmark)
		:precondition  ( and 
				(robot_at ?r ?to)
				(person_at ?p ?to)
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
	  :effect (and
		      (forall (?loc - landmark)
			  (not (robot_at ?r ?loc))
		      )
		      (init_guide_person_to_landmark_attempt)
  	        )

	)
	(:action UpdatePersonLoc1
		:parameters (?p - person ?from - landmark ?to - landmark)
		:precondition  ( and 
				(guide_to_succeeded_attempt_1 )
				(person_at ?p ?from)
				(food_location ?to)
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:effect  ( and 
				(not (person_at ?p ?from))
				(person_at ?p ?to)
			)
	)
	(:action UpdatePersonLoc2
		:parameters (?p - person ?from - landmark ?to - landmark)
		:precondition  ( and 
				(guide_to_succeeded_attempt_2 )
				(person_at ?p ?from)
				(food_location ?to)
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:effect  ( and 
				(not (person_at ?p ?from))
				(person_at ?p ?to)
			)
	)
	(:action UpdateSuccess1
		:parameters ()
		:precondition  ( and 
				(remind_food_succeeded )
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:effect (success )

	)
	(:action UpdateSuccess2
		:parameters ()
		:precondition  ( and 
				(not (remind_food_succeeded ))
				(remind_food_succeeded2 )
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:effect (success )

	)
	(:action UpdateSuccess3
		:parameters (?p - person)
		:precondition  ( and 
				(asked_caregiver_help ?p)
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:effect (success )

	)
	(:action askCaregiverHelpFood1
		:parameters (?r - robot ?p - person ?loc - landmark)
		:precondition  ( and 
				(not (remind_food_succeeded ))
				(not (remind_food_succeeded2 ))
				(robot_at ?r ?loc)
				(person_at ?p ?loc)
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:effect (asked_caregiver_help ?p)

	)
	(:action askCaregiverHelpFood2
		:parameters (?r - robot ?p - person ?loc - landmark)
		:precondition  ( and 
				(not (guide_to_succeeded_attempt_1 ))
				(not (guide_to_succeeded_attempt_2 ))
				(robot_at ?r ?loc)
				(person_at ?p ?loc)
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:effect (asked_caregiver_help ?p)

	)
	(:action checkGuideToSucceeded1
		:parameters (?loc - landmark)
		:precondition  ( and 
				(tried_guide_person_landmark_1 )
				(enable_check_guide_1 )
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:observe (guide_to_succeeded_attempt_1 )
	)
	(:action checkGuideToSucceeded2
		:parameters (?loc - landmark)
		:precondition  ( and 
				(tried_guide_person_landmark_2 )
				(enable_check_guide_2 )
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:observe (guide_to_succeeded_attempt_2 )
	)
	(:action detectPerson
		:parameters (?r - robot ?p - person ?loc - landmark)
		:precondition  ( and 
				(robot_at ?r ?loc)
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:observe (person_at ?p ?loc)
	)
	(:action guidePersonToLandmarkAttempt1
		:parameters (?r - robot ?p - person ?to - landmark)
		:precondition  ( and 
				(not (tried_guide_person_landmark_1 ))
				(food_location ?to)
				(not (init_move_to_landmark ))
				(init_guide_person_to_landmark_attempt )
			)
		:effect  ( and 
				(robot_at ?r ?to)
				(tried_guide_person_landmark_1 )
				(enable_check_guide_1 )
				(not (init_guide_person_to_landmark_attempt ))
			)
	)
	(:action guidePersonToLandmarkAttempt2
		:parameters (?r - robot ?p - person ?to - landmark)
		:precondition  ( and 
				(tried_guide_person_landmark_1 )
				(not (tried_guide_person_landmark_2 ))
				(food_location ?to)
				(not (init_move_to_landmark ))
				(init_guide_person_to_landmark_attempt )
			)
		:effect  ( and 
				(robot_at ?r ?to)
				(tried_guide_person_landmark_2 )
				(enable_check_guide_2 )
				(not (init_guide_person_to_landmark_attempt ))
			)
	)
	(:action initMoveToLandmark
		:parameters (?r - robot)
		:precondition  ( and 
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
	  :effect (and
		      (forall (?loc - landmark)
			  (not (robot_at ?r ?loc))
		      )
		      (init_move_to_landmark)
  	        )

	)
	(:action moveToLandmark
		:parameters (?r - robot ?to - landmark)
		:precondition  ( and 
				(init_move_to_landmark )
				(not (init_guide_person_to_landmark_attempt ))
			)
		:effect  ( and 
				(robot_at ?r ?to)
				(not (enable_check_guide_1 ))
				(not (enable_check_guide_2 ))
				(not (init_move_to_landmark ))
			)
	)
	(:action remindAutomatedFoodAt
		:parameters (?r - robot ?p - person ?loc - landmark)
		:precondition  ( and 
				(robot_at ?r ?loc)
				(person_at ?p ?loc)
				(food_location ?loc)
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:observe (remind_food_succeeded )
	)
	(:action remindAutomatedFoodAt2
		:parameters (?r - robot ?p - person ?loc - landmark)
		:precondition  ( and 
				(not (remind_food_succeeded ))
				(robot_at ?r ?loc)
				(person_at ?p ?loc)
				(food_location ?loc)
				(not (init_move_to_landmark ))
				(not (init_guide_person_to_landmark_attempt ))
			)
		:observe (remind_food_succeeded2 )
	)
)