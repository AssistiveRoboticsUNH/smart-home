(define
	(domain food_protocol)
	(:requirements :strips :typing)
	(:types
		Landmark
		Person
		Robot
	    FoodProtocol
	)
	(:predicates
	    (enabled ?f - FoodProtocol)

	    (robot_at ?r - Robot ?loc - Landmark)
	    (person_at ?p - Person ?loc - Landmark)
	    (food_location ?f - FoodProtocol ?loc - Landmark)

		(asked_caregiver_help ?f - FoodProtocol ?p - Person)

        (init_guide_person_to_landmark_attempt ?f - FoodProtocol)
        (init_move_to_landmark ?f - FoodProtocol)

        (guide_to_succeeded_attempt_1 ?f - FoodProtocol)
        (guide_to_succeeded_attempt_2 ?f - FoodProtocol)
        (remind_food_succeeded ?f - FoodProtocol)
        (remind_food_succeeded2 ?f - FoodProtocol)

        (tried_guide_person_landmark_1 ?f - FoodProtocol)
        (tried_guide_person_landmark_2 ?f - FoodProtocol)

		(enable_check_guide_1 ?f - FoodProtocol)
		(enable_check_guide_2 ?f - FoodProtocol)

		(already_ate  ?f - FoodProtocol)
        (already_called_about_eating  ?f - FoodProtocol)

		(success)
)

	(:action InitguidePersonToLandmarkAttempt
		:parameters (?f - FoodProtocol ?r - Robot ?p - Person ?to - Landmark)
		:precondition (and
		        (enabled ?f)
				(robot_at ?r ?to)
				(person_at ?p ?to)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
	  :effect (and
		      (forall (?loc - Landmark)
			  (not (robot_at ?r ?loc))
		      )
		      (init_guide_person_to_landmark_attempt ?f)
  	        )

	)
	(:action UpdatePersonLoc1
		:parameters (?f - FoodProtocol ?p - Person ?from - Landmark ?to - Landmark)
		:precondition  (and
		        (enabled ?f)
				(guide_to_succeeded_attempt_1 ?f)
				(person_at ?p ?from)
				(food_location ?f ?to)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:effect  ( and 
				(not (person_at ?p ?from))
				(person_at ?p ?to)
			)
	)
	(:action UpdatePersonLoc2
		:parameters (?f - FoodProtocol ?p - Person ?from - Landmark ?to - Landmark)
		:precondition  (and
                (enabled ?f)
				(guide_to_succeeded_attempt_2 ?f)
				(person_at ?p ?from)
				(food_location ?f ?to)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:effect  ( and 
				(not (person_at ?p ?from))
				(person_at ?p ?to)
			)
	)
	(:action UpdateSuccess1
		:parameters (?f - FoodProtocol )
		:precondition  (and
                (enabled ?f)
				(remind_food_succeeded ?f)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:effect (and (success) (already_ate ?f) (not (enabled ?f)))

	)
	(:action UpdateSuccess2
		:parameters (?f - FoodProtocol )
		:precondition  (and
                (enabled ?f)
				(not (remind_food_succeeded ?f))
				(remind_food_succeeded2 ?f)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:effect (and (success) (already_ate ?f) (not (enabled ?f)))

	)
	(:action UpdateSuccess3
		:parameters (?f - FoodProtocol ?p - Person)
		:precondition  (and
                (enabled ?f)
				(asked_caregiver_help  ?f ?p)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:effect (and (success) (already_called_about_eating ?f) (not (enabled ?f)))

	)
	(:action askCaregiverHelpFood1
		:parameters (?f - FoodProtocol ?r - Robot ?p - Person ?loc - Landmark)
		:precondition  (and
                (enabled ?f)
				(not (remind_food_succeeded ?f))
				(not (remind_food_succeeded2 ?f))
				(robot_at ?r ?loc)
				(person_at ?p ?loc)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:effect (asked_caregiver_help  ?f ?p)

	)
	(:action askCaregiverHelpFood2
		:parameters (?f - FoodProtocol ?r - Robot ?p - Person ?loc - Landmark)
		:precondition  (and
                (enabled ?f)
				(not (guide_to_succeeded_attempt_1 ?f))
				(not (guide_to_succeeded_attempt_2 ?f))
				(robot_at ?r ?loc)
				(person_at ?p ?loc)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:effect (asked_caregiver_help ?f ?p)

	)
	(:action checkGuideToSucceeded1
		:parameters (?f - FoodProtocol ?loc - Landmark)
		:precondition  (and
                (enabled ?f)
				(tried_guide_person_landmark_1 ?f)
				(enable_check_guide_1 ?f)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:observe (guide_to_succeeded_attempt_1 ?f)
	)
	(:action checkGuideToSucceeded2
		:parameters (?f - FoodProtocol ?loc - Landmark)
		:precondition  (and
                (enabled ?f)
				(tried_guide_person_landmark_2 ?f)
				(enable_check_guide_2 ?f)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:observe (guide_to_succeeded_attempt_2 ?f)
	)
	(:action detectPerson
		:parameters (?f - FoodProtocol ?r - Robot ?p - Person ?loc - Landmark)
		:precondition  (and
                (enabled ?f)
				(robot_at ?r ?loc)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:observe (person_at ?p ?loc)
	)
	(:action guidePersonToLandmarkAttempt1
		:parameters (?f - FoodProtocol ?r - Robot ?p - Person ?to - Landmark)
		:precondition  (and
                (enabled ?f)
				(not (tried_guide_person_landmark_1 ?f))
				(food_location  ?f ?to)
				(not (init_move_to_landmark ?f))
				(init_guide_person_to_landmark_attempt ?f)
			)
		:effect  ( and 
				(robot_at ?r ?to)
				(tried_guide_person_landmark_1 ?f)
				(enable_check_guide_1 ?f)
				(not (init_guide_person_to_landmark_attempt ?f))
			)
	)
	(:action guidePersonToLandmarkAttempt2
		:parameters (?f - FoodProtocol ?r - Robot ?p - Person ?to - Landmark)
		:precondition  (and
                (enabled ?f)
				(tried_guide_person_landmark_1 ?f)
				(not (tried_guide_person_landmark_2 ?f))
				(food_location ?f ?to)
				(not (init_move_to_landmark ?f))
				(init_guide_person_to_landmark_attempt ?f)
			)
		:effect  ( and 
				(robot_at ?r ?to)
				(tried_guide_person_landmark_2 ?f)
				(enable_check_guide_2 ?f)
				(not (init_guide_person_to_landmark_attempt ?f))
			)
	)
	(:action initMoveToLandmark
		:parameters (?f - FoodProtocol ?r - Robot)
		:precondition  (and
                (enabled ?f)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
	  :effect (and
		      (forall (?loc - Landmark)
			  (not (robot_at ?r ?loc))
		      )
		      (init_move_to_landmark ?f)
  	        )

	)
	(:action moveToLandmark
		:parameters (?f - FoodProtocol ?r - Robot ?to - Landmark)
		:precondition  (and
                (enabled ?f)
				(init_move_to_landmark ?f)
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:effect  ( and 
				(robot_at ?r ?to)
				(not (enable_check_guide_1 ?f))
				(not (enable_check_guide_2 ?f))
				(not (init_move_to_landmark ?f))
			)
	)
	(:action remindAutomatedFoodAt
		:parameters (?f - FoodProtocol ?r - Robot ?p - Person ?loc - Landmark)
		:precondition  (and
                (enabled ?f)
				(robot_at ?r ?loc)
				(person_at ?p ?loc)
				(food_location ?f ?loc)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:observe (remind_food_succeeded ?f)
	)
	(:action remindAutomatedFoodAt2
		:parameters (?f - FoodProtocol ?r - Robot ?p - Person ?loc - Landmark)
		:precondition  (and
                (enabled ?f)
				(not (remind_food_succeeded ?f))
				(robot_at ?r ?loc)
				(person_at ?p ?loc)
				(food_location ?f ?loc)
				(not (init_move_to_landmark ?f))
				(not (init_guide_person_to_landmark_attempt ?f))
			)
		:observe (remind_food_succeeded2 ?f)
	)
)