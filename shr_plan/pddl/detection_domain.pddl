(define domain detection_trial)

(:requirements :strips :typing)


(:types
	person
	robot
	landmark
)

(:predicates
    (robot_at ?r - robot ?lm - landmark)
    (person_at ?p - person ?lm - landmark)

    (init_move_to_landmark)

    (success)
)

(:action detectPerson
    :parameters (?r - robot ?p - person ?loc - landmark)
    :precondition (and
    			(robot_at ?r ?loc)
			(not (init_move_to_landmark))
			(not (init_guide_person_to_landmark_attempt))
   		 )
    :observe (person_at ?p ?loc)
)

;; Init move
(:action initMoveToLandmark
	:parameters (?r - robot)
	:precondition (and
			(not (init_move_to_landmark))
			(not (init_guide_person_to_landmark_attempt))
		      )
	  :effect (and
		      (forall (?loc - landmark)
			  (not (robot_at ?r ?loc))
		      )
		      (init_move_to_landmark)
  	        )
)

;; Move to any landmark, avoiding terrain
(:action moveToLandmark
	:parameters (?r - robot ?to - landmark)
	:precondition (and
			(init_move_to_landmark)
			(not (init_guide_person_to_landmark_attempt))
		      )
	:effect (and
                (robot_at ?r ?to)
                (not (enable_check_guide_1))
                (not (enable_check_guide_2))
		(not (init_move_to_landmark))
            )
)


;; Notify message at landmark
(:action notifyAutomatedMidnightAt
	:parameters (?r - robot ?p - person ?loc - landmark)
	:precondition  (and
                        (robot_at ?r ?loc)
                        (person_at ?p ?loc)
                        (not (tried_notify_automated))
                        (not (init_detect_person_left_house_1))
                        (not (init_detect_person_left_house_2))
                        (not (init_check_bed_after_return))
	        	(not (init_move_to_landmark))
               	   )
	:effect (tried_notify_automated)
)