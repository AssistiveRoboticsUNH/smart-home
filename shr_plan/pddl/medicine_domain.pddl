(define (domain medicine_protocol)

(:requirements :strips :typing)

(:types
	Person
	Robot
	Landmark
	MedicineProtocol
)

(:predicates
    (medicine_protocol_enabled ?m - MedicineProtocol)
    (robot_at ?r - Robot ?lm - Landmark)
    (person_at ?p - Person ?lm - Landmark)
    (medicine_location ?m - MedicineProtocol ?lm - Landmark)

    (asked_caregiver_help ?m - MedicineProtocol ?p - Person)

    (init_move_to_landmark ?m - MedicineProtocol)
    (init_guide_person_to_landmark_attempt ?m - MedicineProtocol)

    (guide_to_succeeded_attempt_1 ?m - MedicineProtocol ?lm - Landmark)
    (guide_to_succeeded_attempt_2 ?m - MedicineProtocol ?lm - Landmark)
    (notify_automated_succeeded ?m - MedicineProtocol)
    (notify_recorded_succeeded ?m - MedicineProtocol)

    (tried_guide_person_landmark_1 ?m - MedicineProtocol)
    (tried_guide_person_landmark_2 ?m - MedicineProtocol)

    (enable_check_guide_1 ?m - MedicineProtocol)
    (enable_check_guide_2 ?m - MedicineProtocol)

    (already_took_medicine ?m - MedicineProtocol)
    (already_called_about_medicine ?m - MedicineProtocol)
    (success)

)

(:action detectPerson
    :parameters (?m - MedicineProtocol ?r - Robot ?p - Person ?loc - Landmark)
    :precondition (and
          (medicine_protocol_enabled ?m)
    			(robot_at ?r ?loc)
			(not (init_move_to_landmark ?m))
			(not (init_guide_person_to_landmark_attempt ?m))
   		 )
    :observe (person_at ?p ?loc)
)

;; Init move
(:action initMoveToLandmark
	:parameters (?m - MedicineProtocol ?r - Robot)
	:precondition (and
          (medicine_protocol_enabled ?m)
			(not (init_move_to_landmark ?m))
			(not (init_guide_person_to_landmark_attempt ?m))
		      )
	  :effect (and
		      (forall (?loc - landmark)
			  (not (robot_at ?r ?loc))
		      )
		      (init_move_to_landmark ?m)
  	        )
)

;; Move to any landmark, avoiding terrain
(:action moveToLandmark
	:parameters (?m - MedicineProtocol ?r - Robot ?to - Landmark)
	:precondition (and
          (medicine_protocol_enabled ?m)
			(init_move_to_landmark ?m)
			(not (init_guide_person_to_landmark_attempt ?m))
		      )
	:effect (and
                (robot_at ?r ?to)
                (not (enable_check_guide_1 ?m))
                (not (enable_check_guide_2 ?m))
		(not (init_move_to_landmark ?m))
            )
)


 ;; Init Guide
(:action InitguidePersonToLandmarkAttempt
	:parameters (?m - MedicineProtocol ?r - Robot ?p - Person ?to - Landmark)
	:precondition (and
          (medicine_protocol_enabled ?m)
			(robot_at ?r ?to)
			(person_at ?p ?to)
			(not (init_move_to_landmark ?m))
			(not (init_guide_person_to_landmark_attempt ?m))
		      )
	  :effect (and
		      (forall (?loc - Landmark)
			    (not (robot_at ?r ?loc))
		      )
		      (init_guide_person_to_landmark_attempt ?m)
  	        )
)


;; Guide person from one landmark to another
(:action guidePersonToLandmarkAttempt1
	:parameters (?m - MedicineProtocol ?r - Robot ?p - Person ?to - Landmark)
	:precondition (and
          (medicine_protocol_enabled ?m)
	                (not (tried_guide_person_landmark_1 ?m))
                        (medicine_location ?m ?to)
                        (not (init_move_to_landmark ?m))
			(init_guide_person_to_landmark_attempt ?m)
                   )
    :effect (and
                (robot_at ?r ?to)
                (tried_guide_person_landmark_1 ?m)
                (enable_check_guide_1 ?m)
		(not (init_guide_person_to_landmark_attempt ?m))
            )
)

;; Guide person from one landmark to another
(:action guidePersonToLandmarkAttempt2
	:parameters (?m - MedicineProtocol ?r - Robot ?p - Person ?to - Landmark)
	:precondition (and
          (medicine_protocol_enabled ?m)
                        (tried_guide_person_landmark_1 ?m)
                        (not (tried_guide_person_landmark_2 ?m))
                        (medicine_location ?m ?to)
                        (not (init_move_to_landmark ?m))
			(init_guide_person_to_landmark_attempt ?m)
                   )
    :effect (and
                (robot_at ?r ?to)
                (tried_guide_person_landmark_2 ?m)
                (enable_check_guide_2 ?m)
		(not (init_guide_person_to_landmark_attempt ?m))
            )
)

;; Notify message at landmark
(:action checkGuideToSucceeded1
	:parameters (?m - MedicineProtocol ?loc - Landmark)
	:precondition  (and
		            (tried_guide_person_landmark_1 ?m)
		            (enable_check_guide_1 ?m)
			    (not (init_move_to_landmark ?m))
		            (not (init_guide_person_to_landmark_attempt ?m))
	                )
	:observe (guide_to_succeeded_attempt_1 ?m ?loc)
)
;; Notify message at landmark
(:action checkGuideToSucceeded2
	:parameters (?m - MedicineProtocol ?loc - Landmark)
	:precondition  (and
	                    (tried_guide_person_landmark_2 ?m)
	                    (enable_check_guide_2 ?m)
	                    (not (init_move_to_landmark ?m))
			    (not (init_guide_person_to_landmark_attempt ?m))
	                )
	:observe (guide_to_succeeded_attempt_2 ?m ?loc)
)



;; Update person location
(:action UpdatePersonLoc1
	:parameters (?m - MedicineProtocol ?p - Person ?from ?to - Landmark)
	:precondition (and
          (medicine_protocol_enabled ?m)
	                (guide_to_succeeded_attempt_1 ?m)
	                (person_at ?p ?from)
	                (medicine_location ?m ?to)
                        (not (init_move_to_landmark ?m))
		        (not (init_guide_person_to_landmark_attempt ?m))
	               )
    :effect ( and
                (not (person_at ?p ?from))
                (person_at ?p ?to)
            )
)

;; Update person location
(:action UpdatePersonLoc2
	:parameters (?m - MedicineProtocol ?p - Person ?from ?to - Landmark)
	:precondition (and
          (medicine_protocol_enabled ?m)
			(guide_to_succeeded_attempt_2 ?m)
			(person_at ?p ?from)
			(medicine_location ?m ?to)
			(not (init_move_to_landmark ?m))
			(not (init_guide_person_to_landmark_attempt ?m))
                   )
	:effect ( and
                (not (person_at ?p ?from))
                (person_at ?p ?to)
            )
)

;; Update success status
(:action UpdateSuccess1
	:parameters (?m - MedicineProtocol)
	:precondition (and
          (medicine_protocol_enabled ?m)
			(notify_automated_succeeded ?m)
			(not (init_move_to_landmark ?m))
			(not (init_guide_person_to_landmark_attempt ?m))
		)
    :effect (and (success) (already_took_medicine ?m) (not (medicine_protocol_enabled ?m)) )
)
;; Update success status
(:action UpdateSuccess2
	:parameters (?m - MedicineProtocol)
	:precondition (and
          (medicine_protocol_enabled ?m)
			(notify_recorded_succeeded ?m)
			(not (init_move_to_landmark ?m))
			(not (init_guide_person_to_landmark_attempt ?m))
		)
	:effect (and (success) (already_took_medicine ?m) (not (medicine_protocol_enabled ?m)))
)
;; Update success status
(:action UpdateSuccess3
	:parameters (?m - MedicineProtocol ?p - Person)
	:precondition (and
          (medicine_protocol_enabled ?m)
			(asked_caregiver_help ?m ?p)
			(not (init_move_to_landmark ?m))
			(not (init_guide_person_to_landmark_attempt ?m))
		)
	:effect (and (success) (already_called_about_medicine ?m) (not (medicine_protocol_enabled ?m)))
)

;; Notify message at landmark
(:action notifyAutomatedMedicineAt
	:parameters (?m - MedicineProtocol ?r - Robot ?p - Person ?loc - Landmark)
	:precondition  (and
                        (robot_at ?r ?loc)
                        (person_at ?p ?loc)
                        (medicine_location ?m ?loc)
			(not (init_move_to_landmark ?m))
			(not (init_guide_person_to_landmark_attempt ?m))
               	   )
	:observe (notify_automated_succeeded ?m)
)

;; Notify message at landmark
(:action notifyRecordedMedicineAt
	:parameters (?m - MedicineProtocol ?r - Robot ?p - Person ?loc - Landmark)
	:precondition (and
          (medicine_protocol_enabled ?m)
		        (not (notify_automated_succeeded ?m))
		        (robot_at ?r ?loc)
		        (person_at ?p ?loc)
		        (medicine_location ?m ?loc)
			(not (init_move_to_landmark ?m))
			(not (init_guide_person_to_landmark_attempt ?m))
               	   )
	:observe (notify_recorded_succeeded ?m)
)


;; ask for caregiver to convince person to do something
(:action askCaregiverHelpMedicine1
	:parameters (?m - MedicineProtocol ?r - Robot ?p - Person ?loc - Landmark)
	:precondition (and
          (medicine_protocol_enabled ?m)
			(not (notify_automated_succeeded ?m))
			(not (notify_recorded_succeeded ?m))
			(robot_at ?r ?loc)
			(person_at ?p ?loc)
			(not (init_move_to_landmark ?m))
			(not (init_guide_person_to_landmark_attempt ?m))
                   )
	:effect (asked_caregiver_help  ?m ?p)
)

;; ask for caregiver to convince person to do something
(:action askCaregiverHelpMedicine2
	:parameters (?m - MedicineProtocol ?r - Robot ?p - Person ?loc - Landmark)
	:precondition (and
          (medicine_protocol_enabled ?m)
			(not (guide_to_succeeded_attempt_1 ?m))
			(not (guide_to_succeeded_attempt_2 ?m))
			(robot_at ?r ?loc)
			(person_at ?p ?loc)
			(not (init_move_to_landmark ?m))
			(not (init_guide_person_to_landmark_attempt ?m))
                   )
	:effect (asked_caregiver_help ?m ?p)
)


)