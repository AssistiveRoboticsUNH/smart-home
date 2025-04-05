(define (domain high_level_domain)

(:requirements
  :strips
  :typing
)

(:types
  FoodProtocol
  MedicineProtocol
  ExerciseReminderProtocol
  MoveReminderProtocol
  InternalCheckReminderProtocol
  PracticeReminderProtocol
  Landmark
  Time
  Person
)

(:predicates
  (started)

  (robot_at ?lmr - Landmark)
  (person_at ?t - Time ?p - Person ?lmp - Landmark)
  (person_currently_at ?p - Person ?lmp - Landmark)
  (visible_location ?lmp - Landmark)

  (food_protocol_enabled ?f - FoodProtocol)
  (medicine_reminder_enabled ?med - MedicineProtocol)
  (exercise_reminder_enabled ?ex - ExerciseReminderProtocol)
  (move_reminder_enabled ?mv - MoveReminderProtocol)
  (internal_check_reminder_enabled ?ic - InternalCheckReminderProtocol)
  (practice_reminder_enabled ?pra - PracticeReminderProtocol)

  (time_to_eat ?f - FoodProtocol)
  (already_ate  ?f - FoodProtocol)
  (already_called_about_eating  ?f - FoodProtocol)

  ;; medicine
  (time_to_take_medicine ?med - MedicineProtocol)
  (already_took_medicine ?m - MedicineProtocol)
  (already_reminded_medicine ?m - MedicineProtocol)

  ;; exercise reminder
  (time_for_exercise_reminder ?ex - ExerciseReminderProtocol)
  (already_reminded_exercise ?ex - ExerciseReminderProtocol)

  ;; move reminder
  (time_for_move_reminder ?mv - MoveReminderProtocol)
  (already_reminded_move ?mv - MoveReminderProtocol)

  ;; internal check reminder
  (time_for_internal_check_reminder ?ic - InternalCheckReminderProtocol)
  (already_reminded_internal_check ?ic - InternalCheckReminderProtocol)

  ;; practice reminder
  (time_for_practice_reminder ?pra - PracticeReminderProtocol)
  (already_reminded_practice ?pra - PracticeReminderProtocol)

  ;; priority
  (priority_1)
  (priority_2)
  (priority_3)
  (priority_4)
  (priority_5)

  (low_level_failed)

  (success)
)

(:action MoveToLandmark
	:parameters (?from - Landmark ?to - Landmark)
	:precondition (and
	                (robot_at ?from)
	                (started)
	          )
	:effect (and (robot_at ?to) (not (robot_at ?from)) )
)


(:action ChangePriority_1_2
	:parameters ()
	:precondition (and
	    (priority_1)
		)
	:effect (and (priority_2) (not (priority_1)))
)
(:action ChangePriority_2_3
	:parameters ()
	:precondition (and
	    (priority_2)
		)
	:effect (and (priority_3) (not (priority_2)))
)
(:action ChangePriority_3_4
	:parameters ()
	:precondition (and
	    (priority_3)
		)
	:effect (and (priority_4) (not (priority_3)))
)
(:action ChangePriority_4_5
	:parameters ()
	:precondition (and
	    (priority_4)
		)
	:effect (and (priority_5) (not (priority_4)))
)

;; to start ros and navigation before the protocol
(:action StartROS
	:parameters ()
	:precondition (;;and
	       ;; will be triggered before it starts a protocol
           ;; (priority_2)
		)
	:effect (and
	            ;;(not (priority_2))
                (started)
          )
)

(:action StartMedReminderProtocol
	:parameters (?m - MedicineProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
	    (priority_1)
      (time_to_take_medicine ?m)
      (not (already_took_medicine ?m))
      (not (already_reminded_medicine ?m))
      (person_currently_at ?p ?lmp)
      (robot_at ?lmp)
      (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )

      ;; person in visible area
      (visible_location ?lmp)
      (started)
    )
	:effect (and
	          (success)
	          (not (priority_1))
	          (medicine_reminder_enabled ?m)
	          (not (low_level_failed))
              (forall (?internal - InternalCheckReminderProtocol) (not (internal_check_reminder_enabled ?internal)) )
              (forall (?practice - PracticeReminderProtocol) (not (practice_reminder_enabled ?practice)) )
              (forall (?mv - MoveReminderProtocol) (not (move_reminder_enabled ?mv)) )
              (forall (?ex - ExerciseReminderProtocol) (not (exercise_reminder_enabled ?ex)) )
              (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )

          )
)

(:action ContinueMedReminderProtocol
	:parameters (?m - MedicineProtocol)
	:precondition (and
	    (priority_1)
	    (not (low_level_failed))
      (medicine_reminder_enabled ?m)
      (not (already_took_medicine ?m))
      (not (already_reminded_medicine ?m))
      (time_to_take_medicine ?m)
    )
	:effect (and (success) (not (priority_1)) )
)

(:action StartFoodProtocol
	:parameters (?f - FoodProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
	    (priority_2)
      (time_to_eat ?f)
      (person_currently_at ?p ?lmp)
      (robot_at ?lmp)
      (visible_location ?lmp)
      (not (already_ate ?f))
      (not (already_called_about_eating ?f))
      (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )
	  (started)
	)
	:effect (and
	          (success)
            (not (priority_2))
            (food_protocol_enabled ?f)
            (not (low_level_failed))
            (forall (?internal - InternalCheckReminderProtocol) (not (internal_check_reminder_enabled ?internal)) )
            (forall (?practice - PracticeReminderProtocol) (not (practice_reminder_enabled ?practice)) )
            (forall (?mv - MoveReminderProtocol) (not (move_reminder_enabled ?mv)) )
            (forall (?ex - ExerciseReminderProtocol) (not (exercise_reminder_enabled ?ex)) )
            (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )

          )
)
(:action ContinueFoodProtocol
	:parameters (?f - FoodProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))
      (time_to_eat ?f)
      (not (already_ate ?f))
      (not (already_called_about_eating ?f))
      (food_protocol_enabled ?f)
		)
	:effect (and (success) (not (priority_2)) )
)

;; exercise reminder Protocol
(:action StartExerciseReminderProtocol
	:parameters (?ex - ExerciseReminderProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
	  (priority_2)
      (time_for_exercise_reminder ?ex)
      (not (already_reminded_exercise ?ex))
      (person_currently_at ?p ?lmp)
      (robot_at ?lmp)
      (forall (?ex - ExerciseReminderProtocol) (not (exercise_reminder_enabled ?ex)) )

      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)
      (started)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (exercise_reminder_enabled ?ex)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )
              (forall (?internal - InternalCheckReminderProtocol) (not (internal_check_reminder_enabled ?internal)) )
              (forall (?practice - PracticeReminderProtocol) (not (practice_reminder_enabled ?practice)) )
              (forall (?mv - MoveReminderProtocol) (not (move_reminder_enabled ?mv)) )
              (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )

          )
)

(:action ContinueExerciseReminderProtocol
	:parameters (?ex - ExerciseReminderProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))

      (not (already_reminded_exercise ?ex))
      (exercise_reminder_enabled ?ex)
      (time_for_exercise_reminder ?ex)
    )
	:effect (and (success) (not (priority_2)) )
)

;; Move reminder Protocol
(:action StartMoveReminderProtocol
	:parameters (?mv - MoveReminderProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
	    (priority_2)
      (time_for_move_reminder ?mv)
      (not (already_reminded_move ?mv))
      (forall (?mv - MoveReminderProtocol) (not (move_reminder_enabled ?mv)) )
      (person_currently_at ?p ?lmp)
      (robot_at ?lmp)
      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)
      (started)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (move_reminder_enabled ?mv)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )
              (forall (?internal - InternalCheckReminderProtocol) (not (internal_check_reminder_enabled ?internal)) )
              (forall (?practice - PracticeReminderProtocol) (not (practice_reminder_enabled ?practice)) )
              (forall (?ex - ExerciseReminderProtocol) (not (exercise_reminder_enabled ?ex)) )
              (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )

          )
)

(:action ContinueMoveReminderProtocol
	:parameters (?mv - MoveReminderProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))

      (not (already_reminded_move ?mv))
      (move_reminder_enabled ?mv)
      (time_for_move_reminder ?mv)
    )
	:effect (and (success) (not (priority_2)) )
)


;; Internal Check Reminder Protocol

(:action StartInternalCheckReminderProtocol
	:parameters (?ic - InternalCheckReminderProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
	    (priority_2)
      (time_for_internal_check_reminder ?ic)
      (not (already_reminded_internal_check ?ic))
      (forall (?ic - InternalCheckReminderProtocol) (not (internal_check_reminder_enabled ?ic)) )
      (person_currently_at ?p ?lmp)
      (robot_at ?lmp)
      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)
      (started)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (internal_check_reminder_enabled ?ic)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )
              (forall (?move - MoveReminderProtocol) (not (move_reminder_enabled ?move)) )
              (forall (?practice - PracticeReminderProtocol) (not (practice_reminder_enabled ?practice)) )
              (forall (?ex - ExerciseReminderProtocol) (not (exercise_reminder_enabled ?ex)) )
              (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )

          )
)

(:action ContinueInternalCheckReminderProtocol
	:parameters (?ic - InternalCheckReminderProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))

      (not (already_reminded_internal_check ?ic))
      (internal_check_reminder_enabled ?ic)
      (time_for_internal_check_reminder ?ic)
    )
	:effect (and (success) (not (priority_2)) )
)

;; Practice Reminder Protocol

(:action StartPracticeReminderProtocol
	:parameters (?pra - PracticeReminderProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
	    (priority_2)
      (time_for_practice_reminder ?pra)
      (not (already_reminded_practice ?pra))
      (forall (?pra - PracticeReminderProtocol) (not (practice_reminder_enabled ?pra)) )
      (person_currently_at ?p ?lmp)
      (robot_at ?lmp)
      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)
      (started)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (practice_reminder_enabled ?pra)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )
              (forall (?move - MoveReminderProtocol) (not (move_reminder_enabled ?move)) )
              (forall (?internal - InternalCheckReminderProtocol) (not (internal_check_reminder_enabled ?internal)) )
              (forall (?ex - ExerciseReminderProtocol) (not (exercise_reminder_enabled ?ex)) )
              (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )

          )
)

(:action ContinuePracticeReminderProtocol
	:parameters (?pra - PracticeReminderProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))

      (not (already_reminded_practice ?pra))
      (practice_reminder_enabled ?pra)
      (time_for_practice_reminder ?pra)
    )
	:effect (and (success) (not (priority_2)) )
)

(:action Idle
	:parameters ()
	:precondition (and
	    (priority_5)
		)
	:effect (and (success)
	              (not (priority_5))
                (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )
                (forall (?internal - InternalCheckReminderProtocol) (not (internal_check_reminder_enabled ?internal)) )
                (forall (?practice - PracticeReminderProtocol) (not (practice_reminder_enabled ?practice)) )
                (forall (?move - MoveReminderProtocol) (not (move_reminder_enabled ?move)) )
                (forall (?ex - ExerciseReminderProtocol) (not (exercise_reminder_enabled ?ex)) )
                (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )
                (not (low_level_failed))
          )
)

;; shutdown is supposed to stop ros2 processes
;; it should try to dock if it is not docked
;; triggered when there should be protocol and it has been done

(:action Shutdown
	:parameters ()
	:precondition
	    (and
	        (started)
	        ;; has to be higher priority than idle
            (priority_4)

            ;; CANT SHUTDOWN IF time to do something is true and
            ;; all predicates indicating that they it is done are false
            ;; give F in such case

            ;; need to add a forall for every protocol objects in problem
            ;;; 1
            ;;; forall would give false if one is F
            (forall (?med - MedicineProtocol)
                (not
                    (and
                        (time_to_take_medicine ?med)
                        (not (already_took_medicine ?med))
                        (not (already_reminded_medicine ?med))
                    )
                )
            )
            ;;; 2
            (forall (?internal - InternalCheckReminderProtocol)
                (not
                    (and
                        (time_for_internal_check_reminder ?internal)
                        (not (already_reminded_internal_check ?internal))
                    )
                )
            )
            ;;; 3
            (forall (?mv - MoveReminderProtocol)
                (not
                    (and
                        (time_for_move_reminder ?mv)
                        (not (already_reminded_move ?mv))
                    )
                )
            )
            ;;; 4
            (forall (?ex - ExerciseReminderProtocol)
                (not
                    (and
                        (time_for_exercise_reminder ?ex)
                        (not (already_reminded_exercise ?ex))
                    )
                )
            )

            ;;; 5
            (forall (?practice - PracticeReminderProtocol)
                (not
                    (and
                        (time_for_practice_reminder ?practice)
                        (not (already_reminded_practice ?practice))
                    )
                )
            )

            ;;; 6
            (forall (?food - FoodProtocol)
                (not
                    (and
                        (time_to_eat ?food)
                        (not (already_ate ?food))
                        (not (already_called_about_eating ?food))
                    )
                )
            )

	    )
	:effect (and (success)
	            (not (priority_5))
                ;;(forall (?ask - AskBeforeHelp) (not (askforhelp_protocol_enabled ?ask)) )
                (not (low_level_failed))
                ;;(tried_shutdown)
                (not started)
          )
)


)

