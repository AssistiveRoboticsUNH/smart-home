(define (domain high_level_domain)

(:requirements
  :strips
  :typing
)

(:types
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
  (robot_at ?lmr - Landmark)
  (person_at ?t - Time ?p - Person ?lmp - Landmark)
  (person_currently_at ?p - Person ?lmp - Landmark)
  (visible_location ?lmp - Landmark)

  (medicine_reminder_enabled ?med - MedicineProtocol)
  (exercise_reminder_enabled ?ex - ExerciseReminderProtocol)
  (move_reminder_enabled ?mv - MoveReminderProtocol)
  (internal_check_reminder_enabled ?ic - InternalCheckReminderProtocol)
  (practice_reminder_enabled ?pra - PracticeReminderProtocol)

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

(:action StartMedReminderProtocol
	:parameters (?m - MedicineProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
	    (priority_1)
      (time_to_take_medicine ?m)
      (not (already_took_medicine ?m))
      (not (already_reminded_medicine ?m))
      (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )

      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)
    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (medicine_reminder_enabled ?m)
	          (not (low_level_failed))
              (forall (?internal - InternalCheckReminderProtocol) (not (internal_check_reminder_enabled ?internal)) )
              (forall (?practice - PracticeReminderProtocol) (not (practice_reminder_enabled ?practice)) )
              (forall (?mv - MoveReminderProtocol) (not (move_reminder_enabled ?mv)) )
              (forall (?ex - ExerciseReminderProtocol) (not (exercise_reminder_enabled ?ex)) )
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
	:effect (and (success) (not (priority_2)) )
)

;; exercise reminder Protocol
(:action StartExerciseReminderProtocol
	:parameters (?ex - ExerciseReminderProtocol ?lmp - Landmark ?p - Person)
	:precondition (and
	  (priority_2)
      (time_for_exercise_reminder ?ex)
      (not (already_reminded_exercise ?ex))
      (forall (?ex - ExerciseReminderProtocol) (not (exercise_reminder_enabled ?ex)) )

      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)

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

      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)

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

      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)

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

      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)

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

                (not (low_level_failed))
          )
)

)

