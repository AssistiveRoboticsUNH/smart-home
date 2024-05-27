(define (domain high_level_domain)

(:requirements
  :strips
  :typing
)

(:types
  OneReminderProtocol
  LandmarkPerson
  LandmarkRobot
  Time
  Person
)

(:predicates
  (robot_at ?lmr - LandmarkRobot)
  (person_at ?t - Time ?p - Person ?lmp - LandmarkPerson)
  (person_currently_at ?p - Person ?lmp - LandmarkPerson)
  ;;reminder
  (time_for_reminder ?r - OneReminderProtocol)
  (already_reminded_person ?r - OneReminderProtocol)
  (reminder_location ?lmp - LandmarkPerson)
  (robot_location ?lmr - LandmarkRobot)

  ;; priority
  (priority_1)
  (priority_2)
  (priority_3)
  (priority_4)
  (priority_5)

  (low_level_failed)

  (one_reminder_protocol_enabled ?r - OneReminderProtocol)


	(success)

)

(:action MoveToLandmark
	:parameters (?from - LandmarkRobot ?to - LandmarkRobot)
	:precondition (and
	                (robot_at ?from)
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

(:action StartOneReminderProtocol
	:parameters (?r - OneReminderProtocol)
	:precondition (and
	  (priority_2)
      (time_for_reminder ?r)
      (not (already_reminded_person ?r))
      ;;(forall (?one_rem - OneReminderProtocol) (not (one_reminder_protocol_enabled ?one_rem)) )
		;;)
	:effect (and
	          (success)
            (not (priority_2))
            (one_reminder_protocol_enabled ?r)
            (not (low_level_failed))
          )
)

(:action ContinueOneReminderProtocol
	:parameters (?r - OneReminderProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))
      (time_for_reminder ?r)
      (not (already_reminded_person ?r)
      (one_reminder_protocol_enabled ?r)
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
                (forall (?one_rem - OneReminderProtocol) (not (one_reminder_protocol_enabled ?one_rem)) )
                ;;(forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )
                (not (low_level_failed))
          )
)

)

