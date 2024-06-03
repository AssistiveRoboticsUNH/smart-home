(define (domain high_level_domain)

(:requirements
  :strips
  :typing
)

(:types
  MedicineProtocol
  GymProtocol
  SleepReminderProtocol
  WalkingProtocol
  AlertProtocol
  FoodProtocol
  LandmarkRobot
  LandmarkPerson
  Time
  Person
)

(:predicates
  (robot_at ?lmr - LandmarkRobot)
  (person_at ?t - Time ?p - Person ?lmp - LandmarkPerson)
  (person_currently_at ?p - Person ?lmp - LandmarkPerson)
  (visible_location ?lmp - LandmarkPerson)

  (sleep_reminder_enabled ?r - SleepReminderProtocol)
  (medicine_reminder_enabled ?med - MedicineProtocol)
  (gym_reminder_enabled ?gym - GymProtocol)
  (walk_reminder_enabled ?walk - WalkingProtocol)
  (alert_reminder_enabled ?a - AlertProtocol)
  (food_reminder_enabled ?f - FoodProtocol)

  ;; food
  (time_to_eat ?f - FoodProtocol)
  (already_ate ?f - FoodProtocol)
  (already_reminded_eating ?f - FoodProtocol)

  ;; alert reminder
  (time_to_alert ?a - AlertProtocol)

  ;; walk reminder
  (time_for_walk_reminder ?w - WalkingProtocol)
  (already_reminded_walk ?w - WalkingProtocol)

  ;; Gym reminder
  (time_for_gym_reminder ?g - GymProtocol)
  (already_reminded_gym ?g - GymProtocol)

  ;; sleep reminder
  (time_for_sleep_reminder ?s - SleepReminderProtocol)
  (already_reminded_sleep ?s - SleepReminderProtocol)

  ;; medicine
  (time_to_take_medicine ?med - MedicineProtocol)
  (already_took_medicine ?m - MedicineProtocol)
  (already_reminded_medicine ?m - MedicineProtocol)

  ;; priority
  (priority_1)
  (priority_2)
  (priority_3)
  (priority_4)
  (priority_5)

  (low_level_failed)

  (success)
)


;; not needed here since the robot and person dont have to be in the same location
;;(:action MoveToLandmark
;;	:parameters (?from - LandmarkRobot ?to - LandmarkRobot)
;;	:precondition (and
;;	                (robot_at ?from)
;;	          )
;;	:effect (and (robot_at ?to) (not (robot_at ?from)) )
;;)

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

(:action StartSleepReminderProtocol
	:parameters (?r - SleepReminderProtocol ?lmp - LandmarkPerson ?p - Person)
	:precondition (and
	    (priority_2)
      (time_for_sleep_reminder ?r)
      (not (already_reminded_sleep ?r))
      (forall (?rem - SleepReminderProtocol) (not (sleep_reminder_enabled ?rem)) )

      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (sleep_reminder_enabled ?r)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )
	          (forall (?gym - GymProtocol) (not (gym_reminder_enabled ?gym)) )
	          (forall (?walk - WalkingProtocol) (not (walk_reminder_enabled ?walk)) )
	          (forall (?alert - AlertProtocol) (not (alert_reminder_enabled ?alert)) )
              (forall (?food - FoodProtocol) (not (food_reminder_enabled ?food)) )
          )
)

(:action ContinueSleepReminderProtocol
	:parameters (?r - SleepReminderProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))

      (not (already_reminded_sleep ?r))
      (sleep_reminder_enabled ?r)
      (time_for_sleep_reminder ?r)
    )
	:effect (and (success) (not (priority_2)) )
)


(:action StartAlertProtocol
	:parameters (?a - AlertProtocol ?lmp - LandmarkPerson ?p - Person)
	:precondition (and
	    (priority_1)
      (time_to_alert ?a)
      (forall (?alert - AlertProtocol) (not (alert_reminder_enabled ?alert)) )

      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)

    )
	:effect (and
	          (success)
	          (not (priority_1))
	          (alert_reminder_enabled ?a)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )
	          (forall (?gym - GymProtocol) (not (gym_reminder_enabled ?gym)) )
	          (forall (?rem - SleepReminderProtocol) (not (sleep_reminder_enabled ?rem)) )
	          (forall (?walk - WalkingProtocol) (not (walk_reminder_enabled ?walk)) )
              (forall (?food - FoodProtocol) (not (food_reminder_enabled ?food)) )
          )
)

(:action ContinueAlertReminderProtocol
	:parameters (?a - AlertProtocol)
	:precondition (and
	    (priority_1)
	    (not (low_level_failed))

      (alert_reminder_enabled ?a)
      (time_to_alert ?a)
    )
	:effect (and (success) (not (priority_1)) )
)



(:action StartWalkReminderProtocol
	:parameters (?w - WalkingProtocol ?lmp - LandmarkPerson ?p - Person)
	:precondition (and
	    (priority_2)
      (time_for_walk_reminder ?w)
      (not (already_reminded_walk ?w))
      (forall (?walk - WalkingProtocol) (not (walk_reminder_enabled ?walk)) )

      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (walk_reminder_enabled ?w)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )
	          (forall (?gym - GymProtocol) (not (gym_reminder_enabled ?gym)) )
	          (forall (?rem - SleepReminderProtocol) (not (sleep_reminder_enabled ?rem)) )
	          (forall (?alert - AlertProtocol) (not (alert_reminder_enabled ?alert)) )
              (forall (?food - FoodProtocol) (not (food_reminder_enabled ?food)) )
          )
)

(:action ContinueWalkReminderProtocol
	:parameters (?w - WalkingProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))

      (not (already_reminded_walk ?w))
      (walk_reminder_enabled ?w)
      (time_for_walk_reminder ?w)
    )
	:effect (and (success) (not (priority_2)) )
)


(:action StartGymReminderProtocol
	:parameters (?g - GymProtocol ?lmp - LandmarkPerson ?p - Person)
	:precondition (and
	    (priority_2)
      (time_for_gym_reminder ?g)
      (not (already_reminded_gym ?g))
      (forall (?gym - GymProtocol) (not (gym_reminder_enabled ?gym)) )

      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)
    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (gym_reminder_enabled ?g)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )
	          (forall (?rem - SleepReminderProtocol) (not (sleep_reminder_enabled ?rem)) )
	          (forall (?walk - WalkingProtocol) (not (walk_reminder_enabled ?walk)) )
	          (forall (?alert - AlertProtocol) (not (alert_reminder_enabled ?alert)) )
              (forall (?food - FoodProtocol) (not (food_reminder_enabled ?food)) )

          )
)

(:action ContinueGymReminderProtocol
	:parameters (?g - GymProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))

        (gym_reminder_enabled ?g)
        (time_for_gym_reminder ?g)
        (not (already_reminded_gym ?g))
    )
	:effect (and (success) (not (priority_2)) )
)

(:action StartMedReminderProtocol
	:parameters (?m - MedicineProtocol ?lmp - LandmarkPerson ?p - Person)
	:precondition (and
	    (priority_2)
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
              (forall (?rem - SleepReminderProtocol) (not (sleep_reminder_enabled ?rem)) )
              (forall (?gym - GymProtocol) (not (gym_reminder_enabled ?gym)) )
              (forall (?walk - WalkingProtocol) (not (walk_reminder_enabled ?walk)) )
              (forall (?alert - AlertProtocol) (not (alert_reminder_enabled ?alert)) )
              (forall (?food - FoodProtocol) (not (food_reminder_enabled ?food)) )


          )
)

(:action ContinueMedReminderProtocol
	:parameters (?m - MedicineProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))
      (medicine_reminder_enabled ?m)
      (not (already_took_medicine ?m))
      (not (already_reminded_medicine ?m))
      (time_to_take_medicine ?m)
    )
	:effect (and (success) (not (priority_2)) )
)

(:action StartFoodReminderProtocol
	:parameters (?f - FoodProtocol ?lmp - LandmarkPerson ?p - Person)
	:precondition (and
	    (priority_2)
      (time_to_eat ?f)
      (not (already_ate ?f))
      (not (already_reminded_eating ?f))
      (forall (?food - FoodProtocol) (not (food_reminder_enabled ?food)) )

      ;; person in visible area
      (person_currently_at ?p ?lmp)
      (visible_location ?lmp)
    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (food_reminder_enabled ?f)
	          (not (low_level_failed))
              (forall (?rem - SleepReminderProtocol) (not (sleep_reminder_enabled ?rem)) )
              (forall (?gym - GymProtocol) (not (gym_reminder_enabled ?gym)) )
              (forall (?walk - WalkingProtocol) (not (walk_reminder_enabled ?walk)) )
              (forall (?alert - AlertProtocol) (not (alert_reminder_enabled ?alert)) )
              (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )
          )
)

(:action ContinueFoodReminderProtocol
	:parameters (?f - FoodProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))
      (food_reminder_enabled ?f)
      (time_to_eat ?f)
      (not (already_ate ?f))
      (not (already_reminded_eating ?f))
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
                (forall (?rem - SleepReminderProtocol) (not (sleep_reminder_enabled ?rem)) )
                (forall (?med - MedicineProtocol) (not (medicine_reminder_enabled ?med)) )
                (forall (?gym - GymProtocol) (not (gym_reminder_enabled ?gym)) )
                (forall (?walk - WalkingProtocol) (not (walk_reminder_enabled ?walk)) )
                (forall (?alert - AlertProtocol) (not (alert_reminder_enabled ?alert)) )
                (forall (?food - FoodProtocol) (not (food_reminder_enabled ?food)) )


                (not (low_level_failed))
          )
)

)

