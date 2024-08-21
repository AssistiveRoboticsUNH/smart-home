(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     am_meds pm_meds - MedicineProtocol
     move_reminder - MoveReminderProtocol
     exercise_reminder - ExerciseReminderProtocol
     internal_check_reminder - InternalCheckReminderProtocol
     practice_reminder - PracticeReminderProtocol
     living_room home outside bedroom - Landmark
     nathan - Person
     t1 - Time ;;t2 t3 t4 t5
  )
  (:init
      ;;(priority_5)
      (visible_location living_room)
      (visible_location bedroom)

      ;;(person_currently_at nathan visible_area)

      ;;(person_currently_at nathan visible_area)
      ;;(time_for_walk_reminder walking_reminder)

      ;; testing medicine
      ;;(time_to_eat breakfast)
      ;;(already_reminded_eating  breakfast)

      ;; alert walk
      ;;(time_to_alert night_alert)

      ;; testing walk
      ;;(time_for_walk_reminder walking_reminder)
      ;;(already_reminded_walk walking_reminder)

      ;; testing gym
      ;;(time_for_gym_reminder gym_reminder)
      ;;(already_reminded_gym gym_reminder)

      ;; testing medicine
      ;;(time_to_take_medicine morning)
      ;;(already_took_medicine  noon)

      ;; testing sleep
      ;;(time_for_sleep_reminder sleep_reminder)
      ;;(already_reminded_sleep  sleep_reminder)

      ;; testing move
      ;;(time_for_move_reminder move_reminder)
      ;;(already_reminded_move  move_reminder)

      ;; testing internal check
      ;;(time_for_internalcheck_reminder internalcheck_reminder)
      ;;(already_reminded_internalcheck  internalcheck_reminder)

      ;; testing practice
      ;;(time_for_practice_reminder practice_reminder)
      ;;(already_reminded_practice  practice_reminder)
  )
  (:goal (and (success)  ) )
)