(define (problem high_level)
  (:domain shr_domain)
  (:objects
     noon - MedicineProtocol
     sleep_reminder - SleepReminderProtocol
     gym_reminder - GymProtocol
     walking_reminder - WalkingProtocol
     night_alert - AlertProtocol
     breakfast lunch - FoodProtocol
     bedroom visible_area outside bathroom - LandmarkPerson
     designated_space home - LandmarkRobot
     nathan - Person
     t1 - Time ;;t2 t3 t4 t5
  )
  (:init
      (priority_1)
      (visible_location visible_area)


      ;; visible location of person
      ;;(visible_location visible_area)

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


  )
  (:goal (and (success)  ) )
)