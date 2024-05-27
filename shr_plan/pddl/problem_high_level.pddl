(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     reminder1 reminder2 reminder3 - ReminderProtocol
     bedroom inside_not_bedroom outside - LandmarkPerson
     designated_space home - LandmarkRobot
     nathan - Person
     t1 - Time ;;t2 t3 t4 t5
  )
  (:init
      (priority_1)
      (time_for_reminder reminder3)
      ;;(medicine_location kitchen)
      ;;(food_location dining_room)
  )
  (:goal (and (success)  ) )
)