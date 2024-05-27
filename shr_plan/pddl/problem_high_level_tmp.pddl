(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     breakfast - OneReminderProtocol
     lunch - OneReminderProtocol
     dinner - OneReminderProtocol
     mid_alert - OneReminderProtocol
     bedroom inside_not_bedroom outside - LandmarkPerson
     designated_space home - LandmarkRobot
     t1 - Time ;;t2 t3 t4 t5
     nathan - Person

     bedroom inside_not_bedroom outside - LandmarkPerson
     designated_space home - LandmarkRobot
     nathan - Person
     t1 t2 t3 t4 t5 - Time
  )
  (:init
      (priority_1)
      (reminder_location inside_not_bedroom)
      (robot_location designated_space)
      ;;(oneof (person_at t2 nathan bedroom) (person_at t2 nathan door) (person_at t2 nathan couch) (person_at t2 nathan outside) (person_at t2 nathan living_room) (person_at t2 nathan kitchen) (person_at t2 nathan bathroom) (person_at t2 nathan dining_room) )
      ;;(oneof (person_at t3 nathan bedroom) (person_at t3 nathan door) (person_at t3 nathan couch) (person_at t3 nathan outside) (person_at t3 nathan living_room) (person_at t3 nathan kitchen) (person_at t3 nathan bathroom) (person_at t3 nathan dining_room) )
      ;;(oneof (person_at t4 nathan bedroom) (person_at t4 nathan door) (person_at t4 nathan couch) (person_at t4 nathan outside) (person_at t4 nathan living_room) (person_at t4 nathan kitchen) (person_at t4 nathan bathroom) (person_at t4 nathan dining_room) )
      ;;(oneof (person_at t5 nathan bedroom) (person_at t5 nathan door) (person_at t5 nathan couch) (person_at t5 nathan outside) (person_at t5 nathan living_room) (person_at t5 nathan kitchen) (person_at t5 nathan bathroom) (person_at t5 nathan dining_room) )
  )
  (:goal (and (success)  ) )
)