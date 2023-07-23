(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     daily_med - MedicineProtocol
     breakfast - FoodProtocol
     lunch - FoodProtocol
     dinner - FoodProtocol
     daily_wand - WanderingProtocol
     bed door couch outside living_room kitchen bathroom dinning_room - Landmark
     t1 - Time ;;t2 t3 t4 t5
     nathan - Person
  )
  (:init
      (priority_1)
      (medicine_location kitchen)
      (food_location dinning_room)
      ;;(oneof (person_at t2 nathan bed) (person_at t2 nathan door) (person_at t2 nathan couch) (person_at t2 nathan outside) (person_at t2 nathan living_room) (person_at t2 nathan kitchen) (person_at t2 nathan bathroom) (person_at t2 nathan dinning_room) )
      ;;(oneof (person_at t3 nathan bed) (person_at t3 nathan door) (person_at t3 nathan couch) (person_at t3 nathan outside) (person_at t3 nathan living_room) (person_at t3 nathan kitchen) (person_at t3 nathan bathroom) (person_at t3 nathan dinning_room) )
      ;;(oneof (person_at t4 nathan bed) (person_at t4 nathan door) (person_at t4 nathan couch) (person_at t4 nathan outside) (person_at t4 nathan living_room) (person_at t4 nathan kitchen) (person_at t4 nathan bathroom) (person_at t4 nathan dinning_room) )
      ;;(oneof (person_at t5 nathan bed) (person_at t5 nathan door) (person_at t5 nathan couch) (person_at t5 nathan outside) (person_at t5 nathan living_room) (person_at t5 nathan kitchen) (person_at t5 nathan bathroom) (person_at t5 nathan dinning_room) )
  )
  (:goal (and (success)  ) )
)