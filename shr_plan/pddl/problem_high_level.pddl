(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     daily - MedicineProtocol
     breakfast - FoodProtocol
     lunch - FoodProtocol
     dinner - FoodProtocol
     daily - WanderingProtocol
     ;; bed door couch outside living_room kitchen bathroom dinning_room - Landmark
     ;; t1 - Time
     ;; nathan - Person
  )
  (:init
      (priority_1)
      ;;(oneof (person_at t2 nathan bed) (person_at t2 nathan door) (person_at t2 nathan couch) (person_at t2 nathan outside) (person_at t2 nathan living_room) (person_at t2 nathan kitchen) (person_at t2 nathan bathroom) (person_at t2 nathan dinning_room) )
      ;;(oneof (person_at t3 nathan bed) (person_at t3 nathan door) (person_at t3 nathan couch) (person_at t3 nathan outside) (person_at t3 nathan living_room) (person_at t3 nathan kitchen) (person_at t3 nathan bathroom) (person_at t3 nathan dinning_room) )
      ;;(oneof (person_at t4 nathan bed) (person_at t4 nathan door) (person_at t4 nathan couch) (person_at t4 nathan outside) (person_at t4 nathan living_room) (person_at t4 nathan kitchen) (person_at t4 nathan bathroom) (person_at t4 nathan dinning_room) )
      ;;(oneof (person_at t5 nathan bed) (person_at t5 nathan door) (person_at t5 nathan couch) (person_at t5 nathan outside) (person_at t5 nathan living_room) (person_at t5 nathan kitchen) (person_at t5 nathan bathroom) (person_at t5 nathan dinning_room) )
  )
  (:goal (and (success)  ) )
)