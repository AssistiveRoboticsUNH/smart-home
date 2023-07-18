(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     daily - MedicineProtocol
     breakfast - FoodProtocol
     lunch - FoodProtocol
     dinner - FoodProtocol
     daily - WanderingProtocol
  )
  (:init
      (priority_1)
  )
  (:goal (and (success)  ) )
)