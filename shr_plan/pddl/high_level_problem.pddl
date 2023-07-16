(define (problem medicine_reminder)
  (:domain high_level_domain)
  (:objects
     med_rem - MedicineProtocol
     breakfast - FoodProtocol
     wand_rem - WanderingProtocol
  )
  (:init
      (priority_1)
      (time_to_eat breakfast)
      (too_late_to_go_outside wand_rem)
      (person_at_door wand_rem)
  )
  (:goal (and (success)  ) )
)