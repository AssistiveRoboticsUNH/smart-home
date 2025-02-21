(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     living_room kitchen home outside dining_room bedroom bathroom - Landmark

     am_meds pm_meds - MedicineProtocol
     gym_reminder - GymReminderProtocol
     medicine_refill_reminder - MedicineRefillReminderProtocol
     medicine_pharmacy_reminder - MedicineRefillPharmacyReminderProtocol

     nathan - Person
     t1 - Time ;;t2 t3 t4 t5
  )
  (:init
      (priority_1)

      (visible_location living_room)
      (visible_location dining_room)
      (visible_location bedroom)
      (visible_location bathroom)
      (visible_location kitchen)

      (not_visible_location outside)

    (medicine_location living_room)
    (gym_location living_room)
    (medicine_refill_location living_room)
    (medicine_pharmacy_location living_room)

  )
  (:goal (and (success)  ) )
)