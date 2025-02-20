(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     am_meds pm_meds - MedicineProtocol

     gym_reminder - GymReminderProtocol
     medicine_refill_reminder - MedicineRefillReminderProtocol
     medicine_pharmacy_reminder - MedicineRefillPharmacyReminderProtocol


     home door outside living_room bedroom - Landmark
     t1 - Time ;;t2 t3 t4 t5
     nathan - Person
  )
  (:init
      (priority_1)
      (visible_location living_room)
      (visible_location bedroom)
      (visible_location home)
      (visible_location door)

      (not_visible_location outside)


      (medicine_location living_room)
      (gym_location living_room)
      (medicine_refill_location living_room)
      (medicine_pharmacy_location living_room)




      ;;(robot_at bedroom)
      ;;(person_currently_at nathan living_room)

      ;;check move
      ;;(time_for_move_reminder move_reminder)


      ;; check medicine protocol
      ;;(time_to_take_medicine daily_med)

      ;; check medicine_refill
      ;;(time_for_medicine_refill_reminder medicine_refill_reminder)
      ;;(already_reminded_medicine_refill medicine_refill_reminder)

      ;; check medicine_pharmacy
      ;;(time_for_medicine_pharmacy_reminder medicine_pharmacy_reminder)
      ;;(already_reminded_medicine_pharmacy  medicine_pharmacy_reminder)

      ;;check practice
      ;;(time_for_gym_reminder gym_reminder)
      ;;(already_reminded_gym  gym_reminder)


      ;;(oneof (person_at t2 nathan bed) (person_at t2 nathan door) (person_at t2 nathan couch) (person_at t2 nathan outside) (person_at t2 nathan living_room) (person_at t2 nathan kitchen) (person_at t2 nathan bathroom) (person_at t2 nathan dinning_room) )
      ;;(oneof (person_at t3 nathan bed) (person_at t3 nathan door) (person_at t3 nathan couch) (person_at t3 nathan outside) (person_at t3 nathan living_room) (person_at t3 nathan kitchen) (person_at t3 nathan bathroom) (person_at t3 nathan dinning_room) )
      ;;(oneof (person_at t4 nathan bed) (person_at t4 nathan door) (person_at t4 nathan couch) (person_at t4 nathan outside) (person_at t4 nathan living_room) (person_at t4 nathan kitchen) (person_at t4 nathan bathroom) (person_at t4 nathan dinning_room) )
      ;;(oneof (person_at t5 nathan bed) (person_at t5 nathan door) (person_at t5 nathan couch) (person_at t5 nathan outside) (person_at t5 nathan living_room) (person_at t5 nathan kitchen) (person_at t5 nathan bathroom) (person_at t5 nathan dinning_room) )
  )
  (:goal (and (success)  ) )
)