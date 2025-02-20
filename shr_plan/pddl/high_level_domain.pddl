(define (domain high_level_domain)

(:requirements
  :strips
  :typing
)

(:types

  ;; new protocol
  GymReminderProtocol
  MedicineRefillReminderProtocol
  MedicineRefillPharmacyReminderProtocol
  MedicineProtocol
  Landmark
  Time
  Person
)

(:predicates
  (robot_at ?lm - Landmark)
  (person_at ?t - Time ?p - Person ?lm - Landmark)
  (person_currently_at ?p - Person ?lm - Landmark)

  (visible_location ?lmp - Landmark)
  (not_visible_location ?lmp - Landmark)


  ;; medicine
  (medicine_location ?lm - Landmark)
  (time_to_take_medicine ?m - MedicineProtocol)
  (already_took_medicine ?m - MedicineProtocol)
  (already_reminded_medicine ?m - MedicineProtocol)
  (already_called_about_medicine ?m - MedicineProtocol)
  

  ;; gym reminder
  (gym_location ?lm - Landmark)
  (gym_reminder_enabled ?gy - GymReminderProtocol)
  (time_for_gym_reminder ?gy - GymReminderProtocol)
  (already_reminded_gym ?gy - GymReminderProtocol)

  ;; medicine_refill reminder
  (medicine_refill_location ?lm - Landmark)
  (medicine_refill_reminder_enabled ?mdrf - MedicineRefillReminderProtocol)
  (time_for_medicine_refill_reminder ?mdrf - MedicineRefillReminderProtocol)
  (already_reminded_medicine_refill ?mdrf - MedicineRefillReminderProtocol)

  ;; medicinepharmacy  reminder
  (medicine_pharmacy_location ?lm - Landmark)
  (medicine_pharmacy_reminder_enabled ?ic - MedicineRefillPharmacyReminderProtocol)
  (time_for_medicine_pharmacy_reminder ?ic - MedicineRefillPharmacyReminderProtocol)
  (already_reminded_medicine_pharmacy ?ic - MedicineRefillPharmacyReminderProtocol)




  ;; priority
  (priority_1)
  (priority_2)
  (priority_3)
  (priority_4)
  (priority_5)

  (low_level_failed)

  (medicine_protocol_enabled ?m - MedicineProtocol)
  (gym_reminder_enabled ?gy - GymReminderProtocol)
  (medicine_refill_reminder_enabled ?mdrf - MedicineRefillReminderProtocol)
  (medicine_pharmacy_reminder_enabled ?ic - MedicineRefillPharmacyReminderProtocol)


  (success)

)

(:action MoveToLandmark
	:parameters (?from - Landmark ?to - Landmark)
	:precondition (and
	                (robot_at ?from)
	                (visible_location ?from)
	                (visible_location ?to)
	                (not (not_visible_location ?from))
	                (not (not_visible_location ?to))


	          )
	:effect (and (robot_at ?to) (not (robot_at ?from)) )
)


(:action ChangePriority_1_2
	:parameters ()
	:precondition (and
	    (priority_1)
		)
	:effect (and (priority_2) (not (priority_1)))
)
(:action ChangePriority_2_3
	:parameters ()
	:precondition (and
	    (priority_2)
		)
	:effect (and (priority_3) (not (priority_2)))
)
(:action ChangePriority_3_4
	:parameters ()
	:precondition (and
	    (priority_3)
		)
	:effect (and (priority_4) (not (priority_3)))
)
(:action ChangePriority_4_5
	:parameters ()
	:precondition (and
	    (priority_4)
		)
	:effect (and (priority_5) (not (priority_4)))
)



(:action StartMedicineProtocol
	:parameters (?m - MedicineProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
	    (priority_2)
      (time_to_take_medicine ?m)
      (visible_location ?dest)
      (not (not_visible_location ?dest))
      (visible_location ?cur)
      (not (not_visible_location ?cur))
      (person_currently_at ?p ?cur)
      (robot_at ?cur)
      (medicine_location ?dest)
      (not (already_took_medicine ?m))
      (not (already_reminded_medicine ?m))
      (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
		)
	:effect (and
	          (success)
            (not (priority_2))
            (medicine_protocol_enabled ?m)
            (not (low_level_failed))
            (forall (?medicine_pharmacy - MedicineRefillPharmacyReminderProtocol) (not (medicine_pharmacy_reminder_enabled ?medicine_pharmacy)) )
            (forall (?mdrf - MedicineRefillReminderProtocol) (not (medicine_refill_reminder_enabled ?mdrf)) )
            (forall (?gy - GymReminderProtocol) (not (gym_reminder_enabled ?gy)) )
          )
)

(:action ContinueMedicineProtocol
	:parameters (?m - MedicineProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))
      (time_to_take_medicine ?m)
      (not (already_took_medicine ?m))
      (not (already_reminded_medicine ?m))
      (not (already_called_about_medicine ?m))
      (medicine_protocol_enabled ?m)
		)
	:effect (and (success) (not (priority_2)) )
)


;; Gym reminder Protocol
(:action StartGymReminderProtocol
	:parameters (?gy - GymReminderProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
	  (priority_2)


      (robot_at ?cur)
      (gym_location ?dest)

      (time_for_gym_reminder ?gy)
      (not (already_reminded_gym ?gy))
      (forall (?gy - GymReminderProtocol) (not (gym_reminder_enabled ?gy)) )

      ;; person in visible area
      (person_currently_at ?p ?cur)
      (visible_location ?dest)
      (not (not_visible_location ?dest))
      (visible_location ?cur)
      (not (not_visible_location ?cur))

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (gym_reminder_enabled ?gy)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )

              (forall (?medicine_pharmacy - MedicineRefillPharmacyReminderProtocol) (not (medicine_pharmacy_reminder_enabled ?medicine_pharmacy)) )
              (forall (?mdrf - MedicineRefillReminderProtocol) (not (medicine_refill_reminder_enabled ?mdrf)) )
          )
)

(:action ContinueGymReminderProtocol
	:parameters (?gy - GymReminderProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))

      (not (already_reminded_gym ?gy))
      (gym_reminder_enabled ?gy)
      (time_for_gym_reminder ?gy)
    )
	:effect (and (success) (not (priority_2)) )
)



;; medicine_refill reminder Protocol
(:action StartMedicineRefillReminderProtocol
	:parameters (?mdrf - MedicineRefillReminderProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
	    (priority_2)


      (robot_at ?cur)
      (medicine_refill_location ?dest)

      (time_for_medicine_refill_reminder ?mdrf)
      (not (already_reminded_medicine_refill ?mdrf))
      (forall (?mdrf - MedicineRefillReminderProtocol) (not (medicine_refill_reminder_enabled ?mdrf)) )

      ;; person in visible area
      (person_currently_at ?p ?cur)
      (visible_location ?dest)
      (not (not_visible_location ?dest))
      (visible_location ?cur)
      (not (not_visible_location ?cur))

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (medicine_refill_reminder_enabled ?mdrf)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
              (forall (?medicine_pharmacy - MedicineRefillPharmacyReminderProtocol) (not (medicine_pharmacy_reminder_enabled ?medicine_pharmacy)) )
              (forall (?gy - GymReminderProtocol) (not (gym_reminder_enabled ?gy)) )
          )
)

(:action ContinueMedicineRefillReminderProtocol
	:parameters (?mdrf - MedicineRefillReminderProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))

      (not (already_reminded_medicine_refill ?mdrf))
      (medicine_refill_reminder_enabled ?mdrf)
      (time_for_medicine_refill_reminder ?mdrf)
    )
	:effect (and (success) (not (priority_2)) )
)

;; medicine_pharmacy  Reminder Protocol

(:action StartMedicineRefillPharmacyReminderProtocol
	:parameters (?ic - MedicineRefillPharmacyReminderProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
	    (priority_2)

	  (robot_at ?cur)
      (medicine_pharmacy_location ?dest)

      (time_for_medicine_pharmacy_reminder ?ic)
      (not (already_reminded_medicine_pharmacy ?ic))
      (forall (?ic - MedicineRefillPharmacyReminderProtocol) (not (medicine_pharmacy_reminder_enabled ?ic)) )

      ;; person in visible area
      (person_currently_at ?p ?cur)
      (visible_location ?dest)
      (not (not_visible_location ?dest))

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (medicine_pharmacy_reminder_enabled ?ic)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
              (forall (?mdrf - MedicineRefillReminderProtocol) (not (medicine_refill_reminder_enabled ?mdrf)) )
              (forall (?gy - GymReminderProtocol) (not (gym_reminder_enabled ?gy)) )
          )
)

(:action ContinueMedicineRefillPharmacyReminderProtocol
	:parameters (?ic - MedicineRefillPharmacyReminderProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))

      (not (already_reminded_medicine_pharmacy ?ic))
      (medicine_pharmacy_reminder_enabled ?ic)
      (time_for_medicine_pharmacy_reminder ?ic)
    )
	:effect (and (success) (not (priority_2)) )
)



(:action Idle
	:parameters ()
	:precondition (and
	    (priority_5)
		)
	:effect (and (success)
	              (not (priority_5))
                (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
                (forall (?medicine_pharmacy - MedicineRefillPharmacyReminderProtocol) (not (medicine_pharmacy_reminder_enabled ?medicine_pharmacy)) )
                (forall (?mdrf - MedicineRefillReminderProtocol) (not (medicine_refill_reminder_enabled ?mdrf)) )
                (forall (?gy - GymReminderProtocol) (not (gym_reminder_enabled ?gy)) )
                (not (low_level_failed))
          )
)

)

