(define (domain high_level_domain)

(:requirements
  :strips
  :typing
)

(:types
  GymReminderProtocol
  MedicineRefillReminderProtocol
  MedicineRefillPharmacyReminderProtocol
  MedicineProtocol
  WalkingProtocol
  Landmark
  Time
  Person

  ;; low level
    Msg
    ReminderAction
    CallAction
    VoiceAction
)

(:predicates
  (started)

  (robot_at ?lmr - Landmark)
  (person_at ?t - Time ?p - Person ?lmp - Landmark)
  (person_currently_at ?p - Person ?lmp - Landmark)

  (visible_location ?lmp - Landmark)
  (not_visible_location ?lmp - Landmark)

(disable_refill)
  (medicine_protocol_enabled ?med - MedicineProtocol)
  (gym_reminder_enabled ?gy - GymReminderProtocol)
  (medicine_refill_reminder_enabled ?mdrf - MedicineRefillReminderProtocol)
  (medicine_pharmacy_reminder_enabled ?ic - MedicineRefillPharmacyReminderProtocol)
  (walking_protocol_enabled ?w - WalkingProtocol)

  ;; medicine
  ;;(medicine_location ?lm - Landmark)
  (time_to_take_medicine ?med - MedicineProtocol)
  (already_took_medicine ?m - MedicineProtocol)
  (already_reminded_medicine ?m - MedicineProtocol)
  (already_called_about_medicine ?m - MedicineProtocol)


  ;; gym reminder
  ;;(gym_location ?lm - Landmark)
  (gym_reminder_enabled ?gy - GymReminderProtocol)
  (time_for_gym_reminder ?gy - GymReminderProtocol)
  (already_reminded_gym ?gy - GymReminderProtocol)

   ;; medicine_refill reminder
  ;;(medicine_refill_location ?lm - Landmark)
  (medicine_refill_reminder_enabled ?mdrf - MedicineRefillReminderProtocol)
  (time_for_medicine_refill_reminder ?mdrf - MedicineRefillReminderProtocol)
  (already_reminded_medicine_refill ?mdrf - MedicineRefillReminderProtocol)

    ;; medicinepharmacy  reminder
  ;;(medicine_pharmacy_location ?lm - Landmark)
  (medicine_pharmacy_reminder_enabled ?ic - MedicineRefillPharmacyReminderProtocol)
  (time_for_medicine_pharmacy_reminder ?ic - MedicineRefillPharmacyReminderProtocol)
  (already_reminded_medicine_pharmacy ?ic - MedicineRefillPharmacyReminderProtocol)

  ;;walking reminder

  ;;(walking_reminder_location ?lm - Landmark)
  (walking_reminder_enabled ?w - WalkingProtocol)
  (time_for_walking_reminder ?w - WalkingProtocol)
  (already_reminded_walking ?w - WalkingProtocol)
  (good_weather ?w - WalkingProtocol)

  (low_level_failed)

  ;; priority
  (priority_1)
  (priority_2)
  (priority_3)
  (priority_4)
  (priority_5)

  (low_level_failed)
  (dont_use_shutdown)
  (success)

  ;; for low level
    (executed_reminder ?a - ReminderAction)
    (executed_call ?c - CallAction)
    (executed_voice ?a - VoiceAction)

    (message_given ?m - Msg)
)

(:action MoveToLandmark
	:parameters (?from - Landmark ?to - Landmark)
	:precondition (and
	                (robot_at ?from)
	                (started)
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

;; to start ros and navigation before the protocol
(:action StartROS
	:parameters ()
	:precondition (;;and
	       ;; will be triggered before it starts a protocol
           ;; (priority_2)
		)
	:effect (and
	            ;;(not (priority_2))
                (started)
          )
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

      ;;(medicine_location ?dest)
      
      (not (already_took_medicine ?m))
      (not (already_reminded_medicine ?m))
      (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
      (started)
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
      ;;(gym_location ?dest)

      (time_for_gym_reminder ?gy)
      (not (already_reminded_gym ?gy))
      (forall (?gy - GymReminderProtocol) (not (gym_reminder_enabled ?gy)) )

      ;; person in visible area
      (person_currently_at ?p ?cur)
      (visible_location ?dest)
      (not (not_visible_location ?dest))
      (visible_location ?cur)
      (not (not_visible_location ?cur))

      (started)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (gym_reminder_enabled ?gy)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )

              (forall (?medicine_pharmacy - MedicineRefillPharmacyReminderProtocol) (not (medicine_pharmacy_reminder_enabled ?medicine_pharmacy)) )
              (forall (?mdrf - MedicineRefillReminderProtocol) (not (medicine_refill_reminder_enabled ?mdrf)) )
              (forall (?w - WalkingProtocol) (not (walking_reminder_enabled ?w)) )
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

    (not disable_refill)
      (robot_at ?cur)
      ;;(medicine_refill_location ?dest)

      (time_for_medicine_refill_reminder ?mdrf)
      (not (already_reminded_medicine_refill ?mdrf))
      (forall (?mdrf - MedicineRefillReminderProtocol) (not (medicine_refill_reminder_enabled ?mdrf)) )

      ;; person in visible area
      (person_currently_at ?p ?cur)
      (visible_location ?dest)
      (not (not_visible_location ?dest))
      (visible_location ?cur)
      (not (not_visible_location ?cur))

      (started)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (medicine_refill_reminder_enabled ?mdrf)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
              (forall (?medicine_pharmacy - MedicineRefillPharmacyReminderProtocol) (not (medicine_pharmacy_reminder_enabled ?medicine_pharmacy)) )
              (forall (?gy - GymReminderProtocol) (not (gym_reminder_enabled ?gy)) )
              (forall (?w - WalkingProtocol) (not (walking_reminder_enabled ?w)) )
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
      ;;(medicine_pharmacy_location ?dest)

      (time_for_medicine_pharmacy_reminder ?ic)
      (not (already_reminded_medicine_pharmacy ?ic))
      (forall (?ic - MedicineRefillPharmacyReminderProtocol) (not (medicine_pharmacy_reminder_enabled ?ic)) )

      ;; person in visible area
      (person_currently_at ?p ?cur)
      (visible_location ?dest)
      (not (not_visible_location ?dest))

      (started)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (medicine_pharmacy_reminder_enabled ?ic)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
              (forall (?mdrf - MedicineRefillReminderProtocol) (not (medicine_refill_reminder_enabled ?mdrf)) )
              (forall (?gy - GymReminderProtocol) (not (gym_reminder_enabled ?gy)) )
              (forall (?w - WalkingProtocol) (not (walking_reminder_enabled ?w)) )
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


(:action StartWalkingProtocol
	:parameters (?w - WalkingProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
	    (priority_2)

	  (robot_at ?cur)
      ;;(walking_reminder_location ?dest)

      (time_for_walking_reminder ?w)
      (not (already_reminded_walking ?w))
      (good_weather ?w)
      (forall (?ic - WalkingProtocol) (not (walking_reminder_enabled ?w)) )

      ;; person in visible area
      (person_currently_at ?p ?cur)
      (visible_location ?dest)
      (not (not_visible_location ?dest))
      (started)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (walking_reminder_enabled ?w)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
              (forall (?mdrf - MedicineRefillReminderProtocol) (not (medicine_refill_reminder_enabled ?mdrf)) )
              (forall (?medicine_pharmacy - MedicineRefillPharmacyReminderProtocol) (not (medicine_pharmacy_reminder_enabled ?medicine_pharmacy)) )
              (forall (?gy - GymReminderProtocol) (not (gym_reminder_enabled ?gy)) )
          )
)

(:action ContinueWalkingProtocol
	:parameters (?w - WalkingProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))

      (not (already_reminded_walking ?w))
      (good_weather ?w)
      (walking_reminder_enabled ?w)
      (time_for_walking_reminder ?w)
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
                (forall (?w - WalkingProtocol) (not (walking_reminder_enabled ?w)) )
                (not (low_level_failed))
          )
)


;; shutdown is supposed to stop ros2 processes
;; it should try to dock if it is not docked
;; triggered when there should be protocol and it has been done

(:action Shutdown
	:parameters ()
	:precondition
	    (and
	        (started)
	        ;; has to be higher priority than idle
            (priority_4)

            ;; CANT SHUTDOWN IF time to do something is true and
            ;; all predicates indicating that they it is done are false
            ;; give F in such case

            ;; need to add a forall for every protocol objects in problem
            ;;; 1
            ;;; forall would give false if one is F
            (forall (?med - MedicineProtocol)
                (not
                    (and
                        (time_to_take_medicine ?med)
                        (not (already_took_medicine ?med))
                        (not (already_reminded_medicine ?med))
                        (not (already_called_about_medicine ?med))
                    )
                )
            )
            ;;; 2
            (forall (?gym - GymReminderProtocol)
                (not
                    (and
                        (time_for_gym_reminder ?gym)
                        (not (already_reminded_gym ?gym))
                    )
                )
            )
            ;;; 3
            (forall (?mr - MedicineRefillReminderProtocol)
                (not
                    (and
                        (time_for_medicine_refill_reminder ?mr)
                        (not (already_reminded_medicine_refill ?mr))
                    )
                )
            )
            ;;; 4
            (forall (?mrp - MedicineRefillPharmacyReminderProtocol)
                (not
                    (and
                        (time_for_medicine_pharmacy_reminder ?mrp)
                        (not (already_reminded_medicine_pharmacy ?mrp))
                    )
                )
            )

            ;;; 5 
            (forall (?wk - WalkingProtocol)
                (not
                    (and
                        (time_for_walking_reminder ?wk)
                        (good_weather ?wk)
                        (not (already_reminded_walking ?wk))
                    )
                )
            )


	    )
	:effect (and (success)
	            (not (priority_4))
                (not (low_level_failed))
                (not started)
          )
)


)

