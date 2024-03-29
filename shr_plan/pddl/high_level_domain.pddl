(define (domain high_level_domain)

(:requirements
  :strips
  :typing
)

(:types
  FallProtocol
  FoodProtocol
  MedicineProtocol
  WanderingProtocol
  Landmark
  Time
  Person
)

(:predicates
  (robot_at ?lm - Landmark)
  (person_at ?t - Time ?p - Person ?lm - Landmark)
  (person_currently_at ?p - Person ?lm - Landmark)
  ;; medicine
  (medicine_location ?lm - Landmark)
	(time_to_take_medicine ?m - MedicineProtocol)
	(already_took_medicine ?m - MedicineProtocol)
	(already_called_about_medicine ?m - MedicineProtocol)
  ;; eating
  (food_location ?lm - Landmark)
  (time_to_eat ?f - FoodProtocol)
  (already_ate  ?f - FoodProtocol)
  (already_called_about_eating  ?f - FoodProtocol)
  ;; wandering
  (too_late_to_go_outside ?w - WanderingProtocol)
  (person_at_door ?w - WanderingProtocol)
  (person_outside ?w - WanderingProtocol)
  ;; fall
  (person_on_ground ?f - FallProtocol)
  ;; priority
  (priority_1)
  (priority_2)
  (priority_3)
  (priority_4)
  (priority_5)

  (low_level_failed)

  (fall_protocol_enabled ?f - FallProtocol)
  (food_protocol_enabled ?f - FoodProtocol)
  (medicine_protocol_enabled ?m - MedicineProtocol)
  (wandering_protocol_enabled ?w - WanderingProtocol)

	(success)

)

(:action MoveToLandmark
	:parameters (?from - Landmark ?to - Landmark)
	:precondition (and
	                (robot_at ?from)
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
	:parameters (?m - MedicineProtocol ?p - Person ?cur ?dest - Landmark)
	:precondition (and
	    (priority_2)
      (time_to_take_medicine ?m)
      (person_currently_at ?p ?cur)
      (robot_at ?cur)
      (medicine_location ?dest)
      (not (already_took_medicine ?m))
      (not (already_called_about_medicine ?m))
      (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
		)
	:effect (and
	          (success)
            (not (priority_2))
            (medicine_protocol_enabled ?m)
            (not (low_level_failed))
            (forall (?fall - FallProtocol) (not (fall_protocol_enabled ?fall)) )
            (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )
            (forall (?wond - WanderingProtocol) (not (wandering_protocol_enabled ?wond)) )
          )
)

(:action ContinueMedicineProtocol
	:parameters (?m - MedicineProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))
      (time_to_take_medicine ?m)
      (not (already_took_medicine ?m))
      (not (already_called_about_medicine ?m))
      (medicine_protocol_enabled ?m)
		)
	:effect (and (success) (not (priority_2)) )
)

(:action StartFoodProtocol
	:parameters (?f - FoodProtocol  ?p - Person ?cur ?dest - Landmark)
	:precondition (and
	    (priority_2)
      (time_to_eat ?f)
      (person_currently_at ?p ?cur)
      (robot_at ?cur)
      (food_location ?dest)
      (not (already_ate ?f))
      (not (already_called_about_eating ?f))
      (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )
		)
	:effect (and
	          (success)
            (not (priority_2))
            (food_protocol_enabled ?f)
            (not (low_level_failed))
            (forall (?fall - FallProtocol) (not (fall_protocol_enabled ?fall)) )
            (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
            (forall (?wond - WanderingProtocol) (not (wandering_protocol_enabled ?wond)) )
          )
)
(:action ContinueFoodProtocol
	:parameters (?f - FoodProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))
      (time_to_eat ?f)
      (not (already_ate ?f))
      (not (already_called_about_eating ?f))
      (food_protocol_enabled ?f)
		)
	:effect (and (success) (not (priority_2)) )
)

(:action StartFallProtocol
	:parameters (?f - FallProtocol)
	:precondition (and
	    (priority_1)
      (person_on_ground ?f)
      (forall (?fall - FallProtocol) (not (fall_protocol_enabled ?fall)) )
		)
	:effect (and
	          (success)
	          (not (priority_1))
	          (fall_protocol_enabled ?f)
	          (not (low_level_failed))
            (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
            (forall (?wond - WanderingProtocol) (not (wandering_protocol_enabled ?wond)) )
            (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )
          )
)

(:action ContinueFallProtocol
	:parameters (?f - FallProtocol)
	:precondition (and
	    (priority_1)
	    (not (low_level_failed))
      (person_on_ground ?f)
      (fall_protocol_enabled ?f)
		)
	:effect (and (success) (not (priority_1)) )
)

(:action StartWanderingProtocol
	:parameters (?w - WanderingProtocol)
	:precondition (and
	    (priority_1)
      (not
        (and (not (person_at_door ?w) ) (not (person_outside ?w) ) )
      )
      (too_late_to_go_outside ?w)
      (forall (?wond - WanderingProtocol) (not (wandering_protocol_enabled ?wond)) )
    )
	:effect (and
	          (success)
	          (not (priority_1))
	          (wandering_protocol_enabled ?w)
	          (not (low_level_failed))
            (forall (?fall - FallProtocol) (not (fall_protocol_enabled ?fall)) )
            (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
            (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )
          )
)

(:action ContinueWanderingProtocol
	:parameters (?w - WanderingProtocol)
	:precondition (and
	    (priority_1)
	    (not (low_level_failed))
      (not
        (and (not (person_at_door ?w) ) (not (person_outside ?w) ) )
      )
      (too_late_to_go_outside ?w)
      (wandering_protocol_enabled ?w)
    )
	:effect (and (success) (not (priority_1)) )
)


(:action Idle
	:parameters ()
	:precondition (and
	    (priority_5)
		)
	:effect (and (success)
	              (not (priority_5))
                (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
                (forall (?wond - WanderingProtocol) (not (wandering_protocol_enabled ?wond)) )
                (forall (?food - FoodProtocol) (not (food_protocol_enabled ?food)) )
                (forall (?fall - FallProtocol) (not (fall_protocol_enabled ?fall)) )
                (not (low_level_failed))
          )
)

)

