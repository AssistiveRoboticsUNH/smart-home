(define (domain high_level)

(:requirements
  :strips
  :typing
)

(:types
  FallProtocol
  FoodProtocol
  MedicineProtocol
  WonderingProtocol
)

(:predicates
	;; medicine
	(time_to_take_medicine ?m - MedicineProtocol)
	(already_took_medicine ?m - MedicineProtocol)
	(already_called_about_medicine ?m - MedicineProtocol)
  ;; eating
  (time_to_eat ?f - FoodProtocol)
  (already_ate  ?f - FoodProtocol)
  (already_called_about_eating  ?f - FoodProtocol)
  ;; wondering
  (too_late_to_go_outside)
  ;; fall
  (person_on_ground)
  ;; priority
  (priority_1)
  (priority_2)
  (priority_3)
  (priority_4)
  (priority_5)

	(success)

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
	:parameters (?m - MedicineProtocol)
	:precondition (and
	    (priority_2)
      (time_to_take_medicine ?m)
      (not (already_took_medicine ?m))
      (not (already_called_about_medicine ?m))
		)
	:effect (success)
)

(:action StartFoodProtocol
	:parameters (?f - FoodProtocol)
	:precondition (and
	    (priority_2)
      (time_to_eat ?f)
      (not (already_ate ?f))
      (not (already_called_about_eating ?f))
		)
	:effect (success)
)

(:action StartFallProtocol
	:parameters (?f - FallProtocol)
	:precondition (and
	    (priority_1)
      (person_on_ground)
		)
	:effect (success)
)

(:action StartWonderingProtocol
	:parameters (?w - WonderingProtocol)
	:precondition (and
	    (priority_1)
      (too_late_to_go_outside)
		)
	:effect (success)
)

(:action StartIdle
	:parameters ()
	:precondition (and
	    (priority_5)
		)
	:effect (success)
)



)

