( define ( problem problem_1 )
( :domain paul_shr_conditional )
( :objects
	nathan - person
	pioneer - robot
	home bedroom_robot_pos door_robot_pos kitchen_robot_pos couch_robot_pos outside - landmark
)
( :init
	( robot_at pioneer home )
	( medicine_location kitchen_robot_pos )
	( unknown ( person_at nathan bedroom_robot_pos )	)
	( unknown ( person_at nathan kitchen_robot_pos )	)
	( unknown ( person_at nathan couch_robot_pos )	)
	( oneof
		( person_at nathan bedroom_robot_pos )
		( person_at nathan kitchen_robot_pos )
		( person_at nathan couch_robot_pos )
	)
	( unknown ( guide_to_succeeded_attempt_1)	)
	( unknown ( guide_to_succeeded_attempt_2 )	)
	( unknown ( notify_automated_succeeded )	)
	( unknown ( notify_recorded_succeeded )	)
)
( :goal
	( and
		( success )
	)
)
)