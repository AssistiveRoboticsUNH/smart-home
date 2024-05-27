(define (problem high_level)
(:domain high_level_domain)
(:objects
Person1 - Person
Time1 - Time
LandmarkRobot1 LandmarkRobot2 - LandmarkRobot
LandmarkPerson1 LandmarkPerson2 LandmarkPerson3 - LandmarkPerson
OneReminderProtocol1 OneReminderProtocol2 OneReminderProtocol3 OneReminderProtocol4 - OneReminderProtocol
)
(:init
	(robot_location LandmarkRobot1)
	(reminder_location LandmarkPerson2)
	(priority_1)
)
(:goal
(success))
)
