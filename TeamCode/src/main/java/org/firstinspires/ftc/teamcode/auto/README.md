# How to make an auto

Copy `opmodes/AutoFromRest.java`, rename it, and change the steps after `robot.setNormalSpeed(...)`.
The last line should always be `robot.stopAll()`.

Directions: `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`

Intake: `INTAKE_ON` or `INTAKE_OFF`

## Moving

`robot.move(LEFT, 1.0, INTAKE_OFF);`
Drive left for 1 second at normal speed.

`robot.move(LEFT, 1.0, INTAKE_ON, 0.3);`
Drive left at speed 0.3 with the intake on. Goes the same distance as above, just slower.

`robot.pause(1.0, INTAKE_OFF);`
Stand still for 1 second.

`robot.wiggle(FORWARD, 3, 0.1, 0.25, INTAKE_ON, 0.25);`
Wiggle forward and back 3 times, 0.1 each way, pausing 0.25 seconds at each turn, intake on, at speed 0.25.

## Turning

`robot.turnTo(-90, 1.5, INTAKE_OFF);`
Turn to -90 degrees from where the robot started (negative = clockwise). Gives up after 1.5 seconds.

`robot.turnFor(0.5, 0.4, INTAKE_OFF);`
Turn for 0.5 seconds at power 0.4 (positive = clockwise).

## Shooting

`robot.alignToGoal(1.0);`
Turn to face the goal for 1 second (only if the camera can see it).

`robot.shoot(3.0);`
Shoot all the balls. Gives up after 3 seconds.

## Other

`robot.stopIntake();`
Turn the intake off.

`robot.sideServos.goToA();` / `robot.sideServos.goToB();`
Move the side servos.

`robot.stopAll();`
Stop everything. Always the last line.

## Notes

- Directions stay the same on the field even after the robot turns.
- Turn angles are measured from where the robot started, not from the last turn.
- The whole auto has to finish in 30 seconds.
