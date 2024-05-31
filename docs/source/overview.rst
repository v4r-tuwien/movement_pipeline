Overview of the Statemachine
============================

The statemachine consists of numeral `GoToWaypoint` states, which are connected by user input transitions. 
Additionally the state machine houses a smach sequence that executes the 'GoToWaypoint' states in order.

The `GoToWaypoint` state is a simple state that moves the robot to a given waypoint. The state is implemented as a smach state that uses the `move_base/move` action server to move the robot to the given waypoint. The state is parameterized with the waypoint to move to.

The HSR plans its movement according to the map and the given waypoint. If the robot detects an obstacle in the way of the path (e.g. a closed door), the HSR immediately stops and waits for the obstacle to be removed. Thus the execution of the `GoToWaypoint` gets called recursively until the robot reaches the closest possibel waypoint to its destianation. This is done by iterating through the planned path backwards, and checking if the point is reachable. If the point is reachable, the robot tries to move there first before continuing to its original waypoint. 

Due to this implemtation, the `GoToWaypoint` differs from the original one of the graspign pipeline. This change was needed since the HSR may detect a path as blocked, even though it is not (e.g. it detects the door as closed, even though it is open).