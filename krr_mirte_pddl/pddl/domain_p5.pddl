(define (domain p5)
  (:requirements
    :strips
    :typing
    :adl
    :negative-preconditions
    :durative-actions
  )

  (:types
    waypoint
    item ; the object type already exists in PDDL
  )

  (:predicates
    (robot_at ?wp - waypoint)
    
    (object_at ?i - item ?wp - waypoint)
    (robot_holds ?i - item)
    (robot_gripper_free)
    (robot_gripper_busy)

    (object_drop_location ?i - item ?wp - waypoint)
  )

  (:durative-action move
    :parameters (?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start (robot_at ?wp1))
        
        ; (at start (connected ?wp1 ?wp2))
        ; (at end (connected ?wp1 ?wp2))
        ; (over all (connected ?wp1 ?wp2))
    )
    :effect (and
        (at start (not(robot_at ?wp1)))
        (at end (robot_at ?wp2))
    )
  )

  (:durative-action pick
    :parameters (?i - item ?wp - waypoint)
    :duration ( = ?duration 1)
    :condition (and
        (at start (robot_gripper_free))

        (at start (robot_at ?wp))
        (at end (robot_at ?wp))
        (over all (robot_at ?wp))
      
        (at start (object_at ?i ?wp))
        ; (at end (object_at ?i ?wp))
        (over all (object_at ?i ?wp))
    )
    :effect (and
        (at start (not (robot_gripper_free)))
        (at end (robot_gripper_busy))
        (at end (robot_holds ?i))
        (at end (not (object_at ?i ?wp)))
    )
  )

  (:durative-action place
    :parameters (?i - item ?wp - waypoint)
    :duration ( = ?duration 1)
    :condition (and
        (at start (object_drop_location ?i ?wp))
        
        (at start (robot_at ?wp))
        (at end (robot_at ?wp))
        (over all (robot_at ?wp))
        
        (at start (robot_holds ?i))
        (over all (robot_holds ?i))
        ; (at end (object_at ?i ?wp))
    )
    :effect (and
        (at end (not (robot_holds ?i)))
        (at end (object_at ?i ?wp))
        (at end (robot_gripper_free))
        (at end (not (robot_gripper_busy)))
    )
  )

)
