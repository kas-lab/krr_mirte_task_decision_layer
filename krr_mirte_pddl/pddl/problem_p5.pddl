(define (problem p5)
  (:domain p5 )

  (:objects
    wp0 wp_spoon_dirty wp_spoon_clean wp_dishwasher wp_tableware - waypoint
    obj_1_spoon_clean obj_2_spoon_dirty - item
  )

  (:init
    (robot_at wp0)
    (robot_gripper_free)

    (object_at obj_1_spoon_clean wp_spoon_clean)
    (object_drop_location obj_1_spoon_clean wp_tableware)

    (object_at obj_2_spoon_dirty wp_spoon_dirty)
    (object_drop_location obj_2_spoon_dirty wp_dishwasher)
  )

  (:goal
      (robot_at wp_spoon_dirty)
    ; (and
    ;     (object_at obj_1_spoon_clean wp_tableware)
    ;     (object_at obj_2_spoon_dirty wp_dishwasher)
    ; )
  )
)
