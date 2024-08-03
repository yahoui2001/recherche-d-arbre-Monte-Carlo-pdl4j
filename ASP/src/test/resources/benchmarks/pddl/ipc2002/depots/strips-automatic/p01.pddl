(define (problem robot_problem)
    (:domain robot_navigation)
    (:objects
        loc1 loc2 loc3 loc4 - location
        robot1 - robot
    )
    (:init
        (at robot1 loc1)
        
        (connected loc1 loc2)
        (connected loc2 loc3)
        (connected loc3 loc4)
        
      
       
       

       
       
        
    )
    (:goal
        (at robot1 loc4)
    )
)

