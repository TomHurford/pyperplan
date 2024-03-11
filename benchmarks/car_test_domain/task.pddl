(define (problem car-problem-instance)
    (:domain car-problem)

    (:objects
        A B C - location
    )

    (:init 
        (at-car A)
        (linked A B)
        (linked B C)
    )

    (:goal (at-car C))
) 
