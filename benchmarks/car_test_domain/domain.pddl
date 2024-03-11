(define (domain car-problem)

    (:requirements :typing :strips)
    
    (:types location)

    (:predicates 
        (at-car ?l - location)  ; car is at location 
        (linked ?l1 ?l2 - location)  ; two locations are linked
        (has-fuel) ; car has fuel
    )

    (:action refuel
        :parameters (?l - location)
        :precondition (at-car ?l)
        :effect (has-fuel)
    )

    (:action move
        :parameters (?from ?to - location)
        :precondition (and (at-car ?from) (linked ?from ?to) (has-fuel))
        :effect (and (not (at-car ?from)) (at-car ?to) (not (has-fuel)))
    ) 
)
