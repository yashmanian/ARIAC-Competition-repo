(define (problem qual1problem)
  (:domain pnpdomain)
  ;;objects
  (:objects 
    r - robot
    tray - tray
     piston  gear - parttype
     binforpiston  binforgear - bin
  )

  ;;initial state
  (:init 
    (=(current-part-in-tray  piston ) 0)
    (=(current-part-in-tray  gear ) 0)
    (=(current-quantity-in-bin  binforpiston ) 10)
    (=(current-quantity-in-bin  binforgear ) 10)
    (=(part-order-in-tray  piston ) 2)
    (=(part-order-in-tray  gear ) 3)
    (inbin  piston  binforpiston )
    (inbin  gear  binforgear )
    (handempty r)
    (athome r)
    )

  (:goal (and 
      (= (current-part-in-tray  piston ) (part-order-in-tray  piston ))
      (= (current-part-in-tray  gear ) (part-order-in-tray  gear ))
      (>= (current-quantity-in-bin  binforpiston ) 0)
      (>= (current-quantity-in-bin  binforgear ) 0)
      )
    )
  )
