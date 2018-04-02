(define (domain qual1domain)

	(:requirements :strips :typing :fluents)

	(:types robot bin tray parttype)

	(:functions
		(current-part-in-tray ?parttype - parttype)
		(current-quantity-in-bin ?bin - bin)
		(part-order-in-tray ?parttype - parttype)
	)

	(:predicates
		(inbin ?b - parttype ?t - bin)
		(intray ?b - parttype ?c - parttype)
		(holding ?r - robot ?b - parttype)
		(handempty ?r - robot)
		(atbin ?r - robot ?b - bin)
		(attray ?r - robot ?t - tray)
		(athome ?r - robot)
		
	)

	

	(:action pickup
		:parameters(
			?robot - robot
			?parttype - parttype
			?bin - bin)
		:precondition(and
			(inbin ?parttype ?bin)
			(atbin ?robot ?bin)
			(handempty ?robot))
		:effect(and
			(holding ?robot ?parttype)
			(decrease (current-quantity-in-bin ?bin) 1)
			(not(atbin ?robot ?bin))
			(not(handempty ?robot)))
	)

	(:action moveoverbin
		:parameters(
			?robot - robot
			?bin - bin)
		:precondition(and
			(handempty ?robot)
			(athome ?robot))
		:effect(and
			(atbin ?robot ?bin)
			(not(athome ?robot)))
	)

	(:action moveovertray
		:parameters(
			?robot - robot
			?tray - tray
			?parttype - parttype)
		:precondition(and
			(holding ?robot ?parttype))
		:effect(and
			(attray ?robot ?tray))
	)

	(:action putdown
		:parameters(
			?robot - robot
			?parttype - parttype
			?tray - tray)
		:precondition(and
			(holding ?robot ?parttype)
			(attray ?robot ?tray))
		:effect(and
			(increase (current-part-in-tray ?parttype) 1)
			(not(holding ?robot ?parttype))
			(not(attray ?robot ?tray))
			(handempty ?robot)
			(athome ?robot))
	)
)
