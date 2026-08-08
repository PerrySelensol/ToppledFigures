require("physEngine/libs/vectors")
local common = require("./common")

--[=============================================================================]--

local function getDampedSpringValues(dt)
	local damping = 10 -- Damping ratio
	local frequency = 0.25/dt -- Spring frequency

	local omega = 2 * math.pi * frequency -- Spring angular frequency
	local a1 = 2 * damping + omega * dt
	local a2 = dt * omega * a1
	local a3 = 1 / (1 + a2)
	local biasRate = omega / a1
	local massCoeff = a2 * a3
	local impulseCoeff = a3

	return biasRate, massCoeff, impulseCoeff
end

-- //TODO add restitution
local function solveContact(contact, dt, useBias)
	-- Convert contact point to world orientation, but local position
	local contactPointA = contact.A.oriMat*contact.contactPointA
	local contactPointB = contact.B
		and contact.B.oriMat*contact.contactPointB
		or contact.B_oriMat*contact.contactPointB
	
	--point(contactPointA + contact.A.pos, vec(1,0,0))
	--point(contactPointB + (contact.B and contact.B.pos or contact.B_pos), vec(0,1,0))

	local penetration = contact.contactNormal ..
		((contactPointB + (contact.B and contact.B.pos or contact.B_pos)) - (contactPointA + contact.A.pos))

	-- Solve normal velocity
	do
		local targetNormalVelChange = -common.getSeparatingVel(
			contact.A, contact.B,
			contactPointA, contactPointB,
			contact.contactMatrix
		).x

		-- Soft constraints
		-- Make bodies push apart like damped spring systems
		local bias, massScale, impulseScale = 0, 1, 0
		
		if penetration < 0 then
			bias = penetration/dt
		elseif useBias then
			local biasRate, massCoeff, impulseCoeff = getDampedSpringValues(dt)

			bias = math.min(4, biasRate * (penetration-0.001))
			massScale, impulseScale = massCoeff, impulseCoeff
		end

		-- Impulse needed to kill separating velocity
		local normalImpulse = (massScale * (targetNormalVelChange + bias) / contact.normalInertia)
			- (impulseScale * contact.accumulatedNormalImpulse)

		-- Clamp accumulated normal impulse so it's non-negative at the end
		local oldAccumNormalImpulse = contact.accumulatedNormalImpulse
		contact.accumulatedNormalImpulse = math.max(contact.accumulatedNormalImpulse + normalImpulse, 0)
		normalImpulse = contact.accumulatedNormalImpulse - oldAccumNormalImpulse

		-- Apply impulse
		local totalImpulseWorld = contact.contactMatrix[1] * normalImpulse
		contact.A:addWorldImpulse(totalImpulseWorld, contactPointA)
		if contact.B then contact.B:addWorldImpulse(-totalImpulseWorld, contactPointB) end
	end

	-- Solve tangent velocity
	do
		local targetTangentVelChange = -common.getSeparatingVel(
			contact.A, contact.B,
			contactPointA, contactPointB,
			contact.contactMatrix
		).yz

		local tangentImpulse = contact.tangentInertia:inverted()*targetTangentVelChange

		-- Clamp accumuated tangent impulse so its length is smaller than (friction*normalImpulse)
		local oldAccumTangentImpulse = contact.accumulatedTangentImpulse
		contact.accumulatedTangentImpulse =
			(contact.accumulatedTangentImpulse + tangentImpulse)
			:clampLength(0, contact.friction*contact.accumulatedNormalImpulse)
		tangentImpulse = contact.accumulatedTangentImpulse - oldAccumTangentImpulse

		-- Apply impulse
		local totalImpulseWorld = contact.contactMatrix * vec(0, tangentImpulse[1], tangentImpulse[2])
		contact.A:addWorldImpulse(totalImpulseWorld, contactPointA)
		if contact.B then contact.B:addWorldImpulse(-totalImpulseWorld, contactPointB) end
	end
end

return function(world)
	local dt = world.stepDuration/world.worldSubsteps

	common.prepareAllConstraints(world)

	world:integrateBodyVelocities(dt)

	common.warmStartAllConstraints(world)
	
	for _ = 1, world.velocityIterations do
		for _, contact in ipairs(world.constraints) do
			solveContact(contact, dt, true)
		end
	end

	world:integrateBodyPositions(dt)

	-- Relaxation: remove excess impulse caused by warmstarting
	for _ = 1, world.positionIterations do
		for _, contact in ipairs(world.constraints) do
			solveContact(contact, dt, false)
		end
	end

	common.storeAllImpulses(world, 1, 1)
end