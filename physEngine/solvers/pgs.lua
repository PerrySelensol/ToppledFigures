require("physEngine/libs/vectors")
local common = require("./common")

--[=============================================================================]--

-- //TODO add restitution
local function solveContact(contact, dt)
	-- Convert contact point to world orientation, but local position
	local contactPointA = contact.A.oriMat*contact.contactPointA
	local contactPointB = contact.B
		and contact.B.oriMat*contact.contactPointB
		or contact.B_oriMat*contact.contactPointB

	local penetration = contact.contactNormal ..
		((contactPointB + (contact.B and contact.B.pos or contact.B_pos)) - (contactPointA + contact.A.pos))

	-- This is our target change in separating velocity after collision
	-- i.e. we need to fully zero out the separating velocity
	local targetVelChange = -common.getSeparatingVel(
		contact.A, contact.B,
		contactPointA, contactPointB,
		contact.contactMatrix
	)
	---@cast targetVelChange Vector3

	--point(contactPointA + contact.A.pos, vec(1,0,0))
	--point(contactPointB + (contact.B and contact.B.pos or contact.B_pos), vec(0,1,0))
	
	-- Baumgarte Stabilization
	-- This bias, proportional to penetration depth, is added to total nomral impulse
	-- Allows penetrating bodies to push themselves apart
	local bias
	if penetration >= 0 then
		bias = math.max(0, (penetration - 0.005)) * (0.2/dt)
	else
		bias = penetration/dt
	end

	-- Impulse needed to kill separating velocity
	local normalImpulse = (targetVelChange.x + bias) / contact.normalInertia
	local tangentImpulse = contact.tangentInertia:inverted()*targetVelChange.yz

	-- Clamp accumulated normal impulse so it's non-negative at the end
	local oldAccumNormalImpulse = contact.accumulatedNormalImpulse
	contact.accumulatedNormalImpulse = math.max(contact.accumulatedNormalImpulse + normalImpulse, 0)
	normalImpulse = contact.accumulatedNormalImpulse - oldAccumNormalImpulse

	-- Clamp accumuated tangent impulse so its length is smaller than (friction*normalImpulse)
	local oldAccumTangentImpulse = contact.accumulatedTangentImpulse
	contact.accumulatedTangentImpulse =
		(contact.accumulatedTangentImpulse + tangentImpulse)
		:clampLength(0, contact.friction*contact.accumulatedNormalImpulse)
	tangentImpulse = contact.accumulatedTangentImpulse - oldAccumTangentImpulse

	-- Convert impulse to world space then applying it to both bodies
	local totalImpulseWorld = contact.contactMatrix * vec(normalImpulse, tangentImpulse[1], tangentImpulse[2])
	contact.A:addWorldImpulse(totalImpulseWorld, contactPointA)
	if contact.B then contact.B:addWorldImpulse(-totalImpulseWorld, contactPointB) end
end

return function(world)
	local dt = world.stepDuration/world.worldSubsteps

	common.prepareAllConstraints(world)

	world:integrateBodyVelocities(dt)

	common.warmStartAllConstraints(world)
	
	for _ = 1, world.velocityIterations do
		for _, contact in ipairs(world.constraints) do
			solveContact(contact, dt)
		end
	end

	world:integrateBodyPositions(dt)

	-- Relaxation: remove excess impulse caused by warmstarting
	for _ = 1, world.positionIterations do
		for _, contact in ipairs(world.constraints) do
			solveContact(contact, dt)
		end
	end

	common.storeAllImpulses(world, 1, 1)
end