require("physEngine/libs/vectors")
local common = require("./common")

--[=============================================================================]--

local function getDampedSpringValues(dt)
	local damping = 2 -- Damping ratio
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
	local normalImpulse = (massScale * (targetVelChange.x + bias) / contact.normalInertia)
		- (impulseScale * contact.accumulatedNormalImpulse)
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

local function warmStartContact(world, contact)
	local cachedImpulses = world.cache[contact.contactID]
	if not cachedImpulses then return end
	local normalImpulse, tangentImpulse = cachedImpulses[1], cachedImpulses[2]*0.2
	contact.accumulatedNormalImpulse = normalImpulse
	contact.accumulatedTangentImpulse = tangentImpulse

	local contactPointA = contact.A.oriMat*contact.contactPointA
	local contactPointB = contact.B
		and contact.B.oriMat*contact.contactPointB
		or contact.B_oriMat*contact.contactPointB

	local totalImpulseWorld = contact.contactMatrix * vec(normalImpulse, tangentImpulse[1], tangentImpulse[2])
	contact.A:addWorldImpulse(totalImpulseWorld, contactPointA)
	if contact.B then contact.B:addWorldImpulse(-totalImpulseWorld, contactPointB) end
end

return function(world)
	local dt = world.stepDuration/(world.worldSubsteps*world.velocityIterations)

	for _, contact in ipairs(world.constraints) do
		common.prepareContact(contact)
	end

	-- Approximate sub-stepping rather than iterating (contact points are not updated)
	for _ = 1, world.velocityIterations do
		world:integrateBodyVelocities(dt)

		for _, contact in ipairs(world.constraints) do
			warmStartContact(world, contact)
		end

		for _, contact in ipairs(world.constraints) do
			solveContact(contact, dt, true)
		end

		world:integrateBodyPositions(dt)

		-- Relaxation: remove excess impulse caused by warmstarting
		for _ = 1, world.positionIterations do
			for _, contact in ipairs(world.constraints) do
				solveContact(contact, dt, false)
			end
		end
	end

	world.cache = {}
	for i, contact in ipairs(world.constraints) do
		world.cache[contact.contactID] = {contact.accumulatedNormalImpulse, contact.accumulatedTangentImpulse}
		world.constraints[i] = nil
	end

end