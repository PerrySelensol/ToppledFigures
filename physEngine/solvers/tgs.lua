local ForceGenerators = require("physEngine/forceGenerators/forceGens")
local quatMath = require("physEngine/libs/quaternions")
require("physEngine/libs/vectors")
local common = require("./common")

--[=============================================================================]--

-- //TODO add restitution
local function solveContact(contact, dt, useBias)
	-- Convert contact point to world orientation, but local position
	local contactPointA = contact.A.oriMat*contact.contactPointA
	local contactPointB = contact.B
		and contact.B.oriMat*contact.contactPointB
		or contact.B_oriMat*contact.contactPointB

	local contactShift =
		(contactPointB + (contact.B and contact.B.pos or contact.B_pos))
		- (contactPointA + contact.A.pos)

	local contactSlid = contact.contactMatrix:transposed()*contactShift
	local penetration = contactSlid.x
	local slid = contactSlid.yz

	-- This is our target change in separating velocity after collision
	-- i.e. we need to fully zero out the separating velocity
	local targetVelChange = -common.getSeparatingVel(
		contact.A, contact.B,
		contactPointA, contactPointB,
		contact.contactMatrix
	)
	---@cast targetVelChange Vector3

	--point(contactPointA + contact.A.pos)
	--point(contactPointA + contact.A.pos + contactShift, vec(0,0,0))

	-- Baumgarte Stabilization
	-- This bias, proportional to penetration depth, is added to total nomral impulse
	-- Allows penetrating bodies to push themselves apart
	local normalBias = 0
	if penetration < 0 then
		normalBias = penetration/dt
	elseif useBias then
		normalBias = math.min(4, math.max(0, (penetration - 0.005)) * (0.1/dt))
	end
	local tangentBias = useBias and (0.2/dt)*slid or vec(0,0)

	-- Impulse needed to kill separating velocity
	local normalImpulse = (targetVelChange.x + normalBias) / contact.normalInertia
	local tangentImpulse = contact.tangentInertia:inverted()*(targetVelChange.yz + tangentBias)

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

local contactImpulses = {}

local function warmStartContact(contact)
	local cachedImpulses = contactImpulses[contact.contactID]
	if not cachedImpulses then return end
	local normalImpulse, tangentImpulse = cachedImpulses[1]*0.9, cachedImpulses[2]*0.2
	contact.accumulatedNormalImpulse = normalImpulse
	contact.accumulatedTangentImpulse = tangentImpulse

	local contactPointA = contact.A.oriMat*contact.contactPointA
	local contactPointB = contact.B and contact.B.oriMat*contact.contactPointB or contact.B_oriMat*contact.contactPointB

	local totalImpulseWorld = contact.contactMatrix * vec(normalImpulse, tangentImpulse[1], tangentImpulse[2])
	contact.A:addWorldImpulse(totalImpulseWorld, contactPointA)
	if contact.B then contact.B:addWorldImpulse(-totalImpulseWorld, contactPointB) end
end

return function(world)
	local dt = world.stepDuration/(world.worldSubsteps*world.velocityIterations)

	for _, contact in ipairs(world.constraints) do
		common.prepareContact(contact)
		warmStartContact(contact)
	end
	contactImpulses = {}

	-- Approximate sub-stepping rather than iterating (contact points are not updated)
	for _ = 1, world.velocityIterations do
		world:integrateBodyVelocities(dt)

		for _, contact in ipairs(world.constraints) do
			solveContact(contact, dt, true)
		end

		world:integrateBodyPositions(dt)
	end

	-- Relaxation: remove excess impulse caused by Baumgarte
	for _ = 1, world.positionIterations do
		for _, contact in ipairs(world.constraints) do
			solveContact(contact, dt, false)
		end
	end

	for i, contact in ipairs(world.constraints) do
		contactImpulses[contact.contactID] = {contact.accumulatedNormalImpulse, contact.accumulatedTangentImpulse}
		world.constraints[i] = nil
	end

end