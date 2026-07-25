local ForceGenerators = require("physEngine/forceGenerators/forceGens")
local quatMath = require("physEngine/libs/quaternions")
require("physEngine/libs/vectors")
local common = require("./common")

--[=============================================================================]--

--//TODO fix friction impulse ruining box stacks
local function solveContact(contact, dt, useBias)
	-- Convert contact point to world orientation, but local position
	local contactPointA = contact.A.oriMat*contact.contactPointA
	local contactPointB = contact.B and contact.B.oriMat*contact.contactPointB or contact.B_oriMat*contact.contactPointB

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

	-- Skip contact pairs that aren't penetrating (with small tolerance)
	--if penetration < 0 then return end
	--point(contactPointA + contact.A.pos)
	--point(contactPointA + contact.A.pos + contactShift, vec(0,0,0))

	-- Baumgarte Stabilization
	-- This bias, proportional to penetration depth, is added to total nomral impulse
	-- Allows penetrating bodies to push themselves apart
	local normalBias = 0
	if penetration < 0 then
		normalBias = penetration/dt
	elseif useBias then
		normalBias = math.min(2, (penetration) * (0.8/dt))
	end
	local tangentBias = useBias and (0.5/dt)*slid or vec(0,0)

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

return function(world)
	local dt = world.stepDuration/world.worldSubsteps

	for _, contact in ipairs(world.constraints) do
		common.prepareContact(contact)
	end

	for _ = 1, world.velocityIterations do

		local h = dt/world.velocityIterations

		world:integrateBodyVelocities(h)

		for i, contact in ipairs(world.constraints) do
			solveContact(contact, h, true)
		end

		world:integrateBodyPositions(h)

	end

	---[[
		for _ = 1, world.positionIterations do

			local h = dt/world.positionIterations

			world:integrateBodyVelocities(h)

			for i, contact in ipairs(world.constraints) do
				solveContact(contact, h, false)
			end

		end
	--]]

	for i = 1, #world.constraints do world.constraints[i] = nil end

end