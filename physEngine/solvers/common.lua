local quatMath = require("physEngine/libs/quaternions")
require("physEngine/libs/vectors")

--[=============================================================================]--

local solverCommons = {}

-- Generate an arbitary basis with a given fixed axis
local generateOrthoBasis; do
	local Y1, Y2 = vec(1,0,0), vec(0,1,0)
	local abs, mat3 = math.abs, matrices.mat3
	function generateOrthoBasis(fixedX)
		local genZ = fixedX^((abs(fixedX..Y1) < 0.75) and Y1 or Y2)

		genZ:normalize()
		local genY = genZ^fixedX

		return mat3(fixedX, genY, genZ)
	end
end

local function crossMat(v) -- Cross product matrix so that crossMat(v) * u = v ^ u
	return matrices.mat3(
		vec(0,		v.z,	-v.y),
		vec(-v.z,	0,		v.x ),
		vec(v.y,	-v.x,	0   )
	)
end

function solverCommons.getSeparatingVel(A, B, contactPointA, contactPointB, contactMatrix)
	local totalSepVel = A.vel + (A.rot ^ contactPointA)
	if B then
		totalSepVel = totalSepVel - (B.vel + (B.rot ^ contactPointB))
	end
	return contactMatrix:transposed() * totalSepVel
end

local I3 = matrices.mat3()
local function prepareContact(world, contact)
	contact.contactMatrix = generateOrthoBasis(contact.contactNormal)

	local cachedImpulses = world.cache[contact.contactID]
	if cachedImpulses then
		contact.accumulatedNormalImpulse = cachedImpulses[1]
		contact.accumulatedTangentImpulse = cachedImpulses[2]
	else
		contact.accumulatedNormalImpulse = 0
		contact.accumulatedTangentImpulse = vec(0,0)
	end

	local normalInertia
	local tangentInertia

	do
		local linearInertiaA = I3 * contact.A.inverseMass

		local cross_relativeContactPointA = crossMat(contact.A.oriMat * contact.contactPointA)
		local angularImpulsePerLinearImpulse = cross_relativeContactPointA * contact.contactMatrix
		local rotPerUnit = contact.A.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = cross_relativeContactPointA * rotPerUnit * -1
		local angularInertiaA = contact.contactMatrix:transposed() * velPerUnit

		local inertiaA = angularInertiaA + linearInertiaA
		normalInertia = inertiaA[1][1]
		tangentInertia = matrices.mat2(inertiaA[2].yz, inertiaA[3].yz)
	end

	if contact.B then
		local linearInertiaB = I3 * contact.B.inverseMass

		local cross_relativeContactPointB = crossMat(contact.B.oriMat * contact.contactPointB)
		local angularImpulsePerLinearImpulse = cross_relativeContactPointB * contact.contactMatrix
		local rotPerUnit = contact.B.inverseInertiaTensorWorld * angularImpulsePerLinearImpulse
		local velPerUnit = cross_relativeContactPointB * rotPerUnit * -1
		local angularInertiaB = contact.contactMatrix:transposed() * velPerUnit

		local inertiaB = angularInertiaB + linearInertiaB
		normalInertia = normalInertia + inertiaB[1][1]
		tangentInertia = tangentInertia + matrices.mat2(inertiaB[2].yz, inertiaB[3].yz)
	end

	contact.normalInertia = normalInertia
	contact.tangentInertia = tangentInertia
end

function solverCommons.prepareAllConstraints(world)
	for _, constraint in ipairs(world.constraints) do
		if constraint.type == "contact" then prepareContact(world, constraint) end
	end
end

function solverCommons.storeAllImpulses(world)
	world.cache = {}
	for _, constraint in ipairs(world.constraints) do
		if constraint.type == "contact" then
			world.cache[constraint.contactID] = {
				constraint.accumulatedNormalImpulse,
				constraint.accumulatedTangentImpulse
			}
		end
	end
end

function solverCommons.warmStartAllConstraints(world)
	for _, contact in ipairs(world.constraints) do
		local impulse = vec(
			contact.accumulatedNormalImpulse,
			contact.accumulatedTangentImpulse[1],
			contact.accumulatedTangentImpulse[2]
		)

		local contactPointA = contact.A.oriMat*contact.contactPointA
		local contactPointB = contact.B
			and contact.B.oriMat*contact.contactPointB
			or contact.B_oriMat*contact.contactPointB

		local totalImpulseWorld = contact.contactMatrix * impulse
		contact.A:addWorldImpulse(totalImpulseWorld, contactPointA)
		if contact.B then contact.B:addWorldImpulse(-totalImpulseWorld, contactPointB) end
	end
end

return solverCommons