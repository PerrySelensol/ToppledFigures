local quatMath = require("physEngine/libs/quaternions")
require("physEngine/libs/vectors")

--[=============================================================================]--

local ForceGenerators = {}
local forceRegistry = {}

function ForceGenerators.gravityForceGen(gravityAcc)
	return function(body, dt)
		-- Disallow infinite mass
		if body.inverseMass == 0 then return end
		body:addForceAtCenter(gravityAcc/body.inverseMass)
	end
end

function ForceGenerators.register(world, body, generator)
	local body_generator = {body, generator}
	world.forces[body_generator] = true
	return body_generator
end

function ForceGenerators.remove(world, body_generator)
	world.forces[body_generator] = nil
end

function ForceGenerators.updateAllForces(world, dt)
	for data in next, world.forces do
		data[2](data[1], dt)
	end
end

return ForceGenerators