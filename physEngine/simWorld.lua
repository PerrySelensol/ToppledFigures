local ForceGenerators = require("./forceGenerators/forceGens")
local ContactGenerators = require("./contacts/init")

local Solvers = require("physEngine/solvers/init")

--[=============================================================================]--

local BODY_ORDER = {
	box = 1,
	halfSpace = 2,
}

local worldID = 1
local World = {
	worlds = {},

	-- Default world parameters
		solver = "pgs",

		-- In Figura, tick is running at constant speed,
		-- but we can change the duration to frame time if needed
		stepDuration = 1/20,
		worldSubsteps = 2,

		velocityIterations = 4,
		positionIterations = 2
}

function World:new(o)
	o = o or {}
	setmetatable(o, self)
	self.__index = self

	-- Internal world data
	do
		o.isRunning = false
		o.worldPart = models:newPart("physicsDisplay"..worldID, "World")
		worldID = worldID+1

		o.internalIDCount = 1
		o.rigidBodies = {}
		o.constraints = {}
		o.forces = {}
		o.cache = {}
	end


	table.insert(self.worlds, o)
	return o
end

function World:render(delta)
	for _, body in next, self.rigidBodies do
		if not body.noRender then body:render(self.isRunning and delta or 1) end
	end
end

function World:addRigidBody(body)
	table.insert(self.rigidBodies, body)
	body.id = self.internalIDCount
	self.internalIDCount = self.internalIDCount + 1
	if body.renderTask then
		body.renderTask = self.worldPart:newBlock("physDisplay_"..body.id):block(body.renderTask --[[@as string]])
	end
	return body
end

function World:integrateBodyPositions(dt)
	for _, body in ipairs(self.rigidBodies) do
		if not body.colliderOnly then
			body:integratePosition(dt)
			body:calculateDerivedData()
		end
	end
end
function World:integrateBodyVelocities(dt)
	ForceGenerators.updateAllForces(self, dt)
	for _, body in ipairs(self.rigidBodies) do
		if not body.colliderOnly then
			body:integrateVelocity(dt)
		end
	end
end

function World:addConstraint(data)
	assert(data.type, "no type")
	assert(data.A, "no A")
	if data.type == "contact" then
		assert(data.contactPointA, "no contactPointA")
		assert(data.contactPointB, "no contactPointB")
		assert(data.contactNormal, "no normal")
		assert(data.friction, "no friction")
		assert(data.restitution, "no restitution")
	end

	table.insert(self.constraints, data)
end

function World:step(manualStep)
	if not (self.isRunning or manualStep) then return end

	local rigidBodies = self.rigidBodies

	for _, body in ipairs(rigidBodies) do
		if not body.colliderOnly then
			body.render_pos = body.pos
			body.render_ori = body.ori
		end
	end

	for _ = 1, self.worldSubsteps do
		-- Currently uses narrow phase only
		--markBench"collsion"
		for i = 1, #rigidBodies do for j = i+1, #rigidBodies do
			local typeA, typeB = rigidBodies[i].type, rigidBodies[j].type
			if typeA == "halfSpace" and typeB == "halfSpace" then goto endOfLoop end
			if BODY_ORDER[typeA] <= BODY_ORDER[typeB] then
				ContactGenerators[typeA .. typeB](self, rigidBodies[i], rigidBodies[j])
			else
				ContactGenerators[typeB .. typeA](self, rigidBodies[j], rigidBodies[i])
			end
			::endOfLoop::
		end end

		--markBench"solve"
		Solvers[self.solver](self)
		--brint()
	end
end

--[=============================================================================]--

function events.tick()
	for _, w in next, World.worlds do
		w:step()
	end
end

local renderName = host:isHost() and "world_render" or "render"
events[renderName] = function(delta)
	for _, w in next, World.worlds do
		w:render(delta)
	end
end

keybinds:newKeybind("pause/play", "key.keyboard.page.up"):onPress(function()
	for _, w in next, World.worlds do
		w.isRunning = not w.isRunning
	end
end)

keybinds:newKeybind("step", "key.keyboard.end"):onPress(function()
	for _, w in next, World.worlds do
		w:step(true)
	end
end)

--function freezeVel(world)
--	for _, body in next, world.rigidBodies do
--		if not body.colliderOnly then
--			body.vel = vec(0,0,0)
--			body.rot = vec(0,0,0)
--		end
--
--	end
--end

return World