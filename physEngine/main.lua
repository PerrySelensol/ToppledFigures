local World = require("physEngine/simWorld")
local Demos = require("physEngine/demos")

local quatMath = require("physEngine/libs/quaternions")
local Box = require("physEngine/rigidBody/box")
local HalfSpace = require("physEngine/rigidBody/halfSpace")
local ForceGenerators = require("physEngine/forceGenerators/forceGens")

--[=============================================================================]--

local world1 = World:new{
	solver = "pgs_soft",

	stepDuration = 1/20,
	worldSubsteps = 2,

	velocityIterations = 6,
	positionIterations = 0
}

local world2 = World:new{
	solver = "tgs_soft",

	stepDuration = 1/20,
	worldSubsteps = 2,

	velocityIterations = 2,
	positionIterations = 2
}

--world1, world2 = world2, world1

Demos.building(world1)
Demos.building(world2)

-- Offset the display, so the world can be somewhere else other than the origin
local WORLD_OFFSET = vec(5,0.00001,0)
world2.worldPart:pos(16*WORLD_OFFSET)

---[[ Throw cubes
	local thrownCube
	local size, mass = 0.25, 5
	keybinds:newKeybind("throw cube", "key.mouse.right"):onPress(function()
		if not thrownCube then
			thrownCube = world1:addRigidBody(
				Box:new("cyan_terracotta", size, size, size, mass):setRestitution(0):setFriction(0.3)
			)
			ForceGenerators.register(world1, thrownCube, ForceGenerators.gravityForceGen(vec(0,-10,0)))
		end
		local eyePos = player:getPos():add(0,player:getEyeHeight(),0)
		thrownCube
			:setOrientation(quat(1,0,0,0))
			:setPos(eyePos)
			:setVel(player:getLookDir()*10)
			:setAngularVelocity(0,0,0)
	end)
--]]

--printTable(simWorld.rigidBodies)

function events.render()
	--drint(box1.vel, box1.rot)
end
