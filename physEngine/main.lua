local World = require("physEngine/simWorld")
local Demos = require("physEngine/demos")

local quatMath = require("physEngine/libs/quaternions")
local Box = require("physEngine/rigidBody/box")
local HalfSpace = require("physEngine/rigidBody/halfSpace")
local ForceGenerators = require("physEngine/forceGenerators/forceGens")

--[=============================================================================]--

local worldPGS = World:new{
	solver = "pgs_soft",

	stepDuration = 1/20,
	worldSubsteps = 2,

	velocityIterations = 4,
	positionIterations = 2
}
--SolveLoops = worldPGS.velocityIterations + worldPGS.positionIterations

local worldTGS = World:new{
	solver = "tgs_soft",

	stepDuration = 1/20,
	worldSubsteps = 2,

	velocityIterations = 3,
	positionIterations = 1
}
--SolveLoops = worldTGS.velocityIterations*(1 + worldTGS.positionIterations)

world1, world2 = worldTGS, worldPGS
--world1, world2 = worldPGS, worldTGS

Demos.building(world1)
Demos.building(world2)
--drint(world1.solver)



world2.worldPart:pos(16*vec(8,0.00001,0))

-- Throw cubes
do
	local thrownCube
	local size, mass = 1, 1
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
end

function events.render()
	--drint(box1.vel, box1.rot)
end
