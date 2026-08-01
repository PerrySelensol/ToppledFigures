local simWorld = require("physEngine/simWorld")

local quatMath = require("physEngine/libs/quaternions")
local Box = require("physEngine/rigidBody/box")
local HalfSpace = require("physEngine/rigidBody/halfSpace")
local ForceGenerators = require("physEngine/forceGenerators/forceGens")

--[=============================================================================]--

-- Offset the display, so the world can be somewhere else other than the origin
local WORLD_OFFSET = vec(0,0,0)
simWorld.worldPart:pos(16*WORLD_OFFSET)

-- Some predefined orientations, for pointing a certain cube feature downwards
local ROTS = {}; do
	local generateOrthoBasis; do
		local Z1, Z2 = vec(0,0,1), vec(0,1,0)
		local abs, mat3 = math.abs, matrices.mat3
		function generateOrthoBasis(fixedY)
			local genX = fixedY^((abs(fixedY..Z1) < 0.75) and Z1 or Z2)

			genX:normalize()
			local genZ = genX^fixedY

			return mat3(genX, fixedY, genZ)
		end
	end

	local SIGN = {[-1] = "-", [0] = "0", [1] = "+"}
	local function sign(i,j,k) return SIGN[i]..SIGN[j]..SIGN[k] end
	for i = -1, 1 do for j = -1, 1 do for k = -1, 1 do
		--local len = math.abs(i)+math.abs(j)+math.abs(k)
		local axis = vec(i,-j,k):normalized()
		ROTS[sign(i,j,k)] = quatMath.rotMatToQuat(generateOrthoBasis(axis):transposed())
	end end end
end

-- Demos; uncomment to use it
local ground = simWorld:addRigidBody(HalfSpace:new(vec(0,0,0), vec(0,1,0)))

--[[ Box-Box contact test
	local box2 = simWorld:addRigidBody(
		Box:new("dropper", 1, 1, 1, 1):setRestitution(1):setFriction(0)
		:setPos(vec(0.2,2.2,0)):setOrientation(ROTS["++0"])
		:setVel(vec(-0,0,0)):setAngularVelocity(0,0,0)
	)
	--ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-2,0))); --print(box1)
	local box1 = simWorld:addRigidBody(
		Box:new("carved_pumpkin", 1, 1, 1, 10000):setRestitution(1):setFriction(0)
		:setPos(vec(0,1,0)):setOrientation(ROTS["+0+"])
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	--ForceGenerators.register(box2, ForceGenerators.gravityForceGen(vec(0,-2,0)))

--]]

--[[
	local box2 = simWorld:addRigidBody(
		Box:new("glass", 1, 1, 1, 1e20):setRestitution(0.4):setFriction(1)
		:setPos(vec(01,0.5,0)):setOrientation(quat(1,0,0.4,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	--ForceGenerators.register(box2, ForceGenerators.gravityForceGen(vec(0,-10,0)))

	local box3 = simWorld:addRigidBody(
			Box:new("spawner", 1, 1, 1, 1):setRestitution(0.4):setFriction(1)
		:setPos(vec(0,1.5,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box3, ForceGenerators.gravityForceGen(vec(0,-10,0)))
--]]

--[[ Textured cube stack 1
	local width = 1
	local friction = 1

	local box1 = simWorld:addRigidBody(
		Box:new("carved_pumpkin", width, 1, width, 1):setRestitution(0.4):setFriction(friction)
		:setPos(vec(0,0.5,0)):setOrientation(quat(1,0,0.2,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-10,0)))

	local box2 = simWorld:addRigidBody(
		Box:new("dropper", width, 1, width, 1):setRestitution(0.4):setFriction(friction)
		:setPos(vec(0,1.5,0)):setOrientation(quat(1,0,0.4,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box2, ForceGenerators.gravityForceGen(vec(0,-10,0)))

	local box3 = simWorld:addRigidBody(
		Box:new("crafter", width, 1, width, 1):setRestitution(0.4):setFriction(friction)
		:setPos(vec(0,2.5,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box3, ForceGenerators.gravityForceGen(vec(0,-10,0)))

	local box4 = simWorld:addRigidBody(
		Box:new("observer", width, 1, width, 1):setRestitution(0.4):setFriction(friction)
		:setPos(vec(0,3.5,0)):setOrientation(quat(1,0,-0.3,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box4, ForceGenerators.gravityForceGen(vec(0,-10,0)))

	local box5 = simWorld:addRigidBody(
		Box:new("furnace", width, 1, width, 1):setRestitution(0.4):setFriction(friction)
		:setPos(vec(0,4.5,0)):setOrientation(quat(1,0,0.7,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box5, ForceGenerators.gravityForceGen(vec(0,-10,0)))
	
	--local shoot = simWorld:addRigidBody(
	--	Box:new("slime_block", 0.5, 0.5, 0.5, 1):setRestitution(0.4):setFriction(0.5)
	--	:setPos(vec(30,3,0)):setOrientation(quat(1,0,0,0))
	--	:setVel(vec(-3,0,0)):setAngularVelocity(0,0,0)
	--)
--]]

--[[ Textured cube stack 2
	local width = 1
	local friction = 1

	local box1 = simWorld:addRigidBody(
		Box:new("carved_pumpkin", width, 1, width, 1):setRestitution(0.4):setFriction(friction)
		:setPos(vec(0,0.5,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-10,0)))

	local box2 = simWorld:addRigidBody(
		Box:new("dropper", width, 1, width, 1):setRestitution(0.4):setFriction(friction)
		:setPos(vec(0.1,1.5,0.1)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box2, ForceGenerators.gravityForceGen(vec(0,-10,0)))

	local box3 = simWorld:addRigidBody(
		Box:new("crafter", width, 1, width, 1):setRestitution(0.4):setFriction(friction)
		:setPos(vec(0,2.5,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box3, ForceGenerators.gravityForceGen(vec(0,-10,0)))

	local box4 = simWorld:addRigidBody(
		Box:new("observer", width, 1, width, 1):setRestitution(0.4):setFriction(friction)
		:setPos(vec(-0.1,3.5,-0.1)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box4, ForceGenerators.gravityForceGen(vec(0,-10,0)))

	local box5 = simWorld:addRigidBody(
		Box:new("furnace", width, 1, width, 1):setRestitution(0.4):setFriction(friction)
		:setPos(vec(0,4.5,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box5, ForceGenerators.gravityForceGen(vec(0,-10,0)))
	
	--local shoot = simWorld:addRigidBody(
	--	Box:new("slime_block", 0.5, 0.5, 0.5, 1):setRestitution(0.4):setFriction(0.5)
	--	:setPos(vec(30,3,0)):setOrientation(quat(1,0,0,0))
	--	:setVel(vec(-3,0,0)):setAngularVelocity(0,0,0)
	--)
--]]

--[[ Super stacks
	local pallete = {
		"magenta_concrete",
		"pink_concrete",
		"white_concrete",
		"light_blue_concrete",
		"blue_concrete",
	}
	local width = 1
	local friction = 1
	for k = 1, 7 do
		for i = 1, k do
			local j = (i % 2 == 0) and -0.05 or 0.05
			local box = simWorld:addRigidBody(
				Box:new(pallete[((i-1)%5) + 1], width, 1, width, 1):setRestitution(0.4):setFriction(friction)
				:setPos(vec(j+3*k-11,i-0.5,j)):setOrientation(quat(1,0,0,0))
				:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
			)
			ForceGenerators.register(box, ForceGenerators.gravityForceGen(vec(0,-10,0)))
		end
	end
--]]

--[[ Stack of cubes 1
	local pallete = {
		"magenta_concrete",
		"pink_concrete",
		"white_concrete",
		"light_blue_concrete",
		"blue_concrete",
	}
	local width = 1
	local friction = 0.3
	for i = 1, 5 do
		local j = (i % 2 == 0) and -0.05 or 0.05
		local box = simWorld:addRigidBody(
			Box:new(pallete[((i-1)%5) + 1], width, 1, width, 1):setRestitution(0.4):setFriction(friction)
			:setPos(vec(j,i-0.5,j)):setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		ForceGenerators.register(box, ForceGenerators.gravityForceGen(vec(0,-10,0)))
	end
--]]

--[[ Stack of cubes 2
	local pallete = {
		"magenta_concrete",
		"pink_concrete",
		"white_concrete",
		"light_blue_concrete",
		"blue_concrete",
	}
	local width = 1
	local friction = 0.3
	for i = 1, 5 do
		local j = (i % 2 == 0) and 0 or 0.4
		local box = simWorld:addRigidBody(
			Box:new(pallete[((i-1)%5) + 1], width, 1, width, 1):setRestitution(0.4):setFriction(friction)
			:setPos(vec(0,i-0.5,0)):setOrientation(quat(1,0,j,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		ForceGenerators.register(box, ForceGenerators.gravityForceGen(vec(0,-10,0)))
	end
--]]

--[[ Single contact test
	local box1 = simWorld:addRigidBody(
		Box:new("dropper", 1, 1, 1, 1):setRestitution(0):setFriction(0.3)
		:setPos(vec(0,2,0)):setOrientation(ROTS["+++"])
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-10,0)))
--]]

--[[ Intermediate axis theorem
	local v = 2
	local box1 = simWorld:addRigidBody(
		Box:new("gray_concrete", 2, 0.5, 4, 1):setRestitution(1):setFriction(0)
		:setPos(vec(-6,4,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(v,0.00001,0)
	)
	local box2 = simWorld:addRigidBody(
		Box:new("gray_concrete", 2, 0.5, 4, 1):setRestitution(1):setFriction(0)
		:setPos(vec(0,4,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,v,0.00001)
	)
	local box3 = simWorld:addRigidBody(
		Box:new("gray_concrete", 2, 0.5, 4, 1):setRestitution(1):setFriction(0)
		:setPos(vec(6,4,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0.00001,v)
	)
--]]

--[[ Spinning top
	local box1 = simWorld:addRigidBody(
		Box:new("gray_concrete", 1, 1, 1, 1):setRestitution(0):setFriction(0.4)
		:setPos(vec(0,2,0)):setOrientation(ROTS["+++"] + quat(0,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(2,40,0)
	)
	ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-10,0)))
--]]

--[[ Double domino
	local B = {}
	for i = 1, 15 do
		B[i] = simWorld:addRigidBody(
			Box:new("white_concrete", 0.3, 1, 0.7, 1):setRestitution(0):setFriction(0.5)
			:setPos(vec((i*1.01)-8,0.5,0)):setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		ForceGenerators.register(B[i], ForceGenerators.gravityForceGen(vec(0,-5,0)))
	end

	B[1]:setPos(vec(-6.7,0.6,0)):setOrientation(quat(1,0,0,-0.3))
--]]

--[[ Pyramid
	local h = 3
	for k = 1, h do
		for i = 1, k do for j = 1, k do
			local box = simWorld:addRigidBody(
				Box:new("light_blue_concrete", 1, 1, 1, 1):setRestitution(0):setFriction(0.3)
				:setPos(vec(i-k/2,h+0.5-k,j-k/2)):setOrientation(quat(1,0,0,0))
				:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
			)
			ForceGenerators.register(box, ForceGenerators.gravityForceGen(vec(0,-10,0)))
		end end
	end
--]]

---[[ Brick wall
	local width, height = 1.5, 0.75
	local arrayWidth, arrayHeight = 4, 5
	for i = 1, arrayHeight do
		local s = (i%2 == 0)
		for j = 1, arrayWidth do
			local brick = simWorld:addRigidBody(
				Box:new("red_terracotta", width, height, 1, 1):setRestitution(0):setFriction(0.4)
				:setPos(vec(
					-5+(s and 0 or width*0.5)+j*width,
					(i-0.5)*height,
					(s and 0 or 0.1)
				))
				:setOrientation(quat(1,0,0,0))
				:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
			)
			ForceGenerators.register(brick, ForceGenerators.gravityForceGen(vec(0,-10,0)))
		end
		local fill = simWorld:addRigidBody(
			Box:new("orange_terracotta", width/2, height, 1, 1):setRestitution(0):setFriction(0.4)
			:setPos(vec(
				-5+(s and 0 or width*0.5)+(s and (arrayWidth+0.75) or (0.25))*width,
				(i-0.5)*height,
				((i%2 == 0) and 0 or 0.1)
			))
			:setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		ForceGenerators.register(fill, ForceGenerators.gravityForceGen(vec(0,-10,0)))
	end
--]]

---[[ Throw cubes
	local thrownCube
	local size, mass = 0.25, 5
	keybinds:newKeybind("throw cube", "key.mouse.right"):onPress(function()
		if not thrownCube then
			thrownCube = simWorld:addRigidBody(
				Box:new("cyan_terracotta", size, size, size, mass):setRestitution(0):setFriction(0.3)
			)
			ForceGenerators.register(thrownCube, ForceGenerators.gravityForceGen(vec(0,-10,0)))
		end
		local eyePos = player:getPos():add(0,player:getEyeHeight(),0)
		thrownCube
			:setOrientation(quat(1,0,0,0))
			:setPos(eyePos-WORLD_OFFSET)
			:setVel(player:getLookDir()*10)
			:setAngularVelocity(0,0,0)
	end)
--]]

--printTable(simWorld.rigidBodies)

function events.render()
	--drint(box1.vel, box1.rot)
end
