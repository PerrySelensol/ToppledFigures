local simWorld = require("physEngine/simWorld")

local quatMath = require("physEngine/libs/quaternions")
local Box = require("physEngine/rigidBody/box")
local HalfSpace = require("physEngine/rigidBody/halfSpace")
local ForceGenerators = require("physEngine/forceGenerators/forceGens")

--[=============================================================================]--

local ground = simWorld:addRigidBody(HalfSpace:new(vec(0,0,0), vec(0,1,0)))

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

--[[
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

--[[
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

--[[
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

--[[
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

---[[
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

--[[
	local box1 = simWorld:addRigidBody(
		Box:new("dropper", 1, 1, 1, 1):setRestitution(0):setFriction(0.3)
		:setPos(vec(0,2,0)):setOrientation(ROTS["+++"])
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-10,0)))
--]]

--[[
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

--[[
	local box1 = simWorld:addRigidBody(
		Box:new("gray_concrete", 1, 1, 1, 1):setRestitution(0):setFriction(1)
		:setPos(vec(0,2,0)):setOrientation(ROTS["+++"] + quat(0,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(2,40,0)
	)
	ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-10,0)))
--]]

--[[
	local box1 = simWorld:addRigidBody(
		Box:new("dropper", 1, 1, 1, 1):setRestitution(0):setFriction(1)
		:setPos(vec(0,0.5,0)):setOrientation(ROTS["---"])
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	ForceGenerators.register(box1, ForceGenerators.gravityForceGen(vec(0,-10,0)))
--]]

--[[
	local B = {}
	for i = 1, 15 do
		B[i] = simWorld:addRigidBody(
			Box:new("white_concrete", 0.2, 1, 0.5, 1):setRestitution(0):setFriction(1)
			:setPos(vec((i*1.01)-8,0.5,0)):setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		ForceGenerators.register(B[i], ForceGenerators.gravityForceGen(vec(0,-3,0)))
	end

	B[1]:setPos(vec(-6.78,0.55,0)):setOrientation(quat(1,0,0,-0.2))
--]]

--printTable(simWorld.rigidBodies)

function events.render()
	--drint(box1.vel, box1.rot)
end
