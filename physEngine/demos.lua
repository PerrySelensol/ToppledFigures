local quatMath = require("physEngine/libs/quaternions")

local Box = require("physEngine/rigidBody/box")
local HalfSpace = require("physEngine/rigidBody/halfSpace")
local ForceGenerators = require("physEngine/forceGenerators/forceGens")

--[=============================================================================]--

local Demos = {}

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

function Demos.stacks1(world)
	world:addRigidBody(HalfSpace:new(vec(0,0,0), vec(0,1,0)))
	local pallete = {
		"magenta_concrete",
		"pink_concrete",
		"white_concrete",
		"light_blue_concrete",
		"blue_concrete",
	}
	local width = 1
	local friction = 0.3
	for i = 1, 7 do
		local j = (i % 2 == 0) and -0.05 or 0.05
		local box = world:addRigidBody(
			Box:new(pallete[((i-1)%5) + 1], width, 1, width, 1):setRestitution(0.4):setFriction(friction)
			:setPos(vec(j,i-0.5,j)):setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		ForceGenerators.register(world, box, ForceGenerators.gravityForceGen(vec(0,-10,0)))
	end
end

function Demos.stacks2(world)
	world:addRigidBody(HalfSpace:new(vec(0,0,0), vec(0,1,0)))
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
		local box = world:addRigidBody(
			Box:new(pallete[((i-1)%5) + 1], width, 1, width, 1):setRestitution(0.4):setFriction(friction)
			:setPos(vec(0,i-0.5,0)):setOrientation(quat(1,0,j,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		ForceGenerators.register(world, box, ForceGenerators.gravityForceGen(vec(0,-10,0)))
	end

end

function Demos.intermediateAxis(world)
	local v = 2
	world:addRigidBody(
		Box:new("gray_concrete", 2, 0.5, 4, 1):setRestitution(1):setFriction(0)
		:setPos(vec(-6,4,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(v,0.00001,0)
	)
	world:addRigidBody(
		Box:new("gray_concrete", 2, 0.5, 4, 1):setRestitution(1):setFriction(0)
		:setPos(vec(0,4,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,v,0.00001)
	)
	world:addRigidBody(
		Box:new("gray_concrete", 2, 0.5, 4, 1):setRestitution(1):setFriction(0)
		:setPos(vec(6,4,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0.00001,v)
	)
end

function Demos.super_stacks(world)
	world:addRigidBody(HalfSpace:new(vec(0,0,0), vec(0,1,0)))
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
			local box = world:addRigidBody(
				Box:new(pallete[((i-1)%5) + 1], width, 1, width, 1):setRestitution(0.4):setFriction(friction)
				:setPos(vec(j+3*k-11,i-0.5,j)):setOrientation(quat(1,0,0,0))
				:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
			)
			ForceGenerators.register(world, box, ForceGenerators.gravityForceGen(vec(0,-10,0)))
		end
	end
end

function Demos.double_domino(world)
	world:addRigidBody(HalfSpace:new(vec(0,0,0), vec(0,1,0)))
	local B = {}
	for i = 1, 15 do
		B[i] = world:addRigidBody(
			Box:new("terracotta", 0.3, 1, 0.7, 1):setRestitution(0):setFriction(0.5)
			:setPos(vec((i*1.01)-8,0.5,0)):setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		ForceGenerators.register(world, B[i], ForceGenerators.gravityForceGen(vec(0,-5,0)))
	end

	B[1]:setPos(vec(-6.68,0.6,0)):setOrientation(quat(1,0.001,0,-0.3))
end

function Demos.brick_wall(world)
	world:addRigidBody(HalfSpace:new(vec(0,0,0), vec(0,1,0)))
	local width, height = 1.5, 0.75
	local arrayWidth, arrayHeight = 3, 5
	for i = 1, arrayHeight do
		local s = (i%2 == 0)
		for j = 1, arrayWidth do
			local brick = world:addRigidBody(
				Box:new("red_terracotta", width, height, 1, 1):setRestitution(0):setFriction(0.4)
				:setPos(vec(
					-5+(s and 0 or width*0.5)+j*width,
					(i-0.5)*height,
					(s and 0 or 0.1)
				))
				:setOrientation(quat(1,0,0,0))
				:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
			)
			ForceGenerators.register(world, brick, ForceGenerators.gravityForceGen(vec(0,-10,0)))
		end
		local fill = world:addRigidBody(
			Box:new("orange_terracotta", width/2, height, 1, 1):setRestitution(0):setFriction(0.4)
			:setPos(vec(
				-5+(s and 0 or width*0.5)+(s and (arrayWidth+0.75) or (0.25))*width,
				(i-0.5)*height,
				((i%2 == 0) and 0 or 0.1)
			))
			:setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		ForceGenerators.register(world, fill, ForceGenerators.gravityForceGen(vec(0,-10,0)))
	end

end

function Demos.pyramid(world)
	world:addRigidBody(HalfSpace:new(vec(0,0,0), vec(0,1,0)))
	local h = 3
	for k = 1, h do
		for i = 1, k do for j = 1, k do
			local box = world:addRigidBody(
				Box:new("light_blue_concrete", 1, 1, 1, 1):setRestitution(0):setFriction(0.3)
				:setPos(vec(i-k/2,h+0.5-k,j-k/2)):setOrientation(quat(1,0,0,0))
				:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
			)
			ForceGenerators.register(world, box, ForceGenerators.gravityForceGen(vec(0,-10,0)))
		end end
	end
end

function Demos.top(world)
	world:addRigidBody(HalfSpace:new(vec(0,0,0), vec(0,1,0)))
	local box1 = world:addRigidBody(
		Box:new("gray_concrete", 1, 1, 1, 1):setRestitution(0):setFriction(0.4)
		:setPos(vec(0,2,0)):setOrientation(ROTS["+++"] + quat(0,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(2,40,0)
	)
	ForceGenerators.register(world, box1, ForceGenerators.gravityForceGen(vec(0,-10,0)))
end

function Demos.building(world)
	local LAYERS = 2
	world:addRigidBody(HalfSpace:new(vec(0,0,0), vec(0,1,0)))
	local B = {}
	for i = 0, LAYERS-1 do
		B[1+5*i] = world:addRigidBody(
			Box:new("light_gray_concrete", 0.5, 3, 0.5, 1):setRestitution(0):setFriction(0.4)
			:setPos(vec(2,1.5+i*3.5,2)):setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		B[2+5*i] = world:addRigidBody(
			Box:new("light_gray_concrete", 0.5, 3, 0.5, 1):setRestitution(0):setFriction(0.4)
			:setPos(vec(-2,1.5+i*3.5,2)):setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		B[3+5*i] = world:addRigidBody(
			Box:new("light_gray_concrete", 0.5, 3, 0.5, 1):setRestitution(0):setFriction(0.4)
			:setPos(vec(2,1.5+i*3.5,-2)):setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		B[4+5*i] = world:addRigidBody(
			Box:new("light_gray_concrete", 0.5, 3, 0.5, 1):setRestitution(0):setFriction(0.4)
			:setPos(vec(-2,1.5+i*3.5,-2)):setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
		B[5+5*i] = world:addRigidBody(
			Box:new("light_gray_concrete", 5, 0.5, 5, 1):setRestitution(0):setFriction(0.4)
			:setPos(vec(0,3.25+i*3.5,0)):setOrientation(quat(1,0,0,0))
			:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
		)
	end
	B[0] = world:addRigidBody(
		Box:new("bedrock", 4, 4, 4, 20):setRestitution(0):setFriction(0.4)
		:setPos(vec(0,2+LAYERS*3.5,0)):setOrientation(quat(1,0,0,0))
		:setVel(vec(0,0,0)):setAngularVelocity(0,0,0)
	)
	for i = 0, #B do
		ForceGenerators.register(world, B[i], ForceGenerators.gravityForceGen(vec(0,-10,0)))
	end
end

return Demos