require("sybot_lib")

-- Load the robot
load_rob("res/SyArm_Mk1.conf.json")
print_rob()
--
local pos = { x = 100, y = 300, z = 300 }
local test = true

if test then 
    move_j()
end