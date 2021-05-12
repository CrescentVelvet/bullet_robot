local PLACE_POS = {
    CGeoPoint:new_local(1000,0),
    CGeoPoint:new_local(1000,500)
}
local SHOOT_POS = CGeoPoint:new_local(4000,0)
local TURN_DIR = 0.0

local rot_speed = 0.0
local slide_limit = 300
local rot_first = -1
local rot_limit = rot_first
local power = 4000
local kick_x = 0.0
local kick_y = 0.0
local dir_car = 0.0
-- 2 for Slide move, 1 for Rotate move
local mode = 1

function getpower()
    return function()
        return power
    end
end
function rotspeed()
    return function()
        return rot_speed
    end
end
function slidespeed()
    return function()
        return -slide_limit
    end
end
function slidedir()
    return function()
        return -slide_limit/5000
    end
end
function linedir()
    return function()
        return player.toPointDir(SHOOT_POS, "Leader")
    end
end
function runpos()
    return function()
        return PLACE_POS[mode] + CVector(-math.cos(player.dir("Leader")), -math.sin(player.dir("Leader")))*98
    end
end

function getflag()
    return function()
        -- print(player.rawVelMod("Leader"))
        if mode == 1 and math.abs(player.rotVel("Leader")) > math.abs(rot_limit*0.9) then
            if  (math.abs(rot_limit) < 0.1 and math.abs(player.rotVel("Leader"))<0.1 and  math.abs(player.dir("Leader"))<0.1) or (math.abs(rot_limit) > 0.1 and  math.abs(player.toPointDir(SHOOT_POS, "Leader")-player.dir("Leader"))<(math.abs(rot_limit))*2/75) then
                kick_x = player.posX("Leader")
                kick_y = player.posY("Leader")
                dir_car = player.dir("Leader")
                return flag.dribble + flag.kick
            end
        -- if mode == 2 and math.abs(player.posY("Leader")) < slide_limit/75 then
        elseif mode == 2 and player.rawVelMod("Leader") > slide_limit*0.9 and math.abs(player.dir("Leader"))<math.pi/10 then
            kick_x = player.posX("Leader")
            kick_y = player.posY("Leader")
            dir_car = player.dir("Leader")
            return flag.dribble + flag.kick
        end
        return flag.dribble
    end
end

local use_dir = function()
    return function()
        print("usedir:",rot_limit)
        if rot_limit < 0.1 and rot_limit > -0.1 then
            print(1)
            return 1
        end
        return 0
    end
end

local process = function(power)
    local recordfile = io.open("data/regulation.txt","a")
    omega = rot_limit
    delta_dir = (SHOOT_POS - PLACE_POS[1]):dir() - (ball.pos() - PLACE_POS[1]):dir()
    -- print(delta_dir)
    need_reg = power * math.tan(delta_dir)
    recordfile:write(omega," ",need_reg,"\n")
end

local savedata = function ()
    debugEngine:gui_debug_msg(ball.pos(),"savedata!!!")
    if mode == 1 then
        local recordfile = io.open("data/ReguDataRotate.txt", "a")
        print(" ",rot_limit," ",power," ",ball.posX()-kick_x," ",ball.posY()-kick_y," ",dir_car)
        recordfile:write(" ",rot_limit," ",power," ",ball.posX()-kick_x," ",ball.posY()-kick_y," ",dir_car,"\n")
    else
        local recordfile = io.open("data/ReguDataSlide.txt", "a")
        print(" ",slide_limit," ",power," ",ball.posX()-kick_x," ",ball.posY()-kick_y," ",dir_car)
        recordfile:write(" ",slide_limit," ",power," ",ball.posX()-kick_x," ",ball.posY()-kick_y," ",dir_car,"\n")
    end
end

gPlayTable.CreatePlay{

firstState = "fetchBall",
["fetchBall"] = {
    switch = function()
        if player.toTargetDist("Leader") < 100 and player.infraredOn("Leader") then
            if mode == 1 then
                return "turn1"
            else
                return "slide1"
            end
        end
    end,
    Leader = task.fetchBall(runpos(),0,flag.safe),
    match = "{L}"
},
["turn1"] = {
    switch = function()
        rot_speed = 0
        if not player.infraredOn("Leader") then
            return "fetchBall"
        else
            if player.toTargetDist("Leader")< 200 then
                if ball.toPointDist(PLACE_POS[1]) < 200 and player.infraredOn("Leader") then
                    rot_limit = rot_limit + 1.0
                    if(rot_limit > 12.0) then
                        power = power + 500
                        rot_limit = rot_first + 1.0
                    end
                    if(power > 6000) then
                        return "waitforball"
                    end
                    return "turn2"
                end
            end
        end
    end,
    Leader = task.goCmuRush(runpos(), TURN_DIR, _, flag.dribbling),
    match = "{L}"
},
["turn2"] = {
    switch = function()
        debugEngine:gui_debug_line(PLACE_POS[1], SHOOT_POS, 1)
        debugEngine:gui_debug_msg(CGeoPoint:new_local(-1500,-650),string.format("power%.2f",power),1)
        debugEngine:gui_debug_msg(CGeoPoint:new_local(-1500,-500),string.format("limit%.2f",rot_limit),1)
        if rot_speed > rot_limit - 0.08 then
            rot_speed = rot_limit
        else
            rot_speed = rot_speed + 0.08
        end
        if player.kickBall("Leader") then
            return "record"
        end
        if not player.infraredOn("Leader") then
            return "fetchBall"
        end
    end,
    Leader = task.openSpeed(0, 0, rotspeed(), use_dir(), getflag(), SHOOT_POS, getpower()),
    match = "{L}"
},
["slide1"] = {
    switch = function()
        if not player.infraredOn("Leader") then
                return "fetchBall"
        else
            if player.toTargetDist("Leader")< 200 then
                if ball.toPointDist(PLACE_POS[2]) < 200 and player.infraredOn("Leader") then
                    slide_limit = slide_limit + 300
                    if(slide_limit > 3900) then
                        power = power + 500
                        slide_limit = 500
                    end
                    if power > 6000 then
                        return "waitforball"
                    end
                    return "slide2"
                end
            end
        end
    end,
    Leader = task.goCmuRush(runpos(), TURN_DIR, _, flag.dribbling),
    match = "{L}"
},

["slide2"] = {
    switch = function()
        debugEngine:gui_debug_line(PLACE_POS[1], SHOOT_POS, 1)
        debugEngine:gui_debug_msg(CGeoPoint:new_local(-1500,-500),string.format("slidelimit%.2f",slide_limit),1)
        debugEngine:gui_debug_msg(CGeoPoint:new_local(-1500,-650),string.format("power%.2f",power),1)
        if player.kickBall("Leader") then
            return "record"
        end
        if not player.infraredOn("Leader") then
            return "fetchBall"
        end
        -- if player.posY("Leader") < -500 then
        --     slide_limit = slide_limit - 500
        --     return "slide1"
        -- end
    end,
    Leader = task.openSpeed(0, slidespeed(), 0, use_dir(), getflag(), SHOOT_POS, getpower()),
    match = "{L}"
},

["record"] = {
    switch = function()
        if math.abs(ball.posX()-kick_x) > 1000 then
            -- process((SHOOT_POS - PLACE_POS):mod() + 1000)
            savedata()
            return "fetchBall"
        end
    end,
    match = ""
},

["waitforball"] = {
    switch = function()
    end,
    Leader = task.stop(),
    match = "{L}"
},

name = "RegulationData",
applicable ={
    exp = "a",
    a = true
},
attribute = "attack",
timeout = 99999
}

