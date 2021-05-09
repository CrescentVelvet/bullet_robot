local PLACE_POS = {
    CGeoPoint:new_local(1000,0),
    CGeoPoint:new_local(1000,2200)
}
local SHOOT_POS = CGeoPoint:new_local(4000,0)
local TURN_DIR = 0

local rot_speed = 0.0
local slide_limit = 500
local rot_limit = 1.0
local power = 3500
local kick_x = 0.0
local kick_y = 0.0
-- 2 for slide move, 1 for rotate move
local mode = 2  

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

function linedir()
    return function()
        return player.toPointDir(SHOOT_POS, "Leader")
    end
end
function runpos()
    return function()
        return PLACE_POS[mode]+CVector(-90,0)
    end
end

function getflag()
    return function()
        if mode == 1 and bufcnt(player.rotVel("Leader") > rot_limit - 0.08, 20) and math.abs(player.toPointDir(SHOOT_POS, "Leader")-player.dir("Leader"))<rot_limit/75.0 then
            kick_x = player.posX("Leader")
            kick_y = player.posY("Leader")
            return flag.dribble + flag.kick
        end
        if mode ==2 and (math.abs(player.toPointDir(SHOOT_POS, "Leader")-player.dir("Leader"))<2/180*math.pi and math.abs(player.posY("Leader")) < slide_limit/75) then
            return flag.dribble + flag.kick
        end
        return flag.dribble
    end
end

local process = function(power)
    local recordfile = io.open("data/regulation.txt","a")
    omega = rot_limit
    delta_dir = (SHOOT_POS - PLACE_POS[1]):dir() - (ball.pos() - PLACE_POS[1]):dir()
    print(delta_dir)
    need_reg = power * math.tan(delta_dir)
    recordfile:write(omega," ",need_reg,"\n")
end

local savedata = function (rotvel, power)
    if mode == 1 then
        debugEngine:gui_debug_msg(ball.pos(),"savedata!!!")
        local recordfile = io.open("data/ReguDataRotate.txt", "a")
        print(rotvel, power, ball.posY() - kick_y)
        recordfile:write(" ",rotvel," ",power," ",ball.posX()-kick_x," ",ball.posY()-kick_y,"\n")
    else
        local recordfile = io.open("data/ReguDataSlide.txt", "a")
        print(rotvel, power, ball.posY() - kick_y)
        recordfile:write(" ",rotvel," ",power," ",ball.posX()-kick_x," ",ball.posY()-kick_y,"\n")
    end
end

gPlayTable.CreatePlay{

firstState = "fetchBall",
["fetchBall"] = {
    switch = function()
        if bufcnt(player.toTargetDist("Leader")< 150, 20, 9999) then
            if mode == 1 then
                return "turn1"
            else
                return "slide1"
            end
        end
        if bufcnt(not ball.valid(),60,9999) then
            return "waitforball"
        end
    end,
    Leader = task.fetchBall(runpos(),0,true),
    match = "{L}"
},
["turn1"] = {
    switch = function()
        rot_speed = 0
        if bufcnt(not player.infraredOn("Leader"),30,9999) then
            return "fetchBall"
        else
            if player.toTargetDist("Leader")< 200 then
                if ball.toPointDist(PLACE_POS[1]) < 200 and player.infraredOn("Leader") then
                    rot_limit = rot_limit + 1.0
                    if(rot_limit > 8.0) then
                        power = power + 500
                        rot_limit = 2.0
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
        debugEngine:gui_debug_msg(CGeoPoint:new_local(-1500,-500),string.format("rotlimit%.2f",rot_limit),1)
        debugEngine:gui_debug_msg(CGeoPoint:new_local(-1500,-650),string.format("power%.2f",power),1)
        if rot_speed > rot_limit - 0.08 then
            rot_speed = rot_limit
        else
            rot_speed = rot_speed + 0.08
        end
        if bufcnt(not player.infraredOn("Leader"),9999) then
            return "fetchBall"
        end
        if player.kickBall("Leader") then
            return "record"
        end
    end,
    Leader = task.openSpeed(0, 0, rotspeed(), 0, getflag(), SHOOT_POS, getpower()),
    match = "{L}"
},
["slide1"] = {
    switch = function()
        if not player.infraredOn("Leader") then
                return "fetchBall"
        else
            if bufcnt(player.toTargetDist("Leader")< 200,120) then
                if ball.toPointDist(PLACE_POS[2]) < 200 and player.infraredOn("Leader") then
                    slide_limit = slide_limit + 500
                    if(slide_limit > 4500) then
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
        if bufcnt(not player.infraredOn("Leader"),9999) then
            return "fetchBall"
        end
        if player.kickBall("Leader") then
            return "record"
        end
        if player.posY("Leader") < -500 then
            slide_limit = slide_limit - 500
            return "slide1"
        end
    end,
    Leader = task.openSpeed(0, slidespeed(), 0, 1, getflag(), SHOOT_POS, getpower()),
    match = "{L}"
},

["record"] = {
    switch = function()
        if (ball.posX()-kick_x) > 1000 then
            -- process((SHOOT_POS - PLACE_POS):mod() + 1000)
            savedata(rot_limit, power)
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
