local PLACE_POS = CGeoPoint:new_local(0,0)
local SHOOT_POS = CGeoPoint:new_local(4000,0)
local TURN_DIR = -math.pi

local rot_speed = 0
local rot_limit = 0
local power = 3000

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
function linedir()
    return function()
        return player.toPointDir(SHOOT_POS, "Leader")
    end
end
function runpos()
    return function()
        return PLACE_POS+CVector(90,0)
    end
end

function getflag()
    return function()
        if bufcnt(player.rotVel("Leader") > rot_limit - 0.08, 20) and math.abs(player.toPointDir(SHOOT_POS, "Leader")-player.dir("Leader"))<2*math.pi/180 then
            return flag.dribble + flag.kick
        end
        return flag.dribble
    end
end

local process = function(power)
    local recordfile = io.open("data/regulation.txt","a")
    omega = rot_limit
    debugEngine:gui_debug_msg(ball.pos(),"giao!!")
    delta_dir = (SHOOT_POS - PLACE_POS):dir() - (ball.pos() - PLACE_POS):dir()
    print(delta_dir)
    need_reg = power * math.tan(delta_dir)
    recordfile:write(omega," ",need_reg,"\n")
end



gPlayTable.CreatePlay{

firstState = "fetchBall",
["fetchBall"] = {
    switch = function()
        if bufcnt(player.toTargetDist("Leader")< 100, 20, 9999) then
            return "turn1"
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
                if ball.toPointDist(PLACE_POS) < 200 and player.infraredOn("Leader") then
                    rot_limit = rot_limit + 2.0
                    if(rot_limit > 12.0) then
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
        debugEngine:gui_debug_line(PLACE_POS, SHOOT_POS, 1)
        debugEngine:gui_debug_msg(CGeoPoint:new_local(-1500,-500),string.format("rotlimit%.2f",rot_limit),1)
        debugEngine:gui_debug_msg(CGeoPoint:new_local(-1500,-650),string.format("power%.2f",power),1)
        if rot_speed > rot_limit - 0.08 then
            rot_speed = rot_limit
        else
            rot_speed = rot_speed + 0.08
        end
        -- if math.abs(player.dir("Leader") - player.toPointDir(SHOOT_POS, "Leader")) < 10 / 180 * math.pi then
        --     return "kick"
        -- end
        -- print("rot_speed",rot_speed)
        if bufcnt(not player.infraredOn("Leader"),9999) then
            -- PLACE_POS = CGeoPoint:new_local((math.random() - 0.5) * 1000, (math.random() - 0.5) * 5000)
            return "fetchBall"
        end
    end,
    Leader = task.openSpeed(0, 0, rotspeed(), 0, getflag(), SHOOT_POS, getpower()),
    match = "{L}"
},
["record"] = {
    switch = function()
        if ball.toPointDist(PLACE_POS) > 2000 then
            process((SHOOT_POS - PLACE_POS):mod() + 1000)
            -- PLACE_POS = CGeoPoint:new_local((math.random() - 0.5) * 1000, (math.random() - 0.5) * 5000)
            return "fetchBall"
        end
    end,
    match = ""
},
-- ["kick"] = {
--     switch = function()
--         if bufcnt(player.kickBall("Leader"), 1 ,300) then
--             return "fetchBall"
--         end
--     end,
--     Leader = task.zget(SHOOT_POS, _, _, flag.kick),
--     -- Leader = task.zbreak(SHOOT_POS, _, _, _),
--     match = "{L}"
-- },

["waitforball"] = {
    switch = function()
        if bufcnt(ball.valid(),20,9999) then
            return "fetchBall"
        end
    end,
    Leader = task.goCmuRush(runpos()),
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
