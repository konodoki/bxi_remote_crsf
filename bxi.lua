local mid = LCD_W / 2
local temp_warn = 80
local temp_limit = 120
local speed_range_channel = "ch10"
local bat_capa = 10 --电池容量Ah
local ghost_bat = 40 --百分之多少的虚电
-- 变量定义
local my_vbatId, my_currId, my_batid, my_capaid, my_fmid
local motorTemps = {} -- 建立温度表
local function init_func()
	-- 获取传感器 ID
	my_vbatId = getFieldInfo("RxBt") and getFieldInfo("RxBt").id or nil
	my_currId = getFieldInfo("Curr") and getFieldInfo("Curr").id or nil
	my_capaid = getFieldInfo("Capa") and getFieldInfo("Capa").id or nil
	my_batid = getFieldInfo("Bat%") and getFieldInfo("Bat%").id or nil
	my_fmid = getFieldInfo("FM") and getFieldInfo("FM").id or nil
end

local function bg_func()
	-- 0x21 帧在 EdgeTX 中通常映射为 "Ttxt" 传感器
	local text = getValue(my_fmid)
	if type(text) == "string" then
		local idx = string.byte(text, 1)
		local t = string.byte(text, 2)
		if idx and t then
			motorTemps[tonumber(idx)] = tonumber(t)
		end
	end
end
local function draw_summary()
	local vbat = getValue(my_vbatId)
	local curr = getValue(my_currId)
	local bat = (getValue(my_batid) - ghost_bat) / (100 - ghost_bat) * 100
	local speed = getValue(my_capaid) / 100 --这个实际存储的是速度信息cm/s
	local display_vbat = string.format("%3.1f", vbat)
	local display_curr = string.format("%3.1f", curr)
	lcd.drawText(0, 0, display_vbat .. "V", 0)
	lcd.drawText(27, 0, display_curr .. "A", 0)
	local display_bat = string.format("%3.1f", bat)
	lcd.drawText(58, 0, display_bat .. "%", 0)
	local capa_time = 0
	if curr ~= 0 then
		local capa = 0
		if curr < 0 then --放电
			capa = bat / 100 * bat_capa
		else --充电
			capa = (100 - bat) / 100 * bat_capa
		end
		capa_time = capa / curr * 60
		local display_capa_time = string.format("%4.1f", capa_time)
		lcd.drawText(85, 0, display_capa_time .. "min", 0)
	end
	local display_speed = string.format("%03.0f", (getValue(speed_range_channel) + 1000) / 10)
	local display_read_speed = string.format("%3.1f", speed)
	local display_km = string.format("%4.2f", math.abs(capa_time) * 60 * speed / 1000)
	lcd.drawText(5, 54, display_speed .. "% " .. display_read_speed .. "m/s " .. display_km .. "km", 0)
	--最大5个电机
	local sortedList = {}
	local count = 0
	for i = 1, #motorTemps do
		if motorTemps[i] then
			count = count + 1
			sortedList[count] = { id = i, temp = motorTemps[i] }
		end
	end
	if count > 1 then
		for i = 1, count - 1 do
			for j = 1, count - i do
				if sortedList[j].temp < sortedList[j + 1].temp then
					-- 交换位置
					local tempNode = sortedList[j]
					sortedList[j] = sortedList[j + 1]
					sortedList[j + 1] = tempNode
				end
			end
		end
	end
	local displayCount = count < 5 and count or 5
	for i = 1, displayCount do
		local y = 1 + (i * 9)
		local motorStr = "M" .. string.format("%02d", sortedList[i].id)
		local tempStr = sortedList[i].temp .. "C"
		lcd.drawText(5, y, motorStr, SMLSIZE)
		local flag = sortedList[i].temp > temp_warn and INVERS or 0
		if sortedList[i].temp >= temp_limit then
			flag = BLINK
		end
		lcd.drawText(35, y, tempStr, flag)
		lcd.drawGauge(65, y, 55, 7, math.min(sortedList[i].temp, temp_limit), temp_limit)
	end
end
local function draw_detail()
	for i = 1, #motorTemps do
		local x = ((i - 1) % 4) * 34
		local y = math.floor((i - 1) / 4) * 8
		local t = motorTemps[i] or "---"
		local flag = 0
		if motorTemps[i] and motorTemps[i] > temp_warn then
			flag = INVERS
		end
		if motorTemps[i] and motorTemps[i] > temp_limit then
			flag = BLINK
		end
		lcd.drawText(x, y, i .. ":" .. t, SMLSIZE + flag)
	end
end
local screens = { draw_summary, draw_detail }
local pageIndex = 1
local function run_func(event)
	lcd.clear()
	if event == EVT_ENTER_BREAK then
		pageIndex = pageIndex + 1
		if pageIndex > #screens then
			pageIndex = 1
		end
	end
	screens[pageIndex]()
	lcd.drawText(114, 58, pageIndex .. "/" .. #screens, SMLSIZE)
	return 0
end

return { run = run_func, init = init_func, background = bg_func }
