# Dual Heater OpenTherm  
### (c) HomeAccessoryKid 2020-2021
## TLDR
This repo is specific to my house, but you will find a reliable OpenTherm routine synchronized with DS18B20 temperature sensors
and some whacky ideas how to heat a house like mine.

## My House and the heating strategy
I live in a well isolated house with a stone bottom floor with floor heating. The top floor uses radiators in the rooms.
All of this makes either the bottom too warm or the top too cold.

### The Challenge
The heat-capacity of the bottom floor requires e.g. 5 hours of heating at 35 degrees to go up 1 degree.
But overnight it only goes down by 1 degree.
The outside temperature drives the amount of heat lost over time which drives the time needed which in the code is called
`factor` for now. The intention is to make `factor` automatically adjusting, but not yet.  
After having heated 5 hours, it takes one to two hours before things settle. So with three states - `STABLE, HEAT` and `EVAL` - I try
to get the desired temperature without overshooting.  

The top floor is much faster in reacting to heat input. And to make use of the modulation feature of the heater,
I set the water temperature output as a linear function of the top floor setpoint minus measured temperature.

This is not a proper PID but I will fix that later. Maybe the fact that the kettle swithes off and on based on the
fact(?) that it  cannot get rid of the warmth when on the lowest modulation is good enough.

### The Catch
Finally, what to do if the bottom is warm enough and upstairs needs warming?
The bottom floor is using a re-distribution construction with a helper pump. If the helper pump is off, the bottom hardly
gets any heat, which is good enough for this plan.  
The pump is controled by a SonoffBasic with a thermo sensor. Effectivly the pump only runs when there is heat input, saving
a lot of electricity anyway. And because both the heater and the pump have HomeKit, I can inhibit the pump if needed.

### HomeKit automation to bring it together
The heater will change and notify its `current_heater_mode` which, by using Eve, triggers a Scene that changes
the `pump active_state`. The pump will change and notify its `in_use` value which, by using Eve, triggers a Scene that changes
the `target_heater_mode` to cool.
On these two events, the pump will stop for 3 minutes and the heater will know for sure this is true.
The `current` values are set back to allow a new trigger  to be detected.

## The gory details
later... for now see the code


## Version history

- 0.6.5 OT command testing for BLOR and Boiler Sequencer Control
max modulation is controlled by tgt_temp2
- 0.6.4 evaltime calculation and testing OT command support
and some output formatting improvements
- 0.6.3 some bizar leaking of values between tm and seven02
moving heater2 logic before heater1 and capture timestamp string before setting seven02
- 0.6.2 weird stuff with the localtime function
looks like it is a reference to the same object tm as before...
probably 0.6.0 was OK after all except for this code issue
- 0.6.1 re-issue of the 0.6.0 code to load fixed binary  
somehow the binary was compiled before the fix was saved (?)
- 0.6.0 finally fixed the issue of heater2 on in the night  
because tm was abused, subsequent logic failed  
present heater on but pump off as cur_heat1=2 COOL  
initial README
- 0.5.11 and before development cycle