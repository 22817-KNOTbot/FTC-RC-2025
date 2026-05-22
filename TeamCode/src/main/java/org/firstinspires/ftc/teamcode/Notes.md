# Values
spindexer: 0.4 power; 6,0,0 pid; 128 increment\
shooter power: 0.85; __velocity: 2200 far__, 1760 middle\
intake: 0.8
gate: 0.318 = closed; 0.355 = open; 0.33 = middle\
hood: positive = up. 0.15 = lower; 0.45 = highest\
\
transfer ramp left: lower = out. 0.83 = in; 0.43 = out\
transer ramp right: 0.533 = out; 0.47 = in\
\
mini flipper: 0.87 = out; 0.1 = in (pushing)

brake left: 0.6 = down; 0.5 = up
brake right: 0.39 = down; 0.49 = up

light: 0.28 = red; 0.45 = green; 0.6
\
pto right: 0.5 = disengaged; 0.47 = engaged

# Configuration
## Motors
### Control Hub
0 - Front left\
1 - Back left\
2 - Shooter left\
3 - Shooter right
### Expansion Hub
0 - Front right\
1 - Back right\
2 - Intake\
3 - Transfer
## Servos
### Control Hub
0 - turret 1\
1 - turret 2\
2 - brake right\
3 - brake left\
4 - left intake\
5 - empty
### Expansion Hub
0 - shooter hood\
1 - right intake\
2 - light
## I2C
### Control Hub
2 - pinpoint
3 - colour sensor
## Analog
0/1 - turret axon

# Shooter data
Goal angle = -37 deg
Vertical distance = 37.075 in

Shooter velo	Position (pedro)
1780			(48, 96)
1640			(72, 96)
1760			(72, 72)
1540			(96, 96)
1400			(96, 120)
1540			(72, 120)
1780			(48, 120)
2180			(72, 24)
2120			(96, 9)
2400			(48, 9)
1429			(84, 84) (Close auto)

## Old
Shooter velo	Position (pedro)	Hood angle
2200			(48, 7.25)			50.1320449848
2060			(72, 7.25)			50.964763747
1960			(72, 24)			52.0752668451
1980			(96, 7.25)			51.6771535938
1680			(72, 72)			55.9859280971
1460			(96, 96)			61.5535989797
1620			(72, 96)			58.1620203829
1790			(48, 96)			55.3041306569
1800			(24, 120)			53.6625388433
1680			(48, 120)			56.3607799157
1520			(72, 120)			59.9787192428
1300			(96, 120)			64.905012999

1740			(72, 72)			55.9859280971
1640			(72, 96)			58.1620203829
1520			(72, 120)			58.1620203829
1520			(96, 96)			61.5535989797
1800			(48, 96)			55.3041306569
1760			(48, 120)			56.3607799157