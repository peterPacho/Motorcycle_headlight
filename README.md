## Description
Riding at night is fun but I really didn't like the fact that I couldn't see where I'm going when turning.
This project aims to solve that issue by creating a headlight housing that can rotate, counteracting the motorcycle's lean during the turn, making the projected beam always parallel to the road surface.

Headlight bucket should fit most “Jeep” 7 inch headlights.

[V1 version demo video](https://www.youtube.com/watch?v=rBPONPm-cCk)

There are two MakerFocus TF-Luna lidars attached to the rear of the bike. Distance to the road surface is used to estimate the lean angle. I'm using Lidars since gyro/accelerometers proved to be difficult - I didn't find a way to counteract the centrifugal acceleration during the turn and couldn't obtain reliable data.

### Noncomprehensive parts list:
-	Arduino (and supporting components like resistors, voltage regulator, capacitors)
-	TMC2209 stepper motor driver
-	“Short body” NEMA 17 stepper motor
-	16x2 LCD Display
-	608ZZ skateboard bearings (8x22x7 mm)

![photo](https://github.com/peterPacho/Motorcycle_headlight/blob/master/CAD%20Files/1.png?raw=true)
![photo](https://github.com/peterPacho/Motorcycle_headlight/blob/master/CAD%20Files/2.png?raw=true)
![photo](https://github.com/peterPacho/Motorcycle_headlight/blob/master/CAD%20Files/3.png?raw=true)
![photo](https://github.com/peterPacho/Motorcycle_headlight/blob/master/CAD%20Files/4.png?raw=true)
![photo](https://github.com/peterPacho/Motorcycle_headlight/blob/master/CAD%20Files/5.png?raw=true)
![photo](https://github.com/peterPacho/Motorcycle_headlight/blob/master/CAD%20Files/6.png?raw=true)
