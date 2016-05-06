# Rotation controller
import time
# Defining parameters
wheelDiameter = 0.01
carWidth = 0.02

# Variables
x = 0
y = 0
theta = 0

def rotate(angle, time):
	# initialThetaRad = theta
	finalTheta = theta + angle
	initialTime = time.time()
	finalTime = initialTime + time

	while time.time() < finalTime and theta != finalTheta:
		remainingAngle = finalTheta - theta
		remainingTime = finalTime - time.time()
		
		turnsRad = (math.abs(remainingAngle) * carWidth)/(2 * math.pi * wheelDiameter)
		speed = turnsRad/remainingTime
		if angle >= 0:
			setspeeds(speed, -speed)
		elif angle <= 0:
			setspeeds(-speed, speed)
		else:
			setspeeds(0,0)



