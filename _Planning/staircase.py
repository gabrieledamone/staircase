'''
Name: staircase.py
Last edited by/on: FAM 04/03/19 12:10
Description: building a staircase from stations
from right to left
arranged in form of staircase levels
low redundancy by less movement
objective is to avoid up and down movements and stay as much same level as possible while working in stations
have different level stations at different positions on the workspace
have ready made arrays for brick stations dependent on height
'''
import map as map

### building a staircase from ... (wait for it) staircase height stations!

## preparation

height = int(input("Height of Staircase: ")) # ask for height of staircase
print(height)
print("Building staircase of height" + str(height)) # output for transparency
# if height >= 5: print("Sorry our robot is lazy today") # lazy
# heightAmount = {1: 1,2: 3,3: 6,4: 10,5: 15} # define amount of bricks necessary dependent on height
# brickNums = heightAmount(height) # count amount of bricks necessary = length of location array]

locationDestinationOptions = [map.one, map.two, map.three, map.four, map.five] # load options array of height station maps, order of bricks is order of pick up: right to left view from top

# heightMap = [[],[],[],[]]

locationDestinationMap = locationDestinationOptions[height-1] # read location/destination array from options array from height with locations in order of all bricks (according to logic) and where they need to go
# run function which uses locationDestinationMap to place bricks in Gazebo
station = [i[0] for i in locationDestinationMap] # define station array listing starting locations for bricks
destination = [j[1] for j in locationDestinationMap] # define destination array for destination locations on where bricks need to go
# define actual array reading actual locations of bricks for the robot to avoid obstacles

while True: # while loop to place all functions
	tracker = locationDestinationMap # tracker to exit while loop when all bricks are placed
	for k in locationDestinationMap:
		brickStation = station[k:0] # station reading
		brickDestination = destination[k:1] # destination reading
		pickPlace(arm_pubs, grip_pos, grip_pub,brickStation,brickDestination)
		tracker.pop(0) # removing the brick we just worked with
		if not tracker:
			break
	break

# pushing functions

## pick + place function
# def pickPlace(publishers, grip_pos, grip_pub, brickStation, brickDestination):

	# pick brick from there function (IK blabla)
	# place brick to destination
	# pushing
	# error catching

## destination function
# if all bricks are at destination
# print we are done what de fuck you wanna do now bitch
# do final check ups

## station function
# if all bricks have left a station
# move to next station
# print station to be complete

## staircase completed function
# if location of brick is equal to where it needs to go 
# then it has reached its destination and can be marked as complete

## error catching
#1 if brick is lost between station or destination flag it
# inform user of situation: where, which level, etc.
# abort and wait for options
#2 if staircase breaks by brick not in destination location flag it
# inform user of situation: if it can be fixed or starting again faster
# abort and wait for options

# while array actual not empty continue kinda thing with error catching