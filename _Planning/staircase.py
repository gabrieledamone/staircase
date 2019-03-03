'''
Name: staircase.py
Last edited by/on: FAM 03/03/19 19:10
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
height = raw_input("Height of Staircase: ") # ask for height of staircase
print("Building staircase of height" + str(height)) # output for transparency
heightAmount = { # define amount of bricks necessary dependent on height
	1: 1,
	2: 3,
	3: 6,
	4: 10,
	5: 15,
}
brickNums = heightAmount(height) # count amount of bricks necessary = length of location array]


locationDestinationOptions = [map.one, map.two, map.three, map.four, map.five] # load options array of height station maps, order of bricks is order of pick up: right to left view from top

heightMap = [
[],
[],
[],
[],
]


locationDestinationMap = locationDestinationOptions[height-1] # read location/destination array from options array from height with locations in order of all bricks (according to logic) and where they need to go
actual = # define actual array reading actual locations of bricks for the robot to avoid obstacles
station = # define station array listing starting locations for bricks
destination = # define destination array for destination locations on where bricks need to go

## pick + place function
# read location of brick # in location/destination array
# pick brick from there function (IK blabla)
# place brick to destination
# pushing
# error catching
# if placed correctly fix destination array
# next

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