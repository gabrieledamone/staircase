'''
Name: map.py
Last edited by/on: FAM 03/03/19 19:10
Description: maps for different staircases
'''

# in format location, destination
# need to discuss real or simulation
# we have 4 piles of bricks of height 4 to build to the maximum a 5 by 5 staircase

# 4 piles of bricks from top to bottom
brickPiles = [

# etage 3
[0.75,-0.2,0.195], # 0
[0.5,-0.2,0.195], # 1
[0.75,-0.4,0.195], # 2
[0.5,-0.4,0.195], # 3

# etage 2
[0.75,-0.2,0.13], # 4
[0.5,-0.2,0.13], # 5
[0.75,-0.4,0.13], # 6
[0.5,-0.4,0.13], # 7

# etage 1
[0.75,-0.2,0.065], # 8
[0.5,-0.2,0.065], # 9
[0.75,-0.4,0.065], # 10
[0.5,-0.4,0.065], # 11

# etage 0
[0.75,-0.2,0.0], # 12
[0.5,-0.2,0.0], # 13
[0.75,-0.4,0.0], # 14
[0.5,-0.4,0.0] # 15
]

# stiarcase from bottom to top from far away to close for the biggest staircase we can build
staircaseBricks = [

# etage 0
[0.5,0.6,0.0], # 0
[0.5,0.45,0.0], # 1
[0.5,0.3,0.0], # 2
[0.5,0.15,0.0], # 3
[0.5,0.0,0.0], # 4

# etage 1
[0.5,0.6,0.065], # 5
[0.5,0.45,0.065], # 6
[0.5,0.3,0.065], # 7
[0.5,0.15,0.065], # 8

# etage 2
[0.5,0.6,0.13], # 9
[0.5,0.45,0.13], # 10
[0.5,0.3,0.13], # 11

# etage 3
[0.5,0.6,0.195], # 12
[0.5,0.45,0.195], # 13

# etage 4
[0.5,0.6,0.26] # 14
]

one = [
[brickPiles[0],staircaseBricks[0]]
]

two = [
[brickPiles[0],staircaseBricks[0]],
[brickPiles[1],staircaseBricks[1]],
[brickPiles[2],staircaseBricks[2]]
]

three = [
[brickPiles[0],staircaseBricks[0]],
[brickPiles[1],staircaseBricks[1]],
[brickPiles[2],staircaseBricks[2]],
[brickPiles[3],staircaseBricks[3]],
[brickPiles[4],staircaseBricks[4]],
[brickPiles[5],staircaseBricks[5]]
]

four = [
[brickPiles[0],staircaseBricks[0]],
[brickPiles[1],staircaseBricks[1]],
[brickPiles[2],staircaseBricks[2]],
[brickPiles[3],staircaseBricks[3]],
[brickPiles[4],staircaseBricks[4]],
[brickPiles[5],staircaseBricks[5]],
[brickPiles[6],staircaseBricks[6]],
[brickPiles[7],staircaseBricks[7]],
[brickPiles[8],staircaseBricks[8]],
[brickPiles[9],staircaseBricks[9]],
[brickPiles[10],staircaseBricks[10]]
]

five = [
[brickPiles[0],staircaseBricks[0]],
[brickPiles[1],staircaseBricks[1]],
[brickPiles[2],staircaseBricks[2]],
[brickPiles[3],staircaseBricks[3]],
[brickPiles[4],staircaseBricks[4]],
[brickPiles[5],staircaseBricks[5]],
[brickPiles[6],staircaseBricks[6]],
[brickPiles[7],staircaseBricks[7]],
[brickPiles[8],staircaseBricks[8]],
[brickPiles[9],staircaseBricks[9]],
[brickPiles[10],staircaseBricks[10]],
[brickPiles[11],staircaseBricks[11]],
[brickPiles[12],staircaseBricks[12]],
[brickPiles[13],staircaseBricks[13]],
[brickPiles[14],staircaseBricks[14]]
]