import itertools
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from itertools import product, combinations


fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_zlim(-3.7, 3.7)
# ax.set_ylim(-10,100)
# ax.set_zlim(-10, 100)


# draw cube
def draw(dim, com, position, ax):
    signed_corners = np.array([c for c in itertools.product([-1, 1], repeat=3)])
    corners = signed_corners * dim / 2 + position
    for s, e in combinations(corners, 2):
        if ((s - e) == 0).sum() == 2:
            ax.plot3D(*zip(s, e), color="b")

    ax.scatter(*com + position, color='r')

num_blocks = 12
min_block_size = 5
max_gripper_size = 7
max_block_size = 15
ball_radius = 1.25
wall_thickness = 0.5

# generate the block dimensions (in cm)
dims = np.zeros([num_blocks, 3])

for i in range(num_blocks):
    while True:
        d = np.round((min_block_size+(max_block_size - min_block_size)*np.random.rand(3))*2)/2
        grippable = (d <= max_gripper_size).sum() >= 2
        if grippable:
            dims[i] = d
            break
# these dimensions are from the original block set. to generate new plywood blocks I had to
# limit the COM placement
# dims = np.array([[ 7.  , 6. , 14.5]  ,
# [13. ,  6.5 , 5.5]  ,
# [6. , 6.5, 7. ] ,
# [7.5 ,4.5 ,7. ]  ,
# [ 5.5, 13.5 , 6.5]  ,
# [ 4.5 ,14.,   6.5]  ,
# [ 6.5  ,4. , 13. ]  ,
# [ 5. ,  6.5, 12.5]  ,
# [5.5, 6. , 6.5]  ,
# [10.5 , 5. ,  4.5] ,
# [ 5.5 ,14.5 , 6. ] ,
# [10.5 , 5. ,  5. ] ,
# [5. , 9.5, 6. ] ,
# [ 5. ,  4. , 14.5]  ,
# [4. , 8.5, 4.5] ,
# [6.5 ,5. , 4. ] ])

# generate the block coms (with uniform sampling)
# coms = np.random.rand(num_blocks, 3)*(dims-com_border*2) - (dims/2) + com_border

# NOTE(izzy): blocks were uninteresting with uniform random COM. here's a different strategy:
# randomly set the blocks as far as they can be on certain axes. they are always at the edge
# on the longest axis
signed_coms = np.random.randint(-1,2, size=(num_blocks, 3))
# for i, d in enumerate(dims):
#     signed_coms[i, np.argmax(d)] = np.random.choice([-1,1])
coms = signed_coms * (dims/2 - ball_radius - wall_thickness*2)

# plot the blocks
grid_size = np.ceil(np.sqrt(num_blocks))

for i, (d, c) in enumerate(zip(dims,coms)):
    print(f'Block {i}: {d}\t{c}')
    position = np.array([i%grid_size - grid_size/2, i // grid_size - grid_size/2, 0]) * max_block_size
    draw(d, c, position, ax)

plt.show()

print('\nAnd here are commands you can copy/paste to generate the tags.')
for i, d in enumerate(dims/100): 
    print(f'python create_aruco_block.py {i} --dimensions {d[0]} {d[1]} {d[2]}') 
