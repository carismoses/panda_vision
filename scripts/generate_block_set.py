import itertools
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from itertools import product, combinations


def compute_real_com(dim, com):
    density_of_wood = 0.68 # g/cm^3
    density_of_lead = 11.35 # g/cm^3
    wall_thickness = 0.49 # cm
    ball_radius = 1.25 # cm

    ball_mass = 4/3*np.pi*ball_radius**3 * density_of_lead
    box_mass = (np.prod(dim) - np.prod(dim - wall_thickness*2)) * density_of_wood
    total_mass = box_mass + ball_mass

    return ball_mass/total_mass * com


def compute_real_com_from_blockset(blockset_filename):
    with open(blockset_filename, 'r') as f:
        for line in f.readlines():
            dim_str, com_str = line.replace('[', '').replace(']', '').split(':')[1].split('\t')
            dim = np.array([float(d) for d in dim_str.split()])
            com = np.array([float(c) for c in com_str.split()])
            print(compute_real_com(dim, com))


# draw cube
def draw(dim, com, position, ax):
    signed_corners = np.array([c for c in itertools.product([-1, 1], repeat=3)])
    corners = signed_corners * dim / 2 + position
    for s, e in combinations(corners, 2):
        if ((s - e) == 0).sum() == 2:
            ax.plot3D(*zip(s, e), color="b")

    ax.scatter(*com + position, color='r')



if __name__ == '__main__':

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(-3.7, 3.7)
    # ax.set_ylim(-10,100)
    # ax.set_zlim(-10, 100)


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


    # generate the block coms (with uniform sampling)
    # coms = np.random.rand(num_blocks, 3)*(dims-com_border*2) - (dims/2) + com_border

    # NOTE(izzy): blocks were uninteresting with uniform random COM. here's a different strategy:
    # randomly set the blocks as far as they can be on certain axes.
    signed_coms = np.random.randint(-1,2, size=(num_blocks, 3))
    # the are always at the edge on the longest axis:
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


