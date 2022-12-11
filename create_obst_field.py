import numpy as np
import random

#def create_obst_field(num_obst, xlim, ylim, rlim):
#    x = np.random.uniform(xlim[0], xlim[1], num_obst)
#    y = np.random.uniform(ylim[0], ylim[1], num_obst)
#    r = np.random.uniform(rlim[0], rlim[1], num_obst)

#    obst_field = np.vstack((x, y, r)).T
#    return tuple(map(tuple, obst_field))

def eu_dist(x1, y1, x2, y2):
    return np.hypot((x1 - x2), (y1 - y2))

def gen_obst(xlim, ylim, rlim, seed_tuple): #generate a new obstacle
    x_seed, y_seed, r_seed = seed_tuple
    np.random.seed(x_seed)
    x = np.random.uniform(xlim[0], xlim[1])
    np.random.seed(y_seed)
    y = np.random.uniform(ylim[0], ylim[1])
    np.random.seed(r_seed)
    r = np.random.uniform(rlim[0], rlim[1])
    #new_obst = np.vstack((x, y, r)).T
    return x, y, r

def create_obst_field(num_obst, xlim, ylim, rlim):
    obst_field = []
    seed_list = []   # list of tuples containing (x_seed, y_seed, r_seed) of the obst_field
    while len(obst_field) < num_obst:
      # Get random seeds to then use in the obstacle generation
      x_seed = np.random.randint(1e8)
      y_seed = np.random.randint(1e8)
      r_seed = np.random.randint(1e8)
      seed_tuple = (x_seed, y_seed, r_seed)
    
      x, y, r = gen_obst(xlim, ylim, rlim, seed_tuple)
      
      if not any((x2, y2, r2) for x2, y2, r2 in obst_field if eu_dist(x, y, x2, y2) < r + r2 + 2):
        obst_field.append((x, y, r))
        seed_list.append((x_seed, y_seed, r_seed))

    return tuple(map(tuple, obst_field)), tuple(map(tuple, seed_list))

def recreate_obst_field(num_obst, xlim, ylim, rlim, seed_list):
    obst_field = []
    for i in range(num_obst):
        # Use the seeds to re-create an obst_field
        seed_tuple = seed_list[i]
        x, y, r = gen_obst(xlim, ylim, rlim, seed_tuple)
        obst_field.append((x, y, r))
    return tuple(map(tuple, obst_field))
