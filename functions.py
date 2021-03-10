import numpy as np

def angle_vectors(vector_1, vector_2):
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    return angle

def cart2pol(cart = np.array([0,0])):   # np.array([x,y])
    rho = np.sqrt(cart[0]**2 + cart[1]**2)
    phi = np.arctan2(cart[1], cart[0])
    return np.array([rho, phi])

def pol2cart(pol=np.array([0,0])): # [rho,phi]
    x = pol[0] * np.cos(pol[1])
    y = pol[0] * np.sin(pol[1])
    return np.array([x,y])


def normalize(item):
    return item / np.linalg.norm(item)
