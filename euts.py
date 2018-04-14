# Created on Sat Mar 10 17:18:49 2018
# Author: Chaitanya Pb

# EUTS : EVERYTHING UNDER THE SUN

# Important Comments
# vectors = lists or tuples

#%% Package imports
import numpy as np

#%% Numerical Operations

# Degree-Radian conversion
def rad2deg(rad):
    return rad*(180.0/np.pi)
    
def deg2rad(deg):
    return deg*(np.pi/180.0)

#%% Vector Operations

# Combine two vectors
def vec_combine(a, op, b):
    if len(a) != len(b):
        raise ValueError("Dimensions don't match!")
    
    if op == '+':
        acc = [i+j for i, j in zip(a, b)]
    elif op == '-':
        acc = [i-j for i, j in zip(a, b)]
    elif op == '*':
        acc = [i*j for i, j in zip(a, b)]
    elif op == '/':
        acc = [i/float(j) for i, j in zip(a, b)]
    else:
        raise ValueError("Unsupported operation!")
    
    if a.__class__ == tuple:
        return tuple(acc)
    else:
        return acc

# Operate on a vector
def vec_operate(a, op, b=1, param=1):
    if op == '+':
        acc = [i+b for i in a]
    elif op == '-':
        acc = [i-b for i in a]
    elif op == '*':
        acc = [i*b for i in a]
    elif op == '/':
        acc = [i/float(b) for i in a]
    elif op == 'int':
        acc = [int(i) for i in a]
    elif op == 'round':
        acc = [np.around(i, param) for i in a]
    else:
        raise ValueError("Unsupported operation!")
    
    if a.__class__ == tuple:
        return tuple(acc)
    else:
        return acc

# Euclidean distance between vectors
def vec_euclidean(a, b):
    if len(a) != len(b):
        raise ValueError("Dimensions don't match!")
    
    e = 0
    for i in range(len(a)):
        e += pow(a[i] - b[i], 2)
    return np.sqrt(e)

#%% Dictionary Operations

# Combine two dictionaries
def dict_combine(a, op, b):
    if a.keys() != b.keys():
        raise ValueError("Dictionary keys don't match!")
    
    c = {}
    for key in a.keys():
        if op == '+':
            c[key] = a[key] + b[key]
        elif op == '-':
            c[key] = a[key] - b[key]
        elif op == '*':
            c[key] = a[key] * b[key]
        elif op == '/':
            c[key] = a[key] / float(b[key])
        else:
            raise ValueError("Unsupported operation!")
    return c

# Operate on a dictionary
def dict_operate(a, op, b=1, param=1):
    if a.keys() != b.keys():
        raise ValueError("Dictionary keys don't match!")
    
    c = {}
    for key in a.keys():
        if op == '+':
            c[key] = a[key] + b
        elif op == '-':
            c[key] = a[key] - b
        elif op == '*':
            c[key] = a[key] * b
        elif op == '/':
            c[key] = a[key] / float(b)
        elif op == 'int':
            c[key] = int(a[key])
        elif op == 'round':
            c[key] = np.around(a[key], param)
        else:
            raise ValueError("Unsupported operation!")
    return c

# Euclidean distance between dictionaries
def dict_euclidean(a, b):
    if a.keys() != b.keys():
        raise ValueError("Dictionary keys don't match!")
    
    e = 0
    for key in a.keys():
        e += pow(a[key] - b[key], 2)
    return np.sqrt(e)

# Truncate a dictionary
def dict_truncate(a, keys):
    return dict((k, a[k]) for k in keys if k in a)