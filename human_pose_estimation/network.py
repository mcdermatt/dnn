"""
Custom CNN for solving inertia based human pose estimation problem
"""

import tensorflow as tf
import tensorflow.keras as keras

def Net(**kwargs):

    #TODO -> take trajPts as input
    #TODO -> remove padding on some layers?
    #TODO -> add weight regularization/ weight decay
    #TODO -> add DROPOUT
    #TODO -> check if multiplying output is favoring joints with smaller range of motion

    inputs = keras.Input(shape=(10,3))
    #TODO Try 2D conv with 1 input filter?
    X = keras.layers.BatchNormalization()(inputs)
    X = tf.keras.layers.Conv1D(filters = 16, kernel_size = 3, activation='relu')(X) #added same padding
    skip = tf.identity(X)

    X = tf.keras.layers.Conv1D(filters = 16, kernel_size = 3, activation='relu', padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)

    X = tf.keras.layers.Conv1D(filters = 16, kernel_size = 3, activation='relu', padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)

    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X)
    
    #Start repeating-------------

    X = tf.keras.layers.Conv1D(filters = 32, kernel_size = 3, activation='relu', padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X)

    X = tf.keras.layers.Conv1D(filters = 32, kernel_size = 3, activation='relu', padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)

    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X)

    #---

    X = tf.keras.layers.Conv1D(filters = 64, kernel_size = 3, activation='relu', padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X)

    X = tf.keras.layers.Conv1D(filters = 64, kernel_size = 3, activation='relu', padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)

    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X)

    #---

    X = tf.keras.layers.Conv1D(filters = 128, kernel_size = 3, activation='relu', padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X)

    X = tf.keras.layers.Conv1D(filters = 128, kernel_size = 3, activation='relu', padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)

    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X)

    #---

    X = tf.keras.layers.Conv1D(filters = 256, kernel_size = 3, activation='relu', padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X)

    X = tf.keras.layers.Conv1D(filters = 256, kernel_size = 3, activation='relu', padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)

    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X)

    #End layers--------------------------------
    # X = keras.layers.MaxPooling1D(pool_size=2)(X) #NEW -> not sure if helpful...
    X = keras.layers.Flatten()(X)
    X = keras.layers.Dense(units = 64, activation = 'relu')(X) #trying without these
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.Dense(units = 64, activation = 'relu')(X) 
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.Dense(units = 64, activation = 'relu')(X) 
    X = keras.layers.BatchNormalization()(X)
    output = keras.layers.Dense(units=7, activation = 'tanh')(X)

    #rescale output to the range of motion of each joint
    output = output*tf.constant([25., 30., 33.75, 55. , 60., 180., 65.]) + tf.constant([0., 0., 26.25, -35., 30., 0., -65.])

    model = tf.keras.Model(inputs,output)

    return model



def Net2(**kwargs):

    """
    Test Network
    """

    inputs = keras.Input(shape=(10,3))
    
    X = keras.layers.Flatten()(X)
    X = keras.layers.Dense(units = 64, activation = 'relu')(X) 
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.Dense(units = 64, activation = 'relu')(X) 
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.Dense(units = 64, activation = 'relu')(X) 
    X = keras.layers.BatchNormalization()(X)
    output = keras.layers.Dense(units=7, activation = 'tanh')(X)

    #rescale output to the range of motion of each joint
    output = output*tf.constant([25., 30., 33.75, 55. , 60., 180., 65.]) + tf.constant([0., 0., 26.25, -35., 30., 0., -65.])
    #forgot a single negative sign...

    model = tf.keras.Model(inputs,output)

    return model