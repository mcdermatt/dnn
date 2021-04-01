"""
Custom CNN for solving inertia based human pose estimation problem
"""

import tensorflow as tf
import tensorflow.keras as keras

def Net(**kwargs):

    #TODO -> take trajPts as input
    inputs = keras.Input(shape=(10,3))

    #TODO Try 2D conv with 1 input filter?
    X = keras.layers.BatchNormalization()(inputs)
    X = tf.keras.layers.Conv1D(filters = 3, kernel_size = 3, activation='relu')(X)
    X = keras.layers.BatchNormalization()(X)
    X = tf.keras.layers.Conv1D(filters = 3, kernel_size = 3, activation='relu')(X)


    X = keras.layers.Flatten()(X)
    X = keras.layers.Dense(units = 64, activation = 'relu')(X) 
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.Dense(units = 64, activation = 'relu')(X) 
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.Dense(units = 64, activation = 'relu')(X) 
    X = keras.layers.BatchNormalization()(X)
    output = keras.layers.Dense(units=7, activation = 'tanh')(X)

    #rescale output to the range of motion of each joint
    output = output*tf.constant([10., 10., 5., 110.,120.,120.,130.]) + tf.constant([0., 0., 0., -90., -30., -30., 0.])

    model = tf.keras.Model(inputs,output)

    return model