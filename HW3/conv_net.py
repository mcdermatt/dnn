"""
Implementation of a ResNet. 
"""

import tensorflow as tf
import tensorflow.keras as keras

def ConvNet(**kwargs):
    """
    Construct a ResNet using `tf.keras` layers
    """
    
    # TODO: upsampling
    #		add residuals
    #       how many times to repeat conv and pool? 20-30?

    #getting 40% + training accuracy
    model = tf.keras.Sequential([keras.layers.Conv2D(filters = 3, kernel_size = (3,3)), #outputs 4d tensor
                                 keras.layers.MaxPool2D(), #outputs 4d tensor
                                 keras.layers.BatchNormalization(),

                                 #TEST- repeat these three layers
                                 keras.layers.Conv2D(filters = 3, kernel_size = (3,3)), #outputs 4d tensor
                                 keras.layers.MaxPool2D(), #outputs 4d tensor
                                 keras.layers.BatchNormalization(),

                                 keras.layers.Conv2D(filters = 3, kernel_size = (3,3), activation ='relu'), #outputs 4d tensor
                                 tf.keras.layers.GlobalAvgPool2D(), #outputs 2d tensor

                                 #normal stuff
                                 keras.layers.BatchNormalization(),
                                 tf.keras.layers.Dense(units = 256, activation = 'relu') 
                                 keras.layers.BatchNormalization(),                                 
                                 tf.keras.layers.Dense(units=10, activation = 'softmax')])

    # This is an example model, which sums up all pixels and does classification 
    # model = tf.keras.Sequential([tf.keras.layers.GlobalAvgPool2D(), 
                                 # tf.keras.layers.Dense(units=10)])


    return model 
