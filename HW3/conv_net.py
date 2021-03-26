"""
Implementation of a ResNet. 
"""

import tensorflow as tf
import tensorflow.keras as keras

def ConvNet(**kwargs):
    """
    Construct a ResNet using `tf.keras` layers
    """



    #Custom Residual Convolutional Network  ~81% accuracy ----------------------------------------------------------------------------------------------
    inputs = keras.Input(shape=(32,32,3)) #ignores batch size
    X = keras.layers.Conv2D(filters = 64, kernel_size = (7,7), kernel_regularizer = tf.keras.regularizers.l1(0.00001), strides = (1,1), padding = 'same')(inputs) #use 7x7 kernel for first img #was strides = (2,2)
    X = keras.layers.ReLU()(X)
    # X = keras.layers.MaxPool2D(pool_size=(2,2), padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)
    # X = keras.layers.add([X,inputs]) <- don't need res layer before main chunks
    skip = tf.identity(X)

    X = keras.layers.Conv2D(filters = 64, kernel_size = (3,3), kernel_regularizer = tf.keras.regularizers.l1(0.00001), padding = 'same')(X) # num filters increases with each layer
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)

    X = keras.layers.Conv2D(filters = 64, kernel_size = (3,3), kernel_regularizer = tf.keras.regularizers.l1(0.00001), padding = 'same')(X) 
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)

	#add outputs from previous layer to inputs of network
    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X) #test adding ReLU in after add
    # Save baseline for next shortcut connections 

    # ----------
    X = keras.layers.Conv2D(filters = 64, kernel_size = (3,3), strides = (2,2), kernel_regularizer = tf.keras.regularizers.l1(0.00001), padding = 'same')(X) # num filters increases with each layer
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)

    X = keras.layers.Conv2D(filters = 64, kernel_size = (3,3), kernel_regularizer = tf.keras.regularizers.l1(0.00001), padding = 'same')(X) 
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)

    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X) 
    #-----------
    X = keras.layers.Conv2D(filters = 128, kernel_size = (3,3), strides = (2,2), kernel_regularizer = tf.keras.regularizers.l1(0.00001), padding = 'same')(X) # num filters increases with each layer
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    X = keras.layers.Conv2D(filters = 128, kernel_size = (3,3), kernel_regularizer = tf.keras.regularizers.l1(0.00001), padding = 'same')(X) 
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X) 

    X = keras.layers.Conv2D(filters = 128, kernel_size = (3,3), kernel_regularizer = tf.keras.regularizers.l1(0.00001), padding = 'same')(X) # num filters increases with each layer
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    X = keras.layers.Conv2D(filters = 128, kernel_size = (3,3), kernel_regularizer = tf.keras.regularizers.l1(0.00001), padding = 'same')(X) 
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X) 

    #-----------
    X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), kernel_regularizer = tf.keras.regularizers.l1(0.00001), padding = 'same')(X) # had strides = (2,2)
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), kernel_regularizer = tf.keras.regularizers.l1(0.00001), padding = 'same')(X) 
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X)

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    X = keras.layers.GlobalAvgPool2D()(X) #outputs 2d tensor
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.Dense(units = 256, activation = 'relu')(X) 
    X = keras.layers.BatchNormalization()(X)                                
    output = keras.layers.Dense(units=10, activation = 'softmax')(X)

    model = tf.keras.Model(inputs,output)

    #-------------------------------------------------------------------------------------------------------------------------------------
    # "Large" linear network with no residuals getting ~40% training accuracy
    # model = tf.keras.Sequential([keras.layers.Conv2D(filters = 3, kernel_size = (3,3)), #outputs 4d tensor
    #                              keras.layers.MaxPool2D(pool_size=(2,2)), #outputs 4d tensor
    #                              keras.layers.BatchNormalization(),

    #                              keras.layers.Conv2D(filters = 3, kernel_size = (3,3), activation = 'relu'), #outputs 4d tensor
    #                              # keras.layers.MaxPool2D(), #outputs 4d tensor
    #                              keras.layers.BatchNormalization(),

    #                              #repeat this
    #                              keras.layers.Conv2D(filters = 3, kernel_size = (3,3), padding = 'same', activation = 'relu'),
    #                              # tf.keras.layers.Dense(units = 64, activation = 'relu'),
    #                              keras.layers.BatchNormalization(), 
    #                              keras.layers.Conv2D(filters = 3, kernel_size = (3,3), padding = 'same', activation = 'relu'),
    #                              # tf.keras.layers.Dense(units = 64, activation = 'relu'),
    #                              keras.layers.BatchNormalization(), 
    #                              keras.layers.Conv2D(filters = 3, kernel_size = (3,3), padding = 'same' , activation = 'relu'),
    #                              # tf.keras.layers.Dense(units = 64, activation = 'relu'),
    #                              keras.layers.BatchNormalization(), 

    #                              keras.layers.Conv2D(filters = 3, kernel_size = (3,3), padding='same', activation ='relu'), #outputs 4d tensor
    #                              tf.keras.layers.GlobalAvgPool2D(), #outputs 2d tensor
    #                              #normal stuff
    #                              keras.layers.BatchNormalization(),
    #                              tf.keras.layers.Dense(units = 256, activation = 'relu'), 
    #                              keras.layers.BatchNormalization(),                                 
    #                              tf.keras.layers.Dense(units=10, activation = 'softmax')])


    #cheating with keras resnet function -> ~70% accuracy
    # model = tf.keras.applications.ResNet50(input_shape = (32,32,3),weights = None,classes=10)

    return model 
