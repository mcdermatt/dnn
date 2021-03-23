"""
Implementation of a ResNet. 
"""

import tensorflow as tf
import tensorflow.keras as keras

import tensorflow.contrib.graph_editor as ge #is this even legal

def ConvNet(**kwargs):
    """
    Construct a ResNet using `tf.keras` layers
    """
    
    # TODO: upsampling
    #		add residuals
    #       conv -> norm -> relu
    # 		tensorboard to view network
    #		figure out **kwargs
    #		add EarlyStopping Callback? (to update LR)
    #		Do I need to individually name each layer? -> figure that out

    #Custom Residual Convolutional Network ----------------------------------------------------------------------------------------------
    #TODO - fix padding

    inputs = keras.Input(shape=(32,32,3)) #ignores batch size
    X = keras.layers.Conv2D(filters = 3, kernel_size = (3,3), padding = 'same', activation = 'relu')(inputs) #should I move activation to seperate func?
    X = keras.layers.MaxPool2D(pool_size=(1,1), padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)

    chunks = 5
    for _ in range(chunks):
	    #repeating unit ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	    #add outputs from previous layer to inputs of network
	    X = keras.layers.add([X,inputs]) #TODO- need to make sure layer3 and inputs are the same shape

	    # Save baseline for shortcut connections
	    inputs = tf.identity(X) # <-creates a copy of the tensor inputs (inputs = X does not work)

	    #run through a couple conv layers
	    X = keras.layers.Conv2D(filters = 3, kernel_size = (3,3), padding = 'same', activation = 'relu')(X)
	    X = keras.layers.BatchNormalization()(X)

	    X = keras.layers.Conv2D(filters = 3, kernel_size = (3,3), padding = 'same', activation = 'relu')(X) 
	    X = keras.layers.BatchNormalization()(X)

	    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    X = keras.layers.GlobalAvgPool2D()(X) #outputs 2d tensor
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.Dense(units = 256, activation = 'relu')(X) 
    X = keras.layers.BatchNormalization()(X)                                
    output = keras.layers.Dense(units=10, activation = 'softmax')(X)

    model = tf.keras.Model(inputs,output)

    #-------------------------------------------------------------------------------------------------------------------------------------
    # Large linear network with no residuals getting ~40% training accuracy
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


    #cheating with keras resnet function -> easily gets 70+%
    # model = tf.keras.applications.ResNet50(input_shape = (32,32,3),weights = None,classes=10)

    return model 
