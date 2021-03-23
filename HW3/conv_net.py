"""
Implementation of a ResNet. 
"""

import tensorflow as tf
import tensorflow.keras as keras

# import tensorflow.contrib.graph_editor as ge #is this even legal

def ConvNet(**kwargs):
    """
    Construct a ResNet using `tf.keras` layers
    """
    
    # TODO: upsampling
    #		add residuals
    #       conv -> norm -> relu (according to prof, video says otherwise)
    # 		tensorboard to view network
    #		figure out **kwargs
    #		add EarlyStopping Callback? (to update LR)
    #		Do I need to individually name each layer? -> figure that out
    # 		ADD LINEAR PROJECTIONS TO ALLOW CONNECTIONS BETWEEN LAYERS OF MISMATCHED SIZE
    # 			- dont change the image size betwen chunks, just change number of filters(??)
    #			- double filters every time image is halved
    #		Resize layers in repeating unit according to table 1 in paper

    #Custom Residual Convolutional Network ----------------------------------------------------------------------------------------------

    inputs = keras.Input(shape=(32,32,3)) #ignores batch size
    X = keras.layers.Conv2D(filters = 16, kernel_size = (7,7), padding = 'same')(inputs) #use 7x7 kernel for first img
    X = keras.layers.ReLU()(X)
    X = keras.layers.MaxPool2D(pool_size=(1,1), padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)
    # X = keras.layers.add([X,inputs]) <- don't need res layer before main chunks
    skip = tf.identity(X)

    chunks = 4
    for i in range(chunks):
	    #repeating unit ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	    # X = keras.layers.add([X,inputs]) #TODO- need to make sure layer3 and inputs are the same shape

	    #run through 2 conv layers
	    X = keras.layers.Conv2D(filters = 16*(2**(i)), kernel_size = (3,3), padding = 'same')(X) # num filters increases with each layer
	    X = keras.layers.ReLU()(X)
	    X = keras.layers.BatchNormalization()(X)

	    X = keras.layers.Conv2D(filters = 16, kernel_size = (3,3), padding = 'same')(X) 
	    X = keras.layers.ReLU()(X)
	    X = keras.layers.BatchNormalization()(X)

   	    #add outputs from previous layer to inputs of network
	    X = keras.layers.add([X,skip])
	    X = keras.layers.ReLU()(X) #test adding ReLU in after add
	    # Save baseline for next shortcut connections
	    skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)

	    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


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


    #cheating with keras resnet function -> easily gets 70+%
    # model = tf.keras.applications.ResNet50(input_shape = (32,32,3),weights = None,classes=10)

    return model 
