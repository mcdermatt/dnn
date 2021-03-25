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
    # 		WHY IS TF RANDOMLTY SWITCHING TO CPU????
    # 
    # 		should be using smaller and smaller layers with more and more filters (think about drawings)
    #			seems like my model is getting down to 1x1 images way too fast?
    # 				resnet paper uses 224x224 images instead of 32x32

    #		look into data augmentation -> increase size of dataset by rotating and shifting images

    #		try  no relu before addingf?

    #		strided conv vs pooloing -> strided conv is more computationally efficient in first few layers
    #  										also uses TRAINABLE PARAMS where pooling is fixed


    #Custom Residual Convolutional Network ----------------------------------------------------------------------------------------------
    #TOP SCORE: 73% accuracy after 25 epochs (~15 mins training)

    inputs = keras.Input(shape=(32,32,3)) #ignores batch size
    X = keras.layers.Conv2D(filters = 64, kernel_size = (7,7), strides = (1,1), padding = 'same')(inputs) #use 7x7 kernel for first img #was strides = (2,2)
    X = keras.layers.ReLU()(X)
    # X = keras.layers.MaxPool2D(pool_size=(2,2), padding = 'same')(X)
    X = keras.layers.BatchNormalization()(X)
    # X = keras.layers.add([X,inputs]) <- don't need res layer before main chunks
    skip = tf.identity(X)

    #LOOPY APPROACH-----------------------------------
    # chunks = 8
    # for i in range(chunks):
	   #  #repeating unit ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	   #  # X = keras.layers.add([X,inputs]) #TODO- need to make sure layer3 and inputs are the same shape

	   #  #run through 2 conv layers
	   #  X = keras.layers.Conv2D(filters = 4*(2**(i)), kernel_size = (3,3), padding = 'same')(X) # num filters increases with each layer
	   #  X = keras.layers.ReLU()(X)
	   #  X = keras.layers.BatchNormalization()(X)

	   #  X = keras.layers.Conv2D(filters = 4, kernel_size = (3,3), padding = 'same')(X) 
	   #  X = keras.layers.ReLU()(X)
	   #  X = keras.layers.BatchNormalization()(X)

   	#     #add outputs from previous layer to inputs of network
	   #  X = keras.layers.add([X,skip])
	   #  X = keras.layers.ReLU()(X) #test adding ReLU in after add
	   #  # Save baseline for next shortcut connections
	   #  skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)

	   #  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    #ALT APPROACH~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    X = keras.layers.Conv2D(filters = 64, kernel_size = (3,3), padding = 'same')(X) # num filters increases with each layer
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)

    X = keras.layers.Conv2D(filters = 64, kernel_size = (3,3), padding = 'same')(X) 
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)

	#add outputs from previous layer to inputs of network
    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X) #test adding ReLU in after add
    # Save baseline for next shortcut connections
    

    # ----------
    X = keras.layers.Conv2D(filters = 64, kernel_size = (3,3), strides = (2,2), padding = 'same')(X) # num filters increases with each layer
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)

    X = keras.layers.Conv2D(filters = 64, kernel_size = (3,3), padding = 'same')(X) 
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)

    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X) 
    #-----------
    X = keras.layers.Conv2D(filters = 128, kernel_size = (3,3), strides = (2,2), padding = 'same')(X) # num filters increases with each layer
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    X = keras.layers.Conv2D(filters = 128, kernel_size = (3,3), padding = 'same')(X) 
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X) 

    X = keras.layers.Conv2D(filters = 128, kernel_size = (3,3), padding = 'same')(X) # num filters increases with each layer
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    X = keras.layers.Conv2D(filters = 128, kernel_size = (3,3), padding = 'same')(X) 
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X) 

    # X = keras.layers.Conv2D(filters = 32, kernel_size = (3,3), padding = 'same')(X) # num filters increases with each layer
    # X = keras.layers.ReLU()(X)
    # X = keras.layers.BatchNormalization()(X)
    # skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    # X = keras.layers.Conv2D(filters = 32, kernel_size = (3,3), padding = 'same')(X) 
    # X = keras.layers.ReLU()(X)
    # X = keras.layers.BatchNormalization()(X)
    # X = keras.layers.add([X,skip])
    # X = keras.layers.ReLU()(X) 
    #-----------
    X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), padding = 'same')(X) # had strides = (2,2)
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), padding = 'same')(X) 
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X)

    X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), padding = 'same')(X) # num filters increases with each layer
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), padding = 'same')(X) 
    X = keras.layers.ReLU()(X)
    X = keras.layers.BatchNormalization()(X)
    X = keras.layers.add([X,skip])
    X = keras.layers.ReLU()(X) 

   	#testing without this last conv layer (was overfitting)
    # X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), padding = 'same')(X) # num filters increases with each layer
    # X = keras.layers.ReLU()(X)
    # X = keras.layers.BatchNormalization()(X)
    # skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    # X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), padding = 'same')(X) 
    # X = keras.layers.ReLU()(X)
    # X = keras.layers.BatchNormalization()(X)
    # X = keras.layers.add([X,skip])
    # X = keras.layers.ReLU()(X)    
    #-----------
    # X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), strides = (2,2), padding = 'same')(X) # num filters increases with each layer
    # X = keras.layers.ReLU()(X)
    # X = keras.layers.BatchNormalization()(X)
    # skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    # X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), padding = 'same')(X) 
    # X = keras.layers.ReLU()(X)
    # X = keras.layers.BatchNormalization()(X)
    # X = keras.layers.add([X,skip])
    # X = keras.layers.ReLU()(X)     

    # X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), padding = 'same')(X) # num filters increases with each layer
    # X = keras.layers.ReLU()(X)
    # X = keras.layers.BatchNormalization()(X)
    # skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    # X = keras.layers.Conv2D(filters = 256, kernel_size = (3,3), padding = 'same')(X) 
    # X = keras.layers.ReLU()(X)
    # X = keras.layers.BatchNormalization()(X)
    # X = keras.layers.add([X,skip])
    # X = keras.layers.ReLU()(X) 

    # X = keras.layers.Conv2D(filters = 512, kernel_size = (3,3), padding = 'same')(X) # num filters increases with each layer
    # X = keras.layers.ReLU()(X)
    # X = keras.layers.BatchNormalization()(X)
    # skip = tf.identity(X) # <-creates a copy of the tensor inputs (skip = X does not work)
    # X = keras.layers.Conv2D(filters = 512, kernel_size = (3,3), padding = 'same')(X) 
    # X = keras.layers.ReLU()(X)
    # X = keras.layers.BatchNormalization()(X)
    # X = keras.layers.add([X,skip])
    # X = keras.layers.ReLU()(X)

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    #TODO -> figure out how many nodes the last convolutional layer should output (currently outputs 4)
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
