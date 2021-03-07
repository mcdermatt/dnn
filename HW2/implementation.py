"""
This problem is modified from a problem in Stanford CS 231n assignment 1. 
In this problem, we implement the neural network with tensorflow instead of numpy
"""

import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

##################################################################################################################
# Function suggestions
# np.random.random_sample(), tf.Variable(), tf.matmul(), tf.nn.relu(), tf.tanh(), tf.sigmoid(), 
# tf.nn.softmax(), tf.reduce_sum(), tf.square(), tf.keras.optimizers.SGD(), tf.keras.optimizers.Adam()
# tf.keras.losses.MSE(), tf.keras.losses.CategoricalCrossentropy(), tf.keras.Model.compile(), tf.keras.Model.fit()
##################################################################################################################

#DEBUG:
#   den.variables = []  -> my layers have 0 trainable parameters, this is why nothing is improving
#       DenseLayer weights and biases are not actually connected to TF rn
#	Add batch normalization

#TODO:
# 	Add batch normalization
#	replace relu with leaky_relu

class DenseLayer(tf.keras.layers.Layer):
    """
    Implement a dense layer 
    """

    def __init__(self, input_dim, output_dim, activation='relu', reg_weight=1e-5, param_init=None):

        """
        Initialize weights of the DenseLayer. In Tensorflow's implementation, the weight 
        matrix and bias term are initialized in the `build` function. Here let's use the simple form. 

        https://www.tensorflow.org/guide/keras/custom_layers_and_models

        args:
            input_dim: integer, the dimension of the input layer
            output_dim: integer, the dimension of the output layer
            activation: string, can be 'linear', 'relu', 'tanh', 'sigmoid', or 'softmax'. 
                        It specifies the activation function of the layer
            reg_weight: the regularization weight/strength, the lambda value in a regularization 
                        term 0.5 * \lambda * ||W||_2^2
                        
            param_init: `dict('W'=W, 'b'=b)`. Here W and b should be `np.array((input_dim, output_dim))` 
                        and `np.array((1, output_dim))`. The weight matrix and the bias vector are 
                        initialized by `W` and `b`. 
                        NOTE: `param_init` is used to check the correctness of your function. For you 
                        own usage, `param_init` can be `None`, and the parameters are initialized 
                        within this function. But when `param_init` is not None, you should 
                        `param_init` to initialize your function. 

        """


        super(DenseLayer, self).__init__()

        # set initial values for weights and bias terms. 
        # Note: bad initializations may lead to bad performance later
        param_init = dict(W=None, b=None)

        if param_init == 'autograder':
            np.random.seed(137)
            param_init['W'] = np.random.random_sample((input_dim, output_dim)) 
            param_init['b'] = np.random.random_sample((output_dim, )) 
        else:
            np.random.seed(69) #nice
            param_init['W'] = np.random.random_sample((input_dim, output_dim)) 
            param_init['b'] = np.random.random_sample((output_dim, ))
           
        
        # Initialize necessary variables
        self.input_dim = input_dim
        self.output_dim = output_dim

        #TODO - get this working with random seed
        self.kernel = self.add_weight("kernel", shape=[self.input_dim,self.output_dim])

        #TODO - add bias try add_weight again??
        self.bias = self.add_weight("bias",shape=[self.output_dim,])

        #was this
        # self.W = param_init['W'] 
        # self.b = param_init['b']

        #TODO add linear and softmax
        if activation == 'sigmoid':
	        self.activation = tf.math.sigmoid

        if activation == 'tanh':
	        self.activation = tf.math.tanh

        if activation == 'relu':
            self.activation = tf.keras.activations.relu

        if activation == 'leaky_relu':
        	self.activation = tf.nn.leaky_relu

        if activation == 'softmax':
        	self.activation = tf.nn.softmax


    def call(self, inputs, training=None, mask=None):
        """
        This function implement the `call` function of the class's parent `tf.keras.layers.Layer`. Please 
        consult the documentation of this function from `tf.keras.layers.Layer`.

        #TODO- do inputs need to be special tf object or can they just be a tensor?
        """
        
        # Implement the linear transformation------------------
        # outputs = inputs*self.W + self.b #NOT THIS(?)- don't want to output a vector

        #TODO- do I have a unique bias for each input to every node in a denselayer? (I think yes)
        # want to take in inputs from each previous node and multiply by a weight and add a bias
        # outputs = tf.tensordot(inputs,self.W, axes = 1) + b 
        # outputs = tf.reduce_sum(inputs*self.W + self.b) #need multiple biases

        #trying this
        # outputs = tf.matmul(inputs, self.kernel) #input: 
        outputs = tf.matmul(inputs, self.kernel) + self.bias #input: 


        # worked mathematically but TF doesn't recognize for backprop
        # outputs = tf.tensordot(inputs,tf.cast(self.W,float),axes=1) + self.b 
        # 			test call with:
        # 			den.call(tf.Variable([1.,2.,3...]))

        # Implement the activation function---------------------
        outputs = self.activation(outputs)

        # check self.add_loss() to add the regularization term to the training objective-----------------
        # self.add_loss(tf.abs(tf.reduce_mean(outputs))) #stores loss for later use in backprop

        return outputs
        

class Feedforward(tf.keras.Model):

    """
    A feedforward neural network. 
    """

    def __init__(self, input_size, depth, hidden_sizes, output_size, reg_weight, task_type):

        """
        Initialize the model. This way of specifying the model architecture is clumsy, but let's use this straightforward
        programming interface so it is easier to see the structure of the program. Later when you program with keras 
        layers, please think about how keras layers are implemented to take care of all components.  

        args:
          input_size: integer, the dimension of the input.
          depth:  integer, the depth of the neural network, or the number of connection layers. 
          hidden_sizes: list of integers. The length of the list should be one less than the depth of the neural network.
                        The first number is the number of output units of first connection layer, and so on so forth.
          output_size: integer, the number of classes. In our regression problem, please use 1. 
          reg_weight: float, The weight/strength for the regularization term.
          task_type: string, 'regression' or 'classification'. The task type. 
        """

        super(Feedforward, self).__init__()

        # Add a contition to make the program robust ----------------------------------
        if not (depth - len(hidden_sizes)) == 1:
            raise(Exception('The depth of the network is ', depth, ', but `hidden_sizes` has ', len(hidden_sizes), ' numbers in it.'))

         
        # install all connection layers except the last one ---------------------------

        #was this
        self.model = tf.keras.Sequential()
       
        
        if task_type == 'regression':
            #set input layers
            self.model.add(DenseLayer(1,hidden_sizes[0]))
       	    #set hidden layers
       	    for layer in range(1,depth-1):
                self.model.add(DenseLayer(hidden_sizes[layer-1],hidden_sizes[layer],activation='tanh'))
            #set output layer
            self.model.add(DenseLayer(hidden_sizes[-1],1,activation='tanh'))


       	if task_type == 'classification':
       	    #set input layers
       	    self.model.add(DenseLayer(input_size,hidden_sizes[0]))
       	    #set hidden layers
            for layer in range(1,depth-1):
                self.model.add(DenseLayer(hidden_sizes[layer-1],hidden_sizes[layer],activation='relu'))
            #set output layer
            self.model.add(DenseLayer(hidden_sizes[-1],output_size,activation='softmax'))


    def call(self, inputs, training=None, mask=None):
        """
        Implement the `call` function of `tf.keras.Model`. Please consult the documentation of tf.keras.Model to understand 
        this function. 
        """
        
        # print a message from the network function. Please don't delete this line and count how many times this message is printed
        # It seems that every time the network is evaluated, the message should be printed. If so, then the message should be printed
        # for #batches times. However, you only see it printed once or twice, why? 

        #TODO- answer this question

        print('I am in the network function!')

        # Now start implement this function and apply the neural network on the input 
        # outputs = None

        outputs = self.model(inputs) # test works with model(tf.Variable([1.,2.,3...]))

        return outputs


def train(x_train, y_train, x_val, y_val, depth, hidden_sizes, reg_weight, num_train_epochs, task_type):

    """
    Train this neural network using stochastic gradient descent.

    args:
      x_train: `np.array((N, D))`, training data of N instances and D features.
      y_train: `np.array((N, C))`, training labels of N instances and C fitting targets 
      x_val: `np.array((N1, D))`, validation data of N1 instances and D features.
      y_val: `np.array((N1, C))`, validation labels of N1 instances and C fitting targets 
      depth: integer, the depth of the neural network 
      hidden_sizes: list of integers. The length of the list should be one less than the depth of the neural network.
                    The first number is the number of output units of first connection layer, and so on so forth.

      reg_weight: float, the regularization strength.
      num_train_epochs: the number of training epochs.
      task_type: string, 'regression' or 'classification', the type of the learning task.
    """

    # prepare the data to make sure the data type is correct. -----------------------------

	#convert numpy data from notebook to tensors
    x_train = tf.convert_to_tensor(x_train) #training data x values    
    y_train = tf.convert_to_tensor(y_train) #training data y values
    x_val = tf.convert_to_tensor(x_val) #x values for testing    
    y_val = tf.convert_to_tensor(y_val) #y values for testing

    #TODO - figure out way to loop through data-> need to deal with many input points

    # initialize a model with the Feedforward class ---------------------------------------

    if task_type == 'regression':
    	input_size = 1
    	output_size = 1
    	batch_size = 32
    	LR = 0.0075

    if task_type == 'classification':
    	# input_size = x_train.shape[0] #TODO: fix this
    	input_size = 784 #28x28 images
    	output_size = 10 #should be 10 for MNIST dataset
    	batch_size = 128
    	LR = 0.001

    	#need to reclassify images as float (uint does not work with TF?)
    	x_train = tf.cast(x_train, tf.float32)
    	x_val = tf.cast(x_val, tf.float32)

    model = Feedforward(input_size,depth,hidden_sizes,output_size,reg_weight,task_type)

    # initialize an opimizer --------------------------------------------------------------
    #TODO - try different optimizers for different tasks
    optim = tf.keras.optimizers.Adam(learning_rate = LR)
    
    # decide the loss for the learning problem --------------------------------------------
    loss = tf.keras.losses.mean_squared_error #was this
    # loss = tf.keras.losses.Huber
    # loss = tf.keras.losses.MeanSquaredError

    # compile and train the model. Consider model.fit() -----------------------------------
    # Note: model.fit() returns the training history. Please keep it and return it later


    model.model.compile(optim,loss,metrics=['accuracy']) # need to specify metrics in tf 2.3

    # return the model and the training history. We will print the training history -------
    history = model.model.fit(x_train,y_train,epochs=num_train_epochs,validation_data = (x_val,y_val),batch_size = batch_size,verbose=1) 
    # history = model.model.fit(x_train,y_train,epochs=num_train_epochs,validation_split=0.5,verbose=1) 

    model.model.summary()

    return model, history

    #TODO visualize loss