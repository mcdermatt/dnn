"""
In this file, you should implement the forward calculation of the conventional RNN and the RNN with GRU cells. 
Please use the provided interface. The arguments are explained in the documentation of the two functions.

You also need to implement two functions that configurate GRUs in special ways.
"""
import numpy as np
from scipy.special import expit as sigmoid
import tensorflow as tf #not sure if this is legal

def rnn(wt_h, wt_x, bias, init_state, input_data):
    """
    RNN forward calculation.

    args:
        wt_h: (2,2) shape [hidden_size, hidden_size], weight matrix for hidden state transformation. Rows corresponds 
              to dimensions of previous hidden states
        wt_x: (3,2) shape [input_size, hidden_size], weight matrix for input transformation
        bias: (2) shape [hidden_size], bias term
        init_state: (2) shape [hidden_size], the initial state of the RNN
        input_data: (4,5,3) shape [batch_size, time_steps, input_size], input data of `batch_size` sequences, each of
                    which has length `time_steps` and `input_size` features at each time step. 
    returns:
        outputs: shape [batch_size, time_steps, hidden_size], outputs along the sequence. The output at each 
                 time step is exactly the hidden state
        final_state: the final hidden state
    
    """

    batch_size = np.shape(input_data)[0]
    time_steps = np.shape(input_data)[1]
    input_size = np.shape(input_data)[2]
    hidden_size = np.shape(wt_h)[0]

    #set h for first step
    h = init_state

    outputs = np.zeros([batch_size, time_steps, hidden_size])

    for t in range(time_steps):

        x = input_data[:,t,:] #shape: (4,3)

        h = np.tanh(np.dot(x,wt_x) + np.dot(h, wt_h) + bias)

        outputs[:,t,:] = h      

    state = outputs[:,-1,:]

    return outputs, state


def gru(wtz_h, wtz_x, biasz, wtr_h, wtr_x, biasr, wth_h, wth_x, biash, init_state, input_data):
    """
    RNN forward calculation.

    args:
        wtz_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation for z gate
        wtz_x: shape [input_size, hidden_size], weight matrix for input transformation for z gate
        biasz: shape [hidden_size], bias term for z gate
        wtr_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation for r gate
        wtr_x: shape [input_size, hidden_size], weight matrix for input transformation for r gate
        biasr: shape [hidden_size], bias term for r gate
        wth_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation for candicate
               hidden state calculation
        wth_x: shape [input_size, hidden_size], weight matrix for input transformation for candicate
               hidden state calculation
        biash: shape [hidden_size], bias term for candicate hidden state calculation
        init_state: shape [hidden_size], the initial state of the RNN
        input_data: shape [batch_size, time_steps, input_size], input data of `batch_size` sequences, each of
                    which has length `time_steps` and `input_size` features at each time step. 
    returns:
        outputs: shape [batch_size, time_steps, hidden_size], outputs along the sequence. The output at each 
                 time step is exactly the hidden state
        final_state: the final hidden state
    """

    batch_size = np.shape(input_data)[0]
    time_steps = np.shape(input_data)[1]
    input_size = np.shape(input_data)[2]
    hidden_size = np.shape(init_state)[1]

    #set h for first step
    h = init_state

    outputs = np.zeros([batch_size, time_steps, hidden_size])

    for t in range(time_steps):

        x = input_data[:,t,:]

        # reset gate -> short term memory (move to new subject)
        r = sigmoid(np.dot(x,wtr_x) + np.dot(h,wtr_h) + biasr)

        # update gate -> long term memory (Retains memory of subject, adds information)
        z = sigmoid(np.dot(x,wtz_x) + np.dot(h,wtz_h) + biasz)

        #update canadate h
        h_hat = np.tanh(np.dot(x,wth_x) + np.dot((r * h), wth_h) + biash)

        #update h
        h = z*h + (1-z)*h_hat

        #save outputs
        outputs[:,t,:] = h 

    state = outputs[:,-1,:]

       
    return outputs, state



def init_gru_with_rnn(wt_h, wt_x, bias):
    """
    This function compute parameters of a GRU such that it performs like a conventional RNN. The input are parameters 
    of an RNN, and the parameters of the GRU are returned. 

    args:
        wt_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation. Rows corresponds 
              to dimensions of previous hidden states
        wt_x: shape [input_size, hidden_size], weight matrix for input transformation
        bias: shape [hidden_size], bias term

    returns:


        wtz_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation for z gate
        wtz_x: shape [input_size, hidden_size], weight matrix for input transformation for z gate
        biasz: shape [hidden_size], bias term for z gate
        wtr_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation for r gate
        wtr_x: shape [input_size, hidden_size], weight matrix for input transformation for r gate
        biasr: shape [hidden_size], bias term for r gate
        wth_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation for candicate
               hidden state calculation
        wth_x: shape [input_size, hidden_size], weight matrix for input transformation for candicate
               hidden state calculation
        biash: shape [hidden_size], bias term for candicate hidden state calculation

    """
    
    #get sizes from 
    hidden_size = np.shape(wt_h)[0]
    print("hidden_size = ", hidden_size)
    input_size = np.shape(wt_x)[0]
    print("input size = ", input_size)
    
    #Iterative Method-------------------------------------------------
    #create test init states and input data
    batch_size = 1
    time_steps = 4
    init_state = np.zeros([batch_size, hidden_size])
    input_data = np.random.rand(batch_size, time_steps, input_size).astype(np.float32)

    #get outputs from RNN function
    rnn_out, rnn_state = rnn(wt_h, wt_x, bias, init_state, input_data)

    #init GRU weights and biases
    wtz_h = np.ones([hidden_size, hidden_size]) # weight matrix for hidden state transformation for z gate
    wtz_x = np.ones([input_size, hidden_size]) # weight matrix for input transformation for z gate
    biasz = np.ones([hidden_size]) # bias term for z gate
    wtr_h = np.ones([hidden_size, hidden_size]) # weight matrix for hidden state transformation for r gate
    wtr_x = np.ones([input_size, hidden_size]) # weight matrix for input transformation for r gate
    biasr = np.ones([hidden_size]) # bias term for r gate
    wth_h = np.ones([hidden_size, hidden_size]) # weight matrix for hidden state transformation for candicate
              # hidden state calculation
    wth_x = np.zeros([input_size, hidden_size]) # weight matrix for input transformation for candicate
               # hidden state calculation
    biash = np.zeros([hidden_size]) #

    #compare outputs
    gru_out, gru_state = gru(wtz_h, wtz_x, biasz, wtr_h, wtr_x, biasr, wth_h, wth_x, biash, init_state, input_data)
    print("GRU: ",gru_state)
    print("RNN: ",rnn_state)

    #init model
    model = Net(wt_h, wt_x, bias)

    #TODO -> make custom loss metric so that we can set the output of the network to be GRU weight values
    #           instead of actual network outputs

    model.compile(
        optimizer=tf.keras.optimizers.Adam(lr=0.0001),
        loss=tf.keras.losses.MeanSquaredError(),
        metrics=[tf.keras.metrics.MeanSquaredError()],)
    
    summary = model.summary()

    flat_in = input_size + 2 #for initial state
    flat_out = 2 #just the state #3*hidden_size*hidden_size + 3*input_size*hidden_size + 3*hidden_size

    #generate test data
    dataSize = 10000
    x_train = np.zeros([dataSize,flat_in])
    y_train = np.zeros([dataSize,flat_out])
    for i in range(dataSize):
        #init state
        IS = np.random.randn(1,1,2)
        #input data
        ID = np.random.randn(1,1,input_size)
        x_train[i,:] = np.append(IS,ID)
        outputs, state = rnn(wt_h, wt_x, bias, IS, ID)
        y_train[i,:] = state

    #run model
    # print(x_train)
    # print(y_train)
    runLen = 50
    trace = model.fit(x=x_train, y=y_train, batch_size=32, epochs=runLen, verbose=1, 
                  validation_split=0.1, shuffle=True)

    #Brute Force Method (hard)---------------------------------------
    # Given: 
    # y_RNN                                            = y_GRU
    # np.tanh(np.dot(x,wt_x) + np.dot(h, wt_h) + bias) = z*h + (1-z)*h_hat
    # np.tanh(np.dot(x,wt_x) + np.dot(h, wt_h) + bias) = sigmoid(np.dot(x,wtz_x) + np.dot(h,wtz_h) + biasz)*h 
    #                                                    + (1-sigmoid(np.dot(x,wtz_x)+np.dot(h,wtz_h)+biasz))*np.tanh(np.dot(x,wth_x)+np.dot((sigmoid(np.dot(x,wtr_x) + np.dot(h,wtr_h) + biasr)*h),wth_h) + biash)

    #for x:
    # 2*sigmoid(dot(x,wt_x) + dot(h,wt_h) + bias) - 1 = sigmoid(dot(x,wtz_x) +

    # wtz_h = wt_h #np.identity(hidden_size)
    # wtz_x = np.zeros([input_size,hidden_size])
    # biasz = np.zeros(hidden_size)
    # wtr_h = np.zeros([hidden_size,hidden_size])
    # wtr_x = wt_x #was np.zeros([input_size, hidden_size])
    # # wtr_x = np.identity(max(hidden_size,input_size))
    # biasr = np.zeros(hidden_size)
    # wth_h = np.ones([hidden_size,hidden_size])
    # wth_x = np.ones([input_size,hidden_size])
    # biash = bias


    return wtz_h, wtz_x, biasz, wtr_h, wtr_x, biasr, wth_h, wth_x, biash


def init_gru_with_long_term_memory(wt_h, wt_x, bias):
    """
    This function compute parameters of a GRU such that it maintains the initial state in the memory. The input are parameters 
    of an RNN, and the parameters of the GRU are returned. These parameters can provide shapes of weight matrices but they 
    should affect the behavior of the GRU RNN. 

    args:
        wt_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation. Rows corresponds 
              to dimensions of previous hidden states
        wt_x: shape [input_size, hidden_size], weight matrix for input transformation
        bias: shape [hidden_size], bias term

    returns:


        wtz_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation for z gate
        wtz_x: shape [input_size, hidden_size], weight matrix for input transformation for z gate
        biasz: shape [hidden_size], bias term for z gate
        wtr_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation for r gate
        wtr_x: shape [input_size, hidden_size], weight matrix for input transformation for r gate
        biasr: shape [hidden_size], bias term for r gate
        wth_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation for candicate
               hidden state calculation
        wth_x: shape [input_size, hidden_size], weight matrix for input transformation for candicate
               hidden state calculation
        biash: shape [hidden_size], bias term for candicate hidden state calculation

    """


    ####################################################################################################
    # Please set a set of parameters for a GRU such that it keeps the initial state in its memory      #
    ####################################################################################################
     
    return wtz_h, wtz_x, biasz, wtr_h, wtr_x, biasr, wth_h, wth_x, biash

def Net(wt_x, wt_h, bias):

    ''' Network for converting GRU parms to RNN
    '''
    input_size = np.shape(wt_x)[0]
    hidden_size = np.shape(wt_x)[1]


    #inputs should be random input (x) values from enviornment AND weights of RNN
    flat_in = input_size + 2
    print("flat in: ",flat_in)
    flat_out = 2 # 3*hidden_size*hidden_size + 3*input_size*hidden_size + 3*hidden_size
    print("flat out: ",flat_out)

    inputs = tf.keras.Input(shape=(5))
    X = tf.keras.layers.Dense(units = 64, activation = 'relu')(inputs)
    X = tf.keras.layers.BatchNormalization()(X)
    X = tf.keras.layers.Dense(units = 64, activation = 'relu')(X)
    # X = tf.keras.layers.BatchNormalization()(X)
    # X = tf.keras.layers.Dense(units = 64, activation = 'relu')(X)

    output = tf.keras.layers.Dense(units = flat_out, activation = 'tanh')(X)

    model = tf.keras.Model(inputs, output)

    return model