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
    hidden_size = np.shape(wtr_h)[0]

    #set h for first step
    h = init_state

    outputs = np.zeros([batch_size, time_steps, hidden_size])
    c = 0
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
        # print("outputs ", outputs[:,t,:])
        # print("h ", h)

        outputs[:,t,:] = h 
    c += 1
    # print(c)

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

    hidden_size = np.shape(wt_h)[0]
    input_size = np.shape(wt_x)[0]

    #RNN Formula---------------------------
    # h = np.tanh(np.dot(x,wt_x) + np.dot(h, wt_h) + bias) <------

    #general GRU formula ------------------
    #h = z*h + (1-z)*h_hat
    # if z = 0:
    # h = h_hat
    # h = np.tanh(np.dot(x,wth_x) + np.dot((r * h), wth_h) + biash)
    # if r = 1
    # h = np.tanh(np.dot(x,wth_x) + np.dot(h, wth_h) + biash) <------

    wth_x = wt_x
    wth_h = wt_h
    biash = bias

    # to make z = 0 add a huge negative bias
    # z = sigmoid(np.dot(x,wtz_x) + np.dot(h,wtz_h) + biasz)
    wtz_x = np.ones([input_size, hidden_size]) #not important
    wtz_h = np.ones([hidden_size, hidden_size]) #not important
    biasz = -1000*np.ones([hidden_size])#huge negative number

    # to make r = 1 add huge positve biasr
    biasr = 1000*np.ones([hidden_size])
    wtr_x = np.ones([input_size, hidden_size]) #not important
    wtr_h = np.ones([hidden_size, hidden_size]) #not important

    return wtz_h, wtz_x, biasz, wtr_h, wtr_x, biasr, wth_h, wth_x, biash

def init_gru_with_rnn_the_dumb_way(wt_h, wt_x, bias):
    """
    This function compute parameters of a GRU such that it performs like a conventional RNN. The input are parameters 
    of an RNN, and the parameters of the GRU are returned.

    this way uses a network and was a huge waste of Matt's Wednesday night 

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
    # print("hidden_size = ", hidden_size)
    input_size = np.shape(wt_x)[0]
    # print("input size = ", input_size)
    
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
    # print("GRU: ",gru_state)
    # print("RNN: ",rnn_state)

    #init model
    model = Net(wt_x, wt_h, bias)

    #TODO -> make custom loss metric so that we can set the output of the network to be GRU weight values
    #           instead of actual network outputs

    model.compile(
        optimizer=tf.keras.optimizers.Adam(lr=0.001),
        loss=tf.keras.losses.MeanSquaredError(),
        metrics=[tf.keras.metrics.MeanSquaredError()],)
    
    summary = model.summary()

    flat_in = input_size + 2*hidden_size #for initial state
    flat_out = 41 #3*hidden_size*hidden_size + 3*input_size*hidden_size + 3*hidden_size + inputs

    #generate test data
    dataSize = 100000
    x_train = np.zeros([dataSize,flat_in])
    y_train = np.zeros([dataSize,41]) # was 14
    count = 0
    for i in range(dataSize):
        #init state
        IS = np.random.randn(1,1,2)
        #input data
        ID = np.random.randn(1,1,input_size)
        # outputs, state = rnn(wt_h, wt_x, bias, IS, ID)
        #create random vector of weights and biases to plug in to GRU func

        wtz_h1 = np.random.randn(hidden_size, hidden_size) 
        wtz_x1 = np.random.randn(input_size, hidden_size) 
        biasz1 = np.random.randn(hidden_size)
        wtr_h1 = np.random.randn(hidden_size, hidden_size) 
        wtr_x1 = np.random.randn(input_size, hidden_size) 
        biasr1 = np.random.randn(hidden_size)
        wth_h1 = np.random.randn(hidden_size, hidden_size) 
        wth_x1 = np.random.randn(input_size, hidden_size) 
        biash1 = np.random.randn(hidden_size)
        

        outputs, state = gru(wtz_h1, wtz_x1, biasz1, wtr_h1, wtr_x1, biasr1, wth_h1, wth_x1, biash1, IS, ID) 
        x_train[i,:] = np.append(np.append(IS,ID),state)
        # temp = np.append(np.ndarray.flatten(wtz_h1), (np.ndarray.flatten(wtz_x1), np.ndarray.flatten(biasz1), np.ndarray.flatten(wtr_h1), np.ndarray.flatten(wtr_x1), np.ndarray.flatten(biasr1), np.ndarray.flatten(wth_h1), np.ndarray.flatten(wth_x1), np.ndarray.flatten(biash1), np.ndarray.flatten(IS),np.ndarray.flatten(ID)))
        temp = np.append(np.append(np.append(np.append(np.append(np.append(np.append(np.append(np.append(np.append(np.ndarray.flatten(wtz_h1), np.ndarray.flatten(wtz_x1)), np.ndarray.flatten(biasz1)), np.ndarray.flatten(wtr_h1)), np.ndarray.flatten(wtr_x1)), np.ndarray.flatten(biasr1)), np.ndarray.flatten(wth_h1)), np.ndarray.flatten(wth_x1)), np.ndarray.flatten(biash1)), np.ndarray.flatten(IS)),np.ndarray.flatten(ID))
        # print("temp = ", temp)
        y_train[i,:] = temp

        count += 1
        # print(count)
    #run model
    # print(x_train)
    # print(y_train)


    runLen = 50
    def scheduler(epoch, lr):
        part1 = runLen//3
        part2 = 2*runLen//3 
        if epoch < part1:
            lr = 0.001
            return lr
        if epoch >= part1 and epoch < part2:
            lr = 0.0001
            return lr
        if epoch >= part2:
            lr = 0.00001
            return lr
    
    callback = tf.keras.callbacks.LearningRateScheduler(scheduler)

    trace = model.fit(x=x_train, y=y_train, batch_size=64, epochs=runLen, verbose=1, 
                  validation_split=0.1, callbacks = [callback], shuffle=True)

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


    hidden_size = np.shape(wt_h)[0]
    input_size = np.shape(wt_x)[0]

    #RNN Formula---------------------------
    # h = np.tanh(np.dot(x,wt_x) + np.dot(h, wt_h) + bias) <------

    #general GRU formula ------------------
    #h = z*h + (1-z)*h_hat
    # if z = 1:
    # h = z*h
    # h = sigmoid(np.dot(x,wtz_x) + np.dot(h,wtz_h) + biasz)*h
    # can make z term 1 by adding large bias

    wth_x = wt_x
    wth_h = wt_h
    biash = bias

    # to make z = 1 add a huge positive bias
    # z = sigmoid(np.dot(x,wtz_x) + np.dot(h,wtz_h) + biasz)
    wtz_x = np.ones([input_size, hidden_size]) #not important
    wtz_h = np.ones([hidden_size, hidden_size]) #not important
    biasz = 1000*np.ones([hidden_size])#huge negative number

    # r term is set arbitrarily here
    biasr = np.ones([hidden_size]) #not important
    wtr_x = np.ones([input_size, hidden_size]) #not important
    wtr_h = np.ones([hidden_size, hidden_size]) #not important

    return wtz_h, wtz_x, biasz, wtr_h, wtr_x, biasr, wth_h, wth_x, biash

#Don't think I need this
# @tf.function()
# def CustomLoss1(yTrue, yPred, inputs, init_state):

#     #convert yPred into weight and bias matrices
#     IS = 3 #temp
#     HS = 2 #temp
#     wtz_h, wtz_x, biasz, wtr_h, wtr_x, biasr, wth_h, wth_x, biash = net2GRU(yPred, HS, IS)
#     #calcualte output from GRU given such matrices  
#     outputs, state = gru(wtz_h, wtz_x, biasz, wtr_h, wtr_x, biasr, wth_h, wth_x, biash, init_state, input_data)
#     yPred = state
#     #subtract output from GRU

#     #need nested func- Keras loss funcs can only take in yTrue, yPred
#     def closs(yTrue, yPred):

#         return loss

#     return closs

def Net(wt_x, wt_h, bias):

    ''' Network for converting GRU parms to RNN

    Given information about starting state, input, and ending state network makes estimates on weights
    '''
    input_size = np.shape(wt_x)[0]
    print("input_size = ", input_size)
    hidden_size = np.shape(wt_x)[1]
    print("hidden_size = ", hidden_size)

    #inputs should be init state, input and ending state
    flat_in = input_size + 2*hidden_size
    print("flat in: ",flat_in)
    # flat_out = 2
    flat_out =  41 #3*hidden_size*hidden_size + 3*input_size*hidden_size + 3*hidden_size + inputs
    print("flat out: ",flat_out)

    inputs = tf.keras.Input(shape=(flat_in))
    X = tf.keras.layers.Dense(units = 512, activation = 'relu')(inputs)
    X = tf.keras.layers.BatchNormalization()(X)
    X = tf.keras.layers.Dense(units = 156, activation = 'relu')(X)
    X = tf.keras.layers.BatchNormalization()(X)
    X = tf.keras.layers.Dense(units = 128, activation = 'relu')(X)
    # X = tf.keras.layers.BatchNormalization()(X)
    # X = tf.keras.layers.Dense(units = 64, activation = 'relu')(X)
    # X = tf.keras.layers.BatchNormalization()(X)
    # X = tf.keras.layers.Dense(units = 64, activation = 'relu')(X)

    output = tf.keras.layers.Dense(units = flat_out, activation = 'tanh')(X)

    model = tf.keras.Model(inputs, output)

    return model

def net2GRU(yPred, HS, IS):

    '''converts output of network to GRU weight and bias matrices

    [ouput from network, hidden_size, input_size]'''

    wtz_h = yPred[:HS*HS].numpy()
    wtr_h = yPred[HS*HS:2*HS*HS].numpy()
    wth_h = yPred[2*HS*HS:3*HS*HS].numpy()

    wtz_x = yPred[3*HS*HS: 3*HS*HS + IS*HS].numpy()
    wtr_x = yPred[3*HS*HS + IS*HS: 3*HS*HS + 2*IS*HS].numpy()
    wtr_x = yPred[3*HS*HS + 2*IS*HS: 3*HS*HS + 3*IS*HS].numpy()

    biasz = yPred[3*HS*HS + 3*IS*HS: 3*HS*HS + 3*IS*HS + HS].numpy()
    biasr = yPred[3*HS*HS + 3*IS*HS + HS: 3*HS*HS + 3*IS*HS + 2*HS].numpy()
    biash = yPred[3*HS*HS + 3*IS*HS + 2*HS: 3*HS*HS + 3*IS*HS + 3*HS].numpy()   

    return wtz_h, wtz_x, biasz, wtr_h, wtr_x, biasr, wth_h, wth_x, biash