"""
In this file, you should implement the forward calculation of the conventional RNN and the RNN with GRU cells. 
Please use the provided interface. The arguments are explained in the documentation of the two functions.

You also need to implement two functions that configurate GRUs in special ways.
"""

import numpy as np
from scipy.special import expit as sigmoid

def rnn(wt_h, wt_x, bias, init_state, input_data):
    """
    RNN forward calculation.

    args:
        wt_h: shape [hidden_size, hidden_size], weight matrix for hidden state transformation. Rows corresponds 
              to dimensions of previous hidden states
        wt_x: shape [input_size, hidden_size], weight matrix for input transformation
        bias: shape [hidden_size], bias term
        init_state: shape [hidden_size], the initial state of the RNN
        input_data: shape [batch_size, time_steps, input_size], input data of `batch_size` sequences, each of
                    which has length `time_steps` and `input_size` features at each time step. 
    returns:
        outputs: shape [batch_size, time_steps, hidden_size], outputs along the sequence. The output at each 
                 time step is exactly the hidden state
        final_state: the final hidden state
    
    """

    #TODO account for batch size

    batch_size = np.shape(input_data)[0]
    time_steps = np.shape(input_data)[1]
    input_size = np.shape(input_data)[2]
    hidden_size = np.shape(wt_h)[0]
    print("hidden size = ", hidden_size)

    #init x to given initial state
    h = init_state
    print("inital state (h) shape: ", np.shape(h))
    outputs = np.zeros([batch_size, time_steps, hidden_size])

    #TODO -> do I actually want to loop  through batch like this (I think no?)
    #       if I keep this I need to rework how I init h (as of rn it contains all in batch at start)
    count = 0
    for i in range(batch_size):
        for t in range(time_steps):

            #get x from input_data at current timestep
            x = input_data[:,t,:]
            print("input (x) shape: ", np.shape(x))
            print("wt_x * x = ", np.matmul(x, wt_x))
            print("wt_h * h = ", np.matmul(h, wt_h))

            #multiply by weights
            x = np.matmul(x, wt_x)
            h = np.matmul(h, wt_h)
            
            #concatenate
            out = np.concatenate((x,h), axis =0)

            #activation layer
            out = sigmoid(out)

            #merge the two branches
            out = out[:np.shape(out)[0]//2] + out[np.shape(out)[0]//2:]

            #average the four batches to get a output of shape [hidden_size]
            out = np.average(out, axis = 0)

            outputs[i,t,:] = out

            #hold on to the output x to carry over to next loop
            h = outputs[i,t,:] #also not right

            count =+ 1
            print(count)



    state = None

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


    outputs = None
    state = None

    ##################################################################################################
    # Please implement the GRU here.                                                                 #
    ##################################################################################################
        
    
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



    ####################################################################################################
    # Please set a set of parameters for a GRU such that it recovers an RNN with parameters passing in #
    ####################################################################################################
    
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

