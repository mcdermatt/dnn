import numpy as np
import warnings

def dropout_forward(x, drop_rate, mode):
    """
    Performs the forward pass for dropout.

    Inputs:
    - x: Input data, of any shape
    - drop_rate: Dropout parameter. We keep each neuron output with probability p.
    - mode: 'test' or 'train'. If the mode is train, then perform dropout;
            if the mode is test, then just return the input.

    Outputs:
    - out: Array of the same shape as x.
    """

    #TODO 
    #   store mask

    mask = None
    out = None

    if mode == 'train':
        #######################################################################
        # TODO: Implement training phase forward pass for inverted dropout.   #
        # Store the dropout mask in the mask variable.                        #
        #######################################################################
        
        rand = np.random.rand(*x.shape) # random vector of same length as x

        rand[rand < drop_rate] = 0 # for each element of rand if val < drop_rate, set weight to zero

        rand[rand >= drop_rate] = 1/(1-drop_rate)

        mask = rand #double check this
        out = x*rand

        #######################################################################
        #                           END OF YOUR CODE                          #
        #######################################################################
    elif mode == 'test':
        #######################################################################
        # TODO: Implement the test phase forward pass for inverted dropout.   #
        #######################################################################
        
        mask = 1 #do not perform dropout when testing

        out = x
        #######################################################################
        #                            END OF YOUR CODE                         #
        #######################################################################


    return out



def conv_forward(x, w, b, conv_param):
    """
    An implementation of the forward pass for a convolutional layer.

    The input consists of N data points, each with C channels, height H and
    width W. We convolve each input with F different filters, where each filter
    spans all C channels and has height HH and width WW.

    Input:
    - x: Input data of shape (N, H, W, C)
    - w: Filter weights of shape (HH, WW, C, F)
    - b: Biases, of shape (F,)
    - conv_param: A dictionary with the following keys:
      - 'stride': The number of pixels between adjacent receptive fields in the
        horizontal and vertical directions. 

      - 'pad': The number of pixels that will be used to zero-pad the input. 
        
    During padding, 'pad' zeros should be placed symmetrically (i.e equally on both sides)
    along the height and width axes of the input. Be careful not to modfiy the original
    input x directly.

    Returns a tuple of:
    - out: Output data, of shape (N, H', W', F) where H' and W' are given by
      H' = (H + 2 * pad - HH + stride) // stride
      W' = (W + 2 * pad - WW + stride) // stride
    """
    out = None
    ###########################################################################
    # TODO: Implement the convolutional forward pass.                         #
    # Hint: you can use the function np.pad for padding.                      #
    ###########################################################################

    #get values out of conv_param
    pad = conv_param["pad"]
    stride = conv_param["stride"] #how far the window slides on each step

    #get height and width of filter
    HH = w.shape[0]
    WW = w.shape[1]
    #get height and width of padded base image
    H = x.shape[2] + pad*2
    W = x.shape[1] + pad*2

    #init output
    Hout = (H + 2*pad - HH + stride) // stride 
    Wout = (W + 2 * pad - WW + stride) // stride
    out = np.zeros([x.shape[0],Hout-1,Wout-1,x.shape[3]])
    
    print("W = ", W, " WW = ", WW, " H = ", H, " HH = ", HH )
    # print((W-WW)//stride)
    # print((H-HH)//stride)

    #slide top to bottom
    for j in range((H-HH)//stride - 1):
        #slide left to right
        for k in range((W-WW)//stride - 1):
            # #trying no loop...
            # padx = x[n,:,:,f]
            # padx = np.pad(padx,pad) 
            # cropped = padx[j*stride:(j*stride+HH),k*stride:(k*stride+WW)]
            # out[:,j,k,:] = np.sum(cropped*w)

            #DEBUG - Do I actually need to loop through this?
            #for each data point
            for n in range(x.shape[0]): #N
                #for each filter
                for f in range(x.shape[3]): #F

                    #pad as a function of height and width of filter
                    #   IMPORTANT: ONLY PAD H AND W PARAMS - NOT F AND N
                    padx = x[n,:,:,f]
                    padx = np.pad(padx,pad) 

                    #get cropped matrix from base image
                    cropped = padx[j*stride:(j*stride+HH),k*stride:(k*stride+WW)]
                    # cropped = padx[j*stride:(j*stride+HH)-1,k*stride:(k*stride+WW)-1]


                    # out[n,j,k,f] = np.sum(cropped*x[n,j,k,f]) #sum(cropped*Filter)          
                    
                    # out[n,j,k,f] = np.sum(cropped*w) #sum(cropped*Filter)
                    out[n,j,k,f] = np.sum(x[n,j,k,f]*cropped) #sum(cropped*Filter)





    ###########################################################################
    #                             END OF YOUR CODE                            #
    ###########################################################################


    return out



def max_pool_forward(x, pool_param):
    """
    A implementation of the forward pass for a max-pooling layer.

    Inputs:
    - x: Input data, of shape (N, H, W, C)
    - pool_param: dictionary with the following keys:
      - 'pool_height': The height of each pooling region
      - 'pool_width': The width of each pooling region
      - 'stride': The distance between adjacent pooling regions

    No padding is necessary here. Output size is given by 

    Returns a tuple of:
    - out: Output data, of shape (N, C, H', W') where H' and W' are given by
      H' = 1 + (H - pool_height) / stride
      W' = 1 + (W - pool_width) / stride
    """
    out = None
    ###########################################################################
    # TODO: Implement the max-pooling forward pass                            #
    ###########################################################################
    pass
    ###########################################################################
    #                             END OF YOUR CODE                            #
    ###########################################################################



    return out


