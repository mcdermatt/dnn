import tensorflow as tf

"""
This is a short tutorial of tensorflow. After this tutorial, you should know the following concepts:
1. constant,
2. operations
3. variables 
4. gradient calculation 
5. optimizer 
"""

def regression_func(x, w, b):
    """
    The function of a linear regression model
    args: 
        x: tf.Tensor with shape (n, d) 
        w: tf.Variable with shape (d,) 
        b: tf.Variable with shape ()

    return: 
        y_hat: tf.Tensor with shape [n,]. y_hat = x * w + b (matrix multiplication)
    """
    # consider these functions: `tf.matmul`, `tf.einsum`, `tf.squeeze` 

    # Multivariable Regression:
    # w = weight vector
    # b = bias (y intercept in 2d case)

    # f(x,y,z) = w1*x + w2*y + w3*z + b
    # y_hat = (x*w) + b

    #need to expand dims since w comes in as shape(2,) which is rank 1 (need at least rank 2) 
    # y_hat = tf.squeeze(tf.einsum('ij,jk->ki',x,tf.expand_dims(w,axis=1))) + b 

    y_hat = tf.tensordot(x, w, axes=1) + b

    return y_hat



def loss_func(y, y_hat):
    """
    The loss function for linear regression

    args:
        y: tf.Tensor with shape (n,) 
        y_hat: tf.Tensor with shape (n,) 

    return:
        loss: tf.Tensor with shape (). loss = (y -  y_hat)^\top (y -  y_hat) 

    """
    # TODO: implement the function. 
    # Consider these functions: `tf.square`, `tf.reduce_sum`

    loss = tf.reduce_sum(tf.square(y_hat - y))

    return loss



def train_lr(x, y, lamb):
    """
    Train a linear regression model.

    args:
        x: tf.Tensor with shape (n, d)
        y: tf.Tensor with shape (n, )
        lamb: tf.Tensor with shape ()
    """
    
    # TODO: implement the function.
    # initialize parameters w and b
    w = tf.Variable([0.0])
    b = tf.Variable(0.0)

    # set an optimizer
    # please check the documentation of tf.keras.optimizers.SGD
    optim = tf.keras.optimizers.SGD(learning_rate = 0.001)

    # loop to optimize w and b 
    for i in range(1000):

        with tf.GradientTape() as gt:
            gt.watch([w, b])
            y_hat = regression_func(x, w, b)
            loss = loss_func(y, y_hat)

        dw, db = gt.gradient(loss, [w,b])

        del gt

        optim.apply_gradients(zip([dw,db],[w,b]))


    return w, b

