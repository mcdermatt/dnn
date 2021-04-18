import tensorflow as tf
import tensorflow.keras as keras

def Net(maxLen, voc_size):

    model = tf.keras.Sequential([
        tf.keras.layers.InputLayer(input_shape=(100,)),
        tf.keras.layers.Lambda(lambda x: x+1),
        tf.keras.layers.Embedding(
            input_dim=voc_size + 1,
            output_dim=95,
            # Use masking to handle the variable sequence lengths
            mask_zero=True),
        # tf.keras.layers.SimpleRNN(10),
        tf.keras.layers.GRU(256, activation = 'tanh', dropout = 0.05),
        tf.keras.layers.Dense(10, activation='relu'),
        tf.keras.layers.Dense(1)
        ])


    return model