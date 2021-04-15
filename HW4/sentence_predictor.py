import tensorflow as tf
import tensorflow.keras as keras
from rnn_lm import masked_lm_loss

def generate_model(train_x, train_y, vocSize, maxLen):

    #train model on data of large batch sizes ------------------------------------------------------------------
    batch_size = 64 #network does not like BatchSize = 512??
    model_batch = Net(maxLen, vocSize, batchSize = batch_size, stateful = False)
    model_batch.summary()

    #using my own loss function
    model_batch.compile(optimizer=tf.keras.optimizers.Adam(lr=0.01), loss=masked_lm_loss)

    model_batch.fit(x=train_x, y=train_y, epochs=4, batch_size=batch_size)

    #actual model can only be used on a batch size of 1 (so that we can do sentence prediction)-----------------
    model = Net(maxLen, vocSize, batchSize = 1, stateful = True)

    #copy weights from the trained model to this new model
    for il, layer in enumerate(model_batch.layers):
        model.layers[il].set_weights(layer.get_weights())    

    return model

def Net(maxLen, vocSize, batchSize = 1, stateful = False):

    model = tf.keras.Sequential()
    model.add(tf.keras.layers.InputLayer(batch_input_shape=(batchSize, 100, 1)))
    model.add(tf.keras.layers.Lambda(lambda x: tf.squeeze(x + 1, axis=[-1])))
    model.add(tf.keras.layers.Embedding(input_dim=vocSize + 1, output_dim=512, input_length=maxLen)) #test new output dims

    #not very good
    # model.add(tf.keras.layers.SimpleRNN(95, activation='tanh', return_sequences=True, stateful=stateful))

    #better?

    model.add(tf.keras.layers.GRU(1024, activation = 'tanh', return_sequences = True, stateful = stateful))
    #if overfitting add dropout here
    model.add(tf.keras.layers.Dense(units = 96, activation = 'softmax'))


    #output of model should be onehot


    return model
