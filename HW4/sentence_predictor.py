import tensorflow as tf
import tensorflow.keras as keras
from rnn_lm import masked_lm_loss

def generate_model(train_x, train_y, vocSize, maxLen):

    #train model on data of large batch sizes ------------------------------------------------------------------
    batch_size = 64 #network does not like BatchSize = 512??
    runLen = 5 #total number of epochs

    model_batch = Net(maxLen, vocSize, batchSize = batch_size, stateful = False) #TODO should stateful be true???
    model_batch.summary()

    def scheduler(epoch, lr):
   
        part1 = runLen//3
        part2 = 2*runLen//3
    
        if epoch < part1:
            lr = 0.01
            return lr
        if epoch >= part1 and epoch < part2:
            lr = 0.001
            return lr
        if epoch >= part2:
            lr = 0.0001
            return lr

    callback = tf.keras.callbacks.LearningRateScheduler(scheduler)

    #using my own loss function
    model_batch.compile(optimizer=tf.keras.optimizers.Adam(lr = 0.01), loss=masked_lm_loss) #TODO use metrics = masked_lm_loss

    model_batch.fit(x=train_x, y=train_y, epochs=runLen, batch_size=batch_size, callbacks = [callback], shuffle = True, validation_split=0.1)

    #actual model can only be used on a batch size of 1 (so that we can do sentence prediction)-----------------
    model = Net(maxLen, vocSize, batchSize = 1, stateful = True)

    #copy weights from the trained model to this new model
    for il, layer in enumerate(model_batch.layers):
        model.layers[il].set_weights(layer.get_weights())    

    return model

def Net(maxLen, vocSize, batchSize = 1, stateful = False):

    drop = 0.05 #ideal?

    model = tf.keras.Sequential()
    #was this
    model.add(tf.keras.layers.InputLayer(batch_input_shape=(batchSize, maxLen, 1))) #[batch size, max sentence length, ???]
    model.add(tf.keras.layers.Lambda(lambda x: tf.squeeze(x + 1, axis=[-1])))
    model.add(tf.keras.layers.Embedding(input_dim=vocSize + 1, output_dim=95, input_length=maxLen)) #test new output dims


    #NOTE: be careful of interactions between dropout and normalization

    # model.add(tf.keras.layers.BatchNormalization())
    model.add(tf.keras.layers.GRU(256, activation = 'tanh', return_sequences = True, dropout = 0.05, stateful = stateful)) #was 1024
    # model.add(tf.keras.layers.LSTM(95, activation = 'tanh', return_sequences = True, stateful = stateful))
    model.add(tf.keras.layers.Dropout(drop)) #add if overfitting
    model.add(tf.keras.layers.Dense(units = vocSize))


    #output of model should be onehot


    return model
