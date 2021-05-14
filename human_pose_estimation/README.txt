To run main visualization:

	<python main.py>

		press "Q" to view actual human movemnt
			-ground truth human is solid filled

		press "W" to view network's real time estimates of human pose
			-estimates appear as transparent drawings of the human

		right click and draw to rotate/ zoom

		left click and drag to pan


		comment/ uncomment lines 31-33 / 36-38 to switch between linear force and sinusoidal force trajectories

To generate training data

	Install MatLab SimScape MultiBody
	Install MatLab Parallel Computing Toolbox

	<generate_trajectories_parallel.m>

		edit "numTraj" on line 18 to adjust desired number of trajectories needed 

		edit "numSims" on line 25 to adjust the max number of simulations to store on RAM between checkpoints (check your RAM specs)

		uncomment lines 154-155 OR 162-162 depending on whether you want translation/ rotation data or accerometer/ gyro data


<notebook.ipynb>

	To train network

		run first cell to import dependencies and initalize GPU
			comment out the physical_devices = ... line if not using a GPU (my old 1060 makes it so I need those two lines)

		run cell "import and augment data by rotating about y axis"
			I have a large dataset of 1,000,000 samples stored locally on my machine but it is >500 Mb so I only included
			a dataset of 10k samples

		run cell "find and fix errors in data"

		run cell to import Net4

		run next cell to train model on this network, adjust hyperparams as needed

	To test model
		
		load or train model 
			existing model can be loaded with:
				 <model = tf.keras.models.load_model("10DOF.kmod")>

		run cell "load larger test dataset"
			if model is trained using 10k dataset, you need to test with the 1k dataset

		run cell "test model"

