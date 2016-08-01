BrnoCompSpeed dataset related to publication:
SOCHOR Jakub et al. BrnoCompSpeed: Comprehensive Dataset for Automatic Single Camera Speed Measurement from Video, IEEE T-ITS 
Contact: {isochor,herout,ijuranek}@fit.vutbr.cz

Additional information, and bibtex citation can be found at https://medusa.fit.vutbr.cz/traffic/
If you use this dataset, please cite our paper.

For information about vehicle types from our country (Czech Republic) you can use:
COD20K dataset http://www.fit.vutbr.cz/research/groups/graph/PoseEstimation/
BoxCars21k dataset https://medusa.fit.vutbr.cz/traffic/


Structure:
----------
code - evaluation code (python2)
     - configuration is in config.py
     - to evaluate it simply run eval.py and wait, it will take a while
     - I STRONGLY recommend to use ipython notebook or other terminal with graph plotting
     - packages you might have: numpy, scipy, matplotlib, tabulate


dataset - dataset itself, containes videos, mask, screenshots, and pkl file with the ground truth data
        - the dataset contains extra session0 which was annotated manually and is not included in the paper
        - the session0 is meant to be as training for all splittings (A,B,C - see the paper)



results - should have same structure as dataset and contain json files with results
        - the structure of the json file should be following:
        {
        "camera_calibration": {"vp1": [x,y], "vp2": [x,y], "pp": [x,y], "scale": lambda}, (see the paper and supplementary pdf for information how this is used)
        "cars":[
        	{
        		"id": number,
        		"frames", "posX", "posY": each key defining list of positions of the car in the video
        		posX and posY should contain x,y coordinates of a point of the vehicle which is on the road plane
        		and frames should contain frame numbers of the reported points. The length of the vectors must be equal.
        		Examples can be found in the results directory. 
        	}
        ]
        }