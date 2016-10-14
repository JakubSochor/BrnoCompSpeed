# BrnoCompSpeed dataset evaluation code

Dataset published with paper **SOCHOR Jakub et al. BrnoCompSpeed: Review of Traffic Camera Calibration and A Comprehensive Dataset for Monocular Speed Measurement, IEEE T-ITS**

The dataset can be downloaded from [our website](https://medusa.fit.vutbr.cz/traffic/research-topics/traffic-camera-calibration/brnocompspeed/). BibTeX citation is also on [the site](https://medusa.fit.vutbr.cz/traffic/research-topics/traffic-camera-calibration/brnocompspeed/).


## How to use the code

1. Install **PYTHON2**, packages you might NOT have: `numpy, scipy, matplotlib, tabulate`
1. Download the [dataset](https://medusa.fit.vutbr.cz/traffic/research-topics/traffic-camera-calibration/brnocompspeed/) and place the results and dataset folders on the same level as the code folder (root of the repository). 
2. (Optional) Modify paths in file code/dataset_info.py
3. Check file code/config.py. The most important variables are `RUN_FOR_SYSTEMS` and `RUN_FOR_VIDEOS`.
4. Run in code direcotory: `python eval.py` and wait. The results will be computed, shown and cached in results directory. The script `eval.py` has several arguments, so you can use `--help` for explanation of the arguments. I STRONGLY recommend to use ipython, spyder or similar terminals. 


## Additional information

* For information about vehicle types from our country (Czech Republic), you can use following datasets:  [COD20K](http://www.fit.vutbr.cz/research/groups/graph/PoseEstimation/), [BoxCars21k](https://medusa.fit.vutbr.cz/traffic/).
* The dataset itself containes videos, mask, screenshots, and pkl file with the ground truth data
* The dataset contains extra session0 which was annotated manually and is not included in the paper
* The session0 is meant to be as training for all the splits (A,B,C - see the paper)


## How to generate your own result JSON files
* Just place them into appropriate subdirectory in the results directory 
* The JSON files should have same structure as the already existing JSON files
* The structure of the files should be following:

		{
		        "camera_calibration": 
					{"vp1": [x,y], 
					 "vp2": [x,y], 
                     "pp": [x,y], 
                     "scale": lambda}, (see the paper and supplementary pdf for information how this is used)
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
		}

## Contact 
{isochor,herout,ijuranek}@fit.vutbr.cz
