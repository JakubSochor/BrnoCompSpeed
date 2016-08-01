# -*- coding: utf-8 -*-

"""
For which systems the evaluation should be done.
It needs to be in the results dir defined in dataset_info.py
"""
RUN_FOR_SYSTEMS = ( "dubska_bmvc14", 
                    "dubska_optimal_scale", 
                    "dubska_optimal_scale_vp2", 
                    "dubska_optimal_calib", 
                    "dubska_optimal_calib_vp2")
                    
"""
For which video the evaluation should be done. 
You can use keys A,B or C.
See dataset_info.py for more information and 
training videos for each splitting
"""
RUN_FOR_VIDEOS = SPLIT_TEST_VIDEOS["A"]


"""
Defines systems and thresholds
If a speed estmiation error for a system in the set is larger
then the threshold, the trajectory is shown
WARNING: You need to delete the cached results (or use -rc argument)
"""
SHOW_BAD_FOR_SYSTEMS = set()
SHOW_BAD_THRESHOLD = 30


"""
If true, vehicles' trajectories for which the computation
of intersections wihh measurement lines failes are shown
WARNING: You need to delete the cached results (or use -rc argument)
"""
SHOW_ERRORS = False



#%%
"""
##############################################################
################### PRESENTATION HELPER FUNCTIONS ############
##############################################################
"""

"""
Conversions for plotting and printing statistics
"""
def labelConversion(systemId):
    if systemId == "dubska_bmvc14":
        return "FullACC [7]"
    elif systemId == "dubska_optimal_scale":
        return "OptScale"
    elif systemId == "dubska_optimal_scale_vp2":
        return "OptScaleVP2"
    elif systemId == "dubska_optimal_calib":
        return "OptCalib"
    elif systemId == "dubska_optimal_calib_vp2":
        return "OptCalibVP2"
    return systemId

"""
Styles for cumulative histograms
"""
def plotStyleCumulativeHist(systemId):
    styleDict = {"linewidth":2}
        
    if systemId == "dubska_bmvc14":
        styleDict["color"] = "black"
        styleDict["linewidth"] = 3
    elif systemId == "dubska_optimal_scale":
        styleDict["color"] = "#00B0F0"
    elif systemId == "dubska_optimal_scale_vp2":
        styleDict["color"] = "darksage"
    elif systemId == "dubska_optimal_calib":
        styleDict["color"] = "#A40000"
    elif systemId == "dubska_optimal_calib_vp2":
        styleDict["color"] = "#FF9900"
    
    return styleDict


"""
Styles for error histograms
"""
def plotStyleErrorHist(systemId):
    styleDict = {"linewidth":2}
        
    if systemId == "dubska_bmvc14":
        styleDict["color"] = "black"
    elif systemId == "dubska_optimal_scale":
        styleDict["color"] = "#00B0F0"
    elif systemId == "dubska_optimal_scale_vp2":
        styleDict["color"] = "darksage"
    elif systemId == "dubska_optimal_calib":
        styleDict["color"] = "#A40000"
    elif systemId == "dubska_optimal_calib_vp2":
        styleDict["color"] = "#FF9900"
    
    return styleDict
    
