#!/usr/bin/python3
#autor Julian Seuffert

from adaptProjTools import *

import psutil
import subprocess
import sys
import re
import os

# importing
if len(sys.argv) == 1:
    print("usage: python3 " + sys.argv[0] + " <projectName>")
    exit(1)


myProjPath = sys.argv[1]
backupProject(myProjPath)
myProjectNode = importProject(myProjPath)

# adaptions
experimentDir = "/home/pavd/Desktop/mt_omnistereo/MasterThesis_Baseline_related/Textured_baselines_Experiment1/"
colourDir = "/home/pavd/Desktop/mt_omnistereo/MasterThesis_Baseline_related/Coloured_baselines_blackenedAreas1_Experiment1/C0/"
calibListPath = experimentDir + "CalibrationFiles/c0calibFiles.txt"
# xpcvPath = "/opt/xpcv/bin/xpcv_project_launcher"
xpcvPath = "/opt/xpcv/bin/xpcv_gui"
#stoppingText = "Point cloud was written to:"
stoppingText = "Process Over"
baselinePattern = re.compile(r"C0_(?P<baseline>\w+)_")


with open(calibListPath) as f:
    for calibPath in f:
        c0       = calibPath.strip()
        c0a      = c0.replace("C0_", "C0a_")
        c0b      = c0.replace("C0_", "C0b_")
        c0noDist = c0.replace("C0_", "C0_noDist_")
        c1       = c0.replace("C0_", "C1_")
        c1a      = c0.replace("C0_", "C1a_")
        c2       = c0.replace("C0_", "C2_")
        c2b      = c0.replace("C0_", "C2b_")

        m = baselinePattern.search(c0)
        baseline = m.group("baseline")
        print("current baseline is:", baseline)

        c0img    = experimentDir + "/C0/camop0_" + baseline + "_baseline.png" 
        c1img    = experimentDir + "/C1/camop1_" + baseline + "_baseline.png"
        c2img    = experimentDir + "/C2/camop2_" + baseline + "_baseline.png"

        # adapt calibration file paths

        moduleNames = ["Calib C0 noDist", "Calib C0", "Calib C1", "Calib C2", "Calib C0a", "Calib C1a", "Calib C0b", "Calib C2b"]
        calibs = [c0noDist, c0, c1, c2, c0a, c1a, c0b, c2b]
        for c in calibs:
            if not os.path.exists(c):
                print("calibration file " + c + " not found!")
                restoreProject(myProjPath)
                exit(1)

        for i in range(len(moduleNames)):
            ret = replace_calib_path(myProjectNode, moduleNames[i], calibs[i])
            if ret == False:
                print("could not find module", moduleNames[i])
                restoreProject(myProject)
                exit(1)

        # adapt image paths

        moduleNames = ["C0 file input", "C1 file input", "C2 file input"]
        imgs = [c0img, c1img, c2img]

        for i in imgs:
            if not os.path.exists(i):
                print("image file " + i + " not found!")
                restoreProject(myProjPath)
                exit(1)

        for i in range(len(moduleNames)):
            ret = replace_img_path(myProjectNode, moduleNames[i], imgs[i])
            if ret == False:
                print("could not find module", moduleNames[i])
                restoreProject(myProjPath)
                exit(1)

        # adapt Colour maps
        edit_prop(myProjectNode, "ColouredImageFileInput", "File(s)", colourDir + "camop0_" + baseline + "_baseline.png")

        # adapt output path
        edit_prop(myProjectNode, "PointCloudSink", "Filename prefix", "ex1_baseline_")
        edit_prop(myProjectNode, "PointCloudEvalSink", "Filename prefix", "ex1_baseline_eval_")

        edit_prop(myProjectNode, "PointCloudSink", "Index min", baseline)
        edit_prop(myProjectNode, "PointCloudEvalSink", "Index min", baseline)
        
        # adapt rectification output paths
        edit_prop(myProjectNode, "C0a Output", "Index min", baseline)
        edit_prop(myProjectNode, "C1a Output", "Index min", baseline)
        edit_prop(myProjectNode, "C0b Output", "Index min", baseline)
        edit_prop(myProjectNode, "C2b Output", "Index min", baseline)
        
        # adapt extrinsic modifier
        extrinsicsPath = os.path.dirname(c0) + "/extrinsics_" + baseline + "_baseline.xml"
        edit_nested_prop(myProjectNode, "ExtrinsicModifier", "Extrinsic", "Calibration File", extrinsicsPath)

        
        
#        # adapt hist out path - goodnessValue
#        command = "sed -i \"s/histFilename = \\\"Histogram_GoodnessValue.*/histFilename = \\\"Histogram_GoodnessValue_" + baseline + "_baseline.pdf\\\"/\" ../xpcv_evaluation.py"
#        os.system(command)
#        try:
#            os.wait()
#        except:
#            pass
#        
#        
        
        #adapt Hist title
        command = "sed -i \"s/- Param:.*'/- Param: baseline " + baseline + " cm'/g\" ../xpcv_evaluation.py"
        os.system(command)
        try:
            os.wait()
        except:
            pass


        # adapt hist out path - sharpEdgeNoise
        command = "sed -i \"s/histFilename = \\\"Histogram_Filtered.*/histFilename = \\\"Histogram_Filtered_" + baseline + "_baseline.pdf\\\"/\" ../xpcv_evaluation.py"
        os.system(command)
        try:
            os.wait()
        except:
            pass
        
        # adapt hist out path - sharpEdgeNoise
        command = "sed -i \"s/histFilename = \\\"Histogram_Unfiltered.*/histFilename = \\\"Histogram_Unfiltered_" + baseline + "_baseline.pdf\\\"/\" ../xpcv_evaluation.py"
        os.system(command)
        try:
            os.wait()
        except:
            pass


        # adapt text file path - goodnessValue
        command = "sed -i \"s/fileName1 = \\\"GoodnessValue.*/fileName1 = \\\"GoodnessValue_" + baseline + "_baseline.txt\\\"/\" ../xpcv_evaluation.py"
        os.system(command)
        try:
            os.wait()
        except:
            pass

        
        
        # adapt text file path - goodnessValue -sharp edge noise
        command = "sed -i \"s/fileName2 = \\\"GoodnessValue-SharpEdgeNoise.*/fileName2 = \\\"GoodnessValue-SharpEdgeNoise_" + baseline + "_baseline.txt\\\"/\" ../xpcv_evaluation.py"
        os.system(command)
        try:
            os.wait()
        except:
            pass


        # exporting
        exportProject(myProjPath, myProjectNode)

        # call XPCV
        proc = subprocess.Popen([xpcvPath,'--project', myProjPath, '--autostart', '--loglevel', 'info'],stdout=subprocess.PIPE, encoding='utf8')
        psutilProcHandle = psutil.Process(proc.pid)
        while proc.poll() == None:
            line = proc.stdout.readline()
            print(line, end="")
            if stoppingText in line:
                for subProc in psutilProcHandle.children(recursive=True):
                    subProc.kill()
                psutilProcHandle.kill()
                break
                

#restoreProject(myProjPath)
print("done")
