import copy
import cv2
import numpy as np 
import xpcv
import Matrix_pb2
import Number_pb2
import csv
from numpy import genfromtxt
import math
import matplotlib.pyplot as plt
import os

# implementation of a custom module
class MyModule(xpcv.Module):
    def __init__(self):
        super().__init__()
        # register pins
        self.addPin(xpcv.InputPin('PointCloudIn', Matrix_pb2.MatrixFloatCh3.DESCRIPTOR, self.onDataIn))
        self.addPin(xpcv.InputPin('ColouredSurfacesIn', Matrix_pb2.MatrixUInt16Ch3.DESCRIPTOR, self.onColIn))
        self.addPin(xpcv.OutputPin('MyOutput', Number_pb2.Float.DESCRIPTOR))
        self.addPin(xpcv.OutputPin('Filtered points', Matrix_pb2.MatrixFloatCh3.DESCRIPTOR))
        self.addPin(xpcv.OutputPin('Filtered colors', Matrix_pb2.MatrixUInt8Ch3.DESCRIPTOR))
        self.addPin(xpcv.OutputPin('Evaluated points', Matrix_pb2.MatrixUInt8Ch1.DESCRIPTOR))
        self.addPin(xpcv.OutputPin('DebugSnippet', Matrix_pb2.MatrixUInt8Ch3.DESCRIPTOR))
        
    # methods for state management (can be removed if not needed)
    def initialize(self):
        self.read_csv()        
        print("Initialize")
        self.hasColors = False
        self.hasPoints = False
        return True

    def start(self):
        print("Start")
        return True

    def stop(self):
        print("Stop")
        pass

    def terminate(self):
        print("Terminate")
        pass

    # callback function of 'PointCloudIn' input pin
    def onDataIn(self, points_in):
        # convert to np array
        self.pointCloudFrame = np.fromstring(points_in.data, np.float32).reshape(points_in.rows, points_in.cols, 3)
        self.pointCloudRows = points_in.rows
        self.pointCloudColumns = points_in.cols
        print("Rows of Point Clouds = ", self.pointCloudRows)
        print("Columns of Point Clouds = ", self.pointCloudColumns)
        self.hasPoints = True
        self.process()
        
    
    # callback function of 'ColouredSurfacesIn' input pin
    def onColIn(self, color_in):
        # convert to np array
        print("start onColIn")
        
        colorImageUInt16 = np.fromstring(color_in.data, np.uint16).reshape(color_in.rows, color_in.cols, 3)

        maxColorChannelVal = np.max(colorImageUInt16)
        if maxColorChannelVal > ((2**16 - 1) + 1e-5):
            print("max color channel value (uint16):", maxColorChannelVal)
            return
 
        scaleFloat = float(1.0 / (2.0**16 - 1.0))
        self.colorImage = colorImageUInt16.astype('float32') * scaleFloat
        maxColorChannelVal = np.max(self.colorImage)
        if maxColorChannelVal > (1 + 1e-5):
            print("max color channel value (float):", maxColorChannelVal)
            return
        
        scaleUInt8 = float((2.0**8 - 1.0) / (2.0**16 - 1.0))
        self.colorImageUInt8 = (colorImageUInt16 * scaleUInt8).astype('uint8')
        
        self.onColInRows = color_in.rows
        self.onColInColumns = color_in.cols
        print("Rows of Coloured Images = ", self.onColInRows)        
        print("Columns of Coloured Images = ", self.onColInColumns)
        self.hasColors = True
        self.process()

    def sendOutputData(self, point3D, bestColorCodes, evalMask):
        
        print("sendOutputData 01")
        
        point3Dcpy = np.array(point3D, copy=True)
        matrix_out = Matrix_pb2.MatrixFloatCh3()
        matrix_out.rows = point3Dcpy.shape[0]
        matrix_out.cols = point3Dcpy.shape[1]
        matrix_out.data = point3Dcpy.tobytes()
        self.getPin('Filtered points').sendData(matrix_out)
        
        print("sendOutputData 02")
        
        #colorImageUInt8Cpy = np.array(self.colorImageUInt8, copy=True)
        #matrix_out4 = Matrix_pb2.MatrixUInt8Ch3()
        #matrix_out4.rows = colorImageUInt8Cpy.shape[0]
        #matrix_out4.cols = colorImageUInt8Cpy.shape[1]
        #matrix_out4.data = colorImageUInt8Cpy.tobytes()
        #self.getPin('Filtered colors').sendData(matrix_out4)
        
        bestColorCodesCpy = (bestColorCodes * 255).astype('uint8')
        np.array(bestColorCodesCpy, copy=True)
        matrix_out5 = Matrix_pb2.MatrixUInt8Ch3()
        matrix_out5.rows = bestColorCodesCpy.shape[0]
        matrix_out5.cols = bestColorCodesCpy.shape[1]
        matrix_out5.data = bestColorCodesCpy.tobytes()
        self.getPin('Filtered colors').sendData(matrix_out5)
        
        print("sendOutputData 03")
        
        evalMaskcpy = 255 * evalMask.astype('uint8')
        matrix_out2 = Matrix_pb2.MatrixUInt8Ch1()
        matrix_out2.rows = evalMaskcpy.shape[0]
        matrix_out2.cols = evalMaskcpy.shape[1]
        matrix_out2.data = evalMaskcpy.tobytes()
        self.getPin('Evaluated points').sendData(matrix_out2)
        
        print("sendOutputData 04")
        
        debugSnippetCpy = np.array(self.debugSnippet, copy=True)
        matrix_out3 = Matrix_pb2.MatrixUInt8Ch3()
        matrix_out3.rows = debugSnippetCpy.shape[0]
        matrix_out3.cols = debugSnippetCpy.shape[1]
        matrix_out3.data = debugSnippetCpy.tobytes()
        self.getPin('DebugSnippet').sendData(matrix_out3)

        print("output sent")

    def process(self):
        print("processing...")
        if self.hasPoints == False:
            return
        if self.hasColors == False:
            return
        l=[]
        try:
            print("X")
            validMaskColors = (self.colorImage != 0)
            validMaskColors = np.any(validMaskColors, axis=2)
            print("shape of validMaskColors:", validMaskColors.shape)
            sumValidMaskColors = np.sum(validMaskColors)
            print("valid points due to color mask:", sumValidMaskColors)
            validMaskPoints = (self.pointCloudFrame != 0) & (self.pointCloudFrame != np.inf) & (self.pointCloudFrame != -np.inf) & (self.pointCloudFrame == self.pointCloudFrame)
            validMaskPoints = np.any(validMaskPoints, axis=2)
            sumValidMaskPoints = np.sum(validMaskPoints)
            print("valid points due to point mask:", sumValidMaskPoints)
            validMask1 = validMaskColors & validMaskPoints
            print("valid points due to combined colored and point mask:", np.sum(validMask1))
            validMask2, bestA, bestB, bestC, bestD, bestColorCodes = self.getParams(self.colorImage)
            valid = validMask1 & validMask2
            print("total valid points", np.sum(valid))
            numValidValues = np.sum(valid)    
            print("debug D")
            point3D = np.array(self.pointCloudFrame, copy=True) # makes a copy

            point3D[np.logical_not(valid)] = 0
            bestColorCodes[np.logical_not(valid)] = 0
            
            print("shape of bestA:", bestA.shape)
            #print("shape of bestB:", bestB.shape)
            #print("shape of bestC:", bestC.shape)
            #print("shape of bestD:", bestD.shape)
            
            point3D /= 1000.0 # because of meter-millimeter conversion
            
            if np.any(point3D != point3D):
                print("point3D contains NaN!!!")
                return
            if np.any(bestA != bestA):
                print("bestA contains NaN!!!")
                return
            if np.any(bestB != bestB):
                print("bestB contains NaN!!!")
                return
            if np.any(bestC != bestC):
                print("bestC contains NaN!!!")
                return
            if np.any(bestD != bestD):
                print("bestD contains NaN!!!")
                return
            
            if np.any(point3D == np.inf) or np.any(point3D == -np.inf):
                print("point3D contains +- inf val!!!")
                return
            if np.any(bestA == np.inf) or np.any(bestA == -np.inf):
                print("bestA contains +- inf val!!!")
                return
            if np.any(bestB == np.inf) or np.any(bestB == -np.inf):
                print("bestB contains +- inf val!!!")
                return
            if np.any(bestC == np.inf) or np.any(bestC == -np.inf):
                print("bestC contains +- inf val!!!")
                return
            if np.any(bestD == np.inf) or np.any(bestD == -np.inf):
                print("bestD contains +- inf val!!!")
                return
    
            self.sendOutputData(point3D, bestColorCodes, valid)

            numerator = np.abs(bestA*point3D[:,:,0] + bestB*point3D[:,:,1] + bestC*point3D[:,:,2] + bestD)
            denominator = np.sqrt((bestA*bestA) + (bestB*bestB) +(bestC*bestC))
            print("debug E")
            #print("shape of denominator:", denominator.shape)
            #print("shape of numerator:", numerator.shape)
            #print("shape of valid mask:", valid.shape)
            
            d = numerator / denominator
            print("distance: " + str(d))

            #print("shape of d:", d.shape)
            d[np.logical_not(valid)] = 0
            
            print("debug F")
            dSum = np.sum(d)
            print("distance summed up: " + str(dSum))
               
            print("debug F.3")
            print("num of valid points: " + str(numValidValues))
            
                        
            # "centroid" of the point cloud (for debugging)
            print("shape of valid:", valid.shape)
            validTmp = np.expand_dims(valid, axis=2)
            valid3D = np.concatenate((validTmp, validTmp, validTmp), axis=2)
            print("debug F.4")
            point3Dvalid = np.array(point3D, copy=True)
            print("debug F.5")
            point3Dvalid[np.logical_not(valid3D)] = 0
            print("debug F.6")
            meanPoint = np.sum(point3Dvalid, axis=(0,1))
            print("mean point shape:", meanPoint.shape)
            print("debug F.7")
            meanPoint /= numValidValues
            print("debug F.8")
            
            #meanPoint = np.array([0, 0, 0])
            #print("debug F.4")
            #print("shape of points3d:", point3D.shape)
            #for i in range(point3D.shape[0]):
                #print("debug F.5")
                #for j in range(point3D.shape[1]):
                    #print("debug F.6")
                    #if valid[i][j] > 0:
                        #print("debug F.7")
                        #point = point3D[i][j]
                        #print("shape of point:", meanPoint.shape)
                        #print("shape of point:", point.shape)
                        #meanPoint = meanPoint + point
                        
            #meanPoint /= numValidValues
            print("centroid at:", meanPoint)
            
            
            goodMeasSharpNoise = dSum / numValidValues
            print("debug goodness measurement: " + str(goodMeasSharpNoise))
            #upperBoundary = 1.2 #means 30 cm   
            upperBoundary = np.inf #for no filtering
            dFiltered = np.array(d, copy=True)
            dFiltered[dFiltered >= upperBoundary] = 0
            print("debug H")
            dSumFiltered = np.sum(dFiltered)
            validFiltered = valid & (d < upperBoundary)
            numValidFilteredValues = np.sum(validFiltered)
            print("debug I")
            goodMeasFilt = dSumFiltered / numValidFilteredValues
                    
        except AttributeError as e:
            print(e)
            return
        except RuntimeError as e:
            print(e)
            return
        print("process 1")    
 #       #goodMeasFilt = np.mean(lFiltered).tolist()
 #       print("type of goodMeas: ", type(goodMeasFilt))
 #       print("The goodness measurement of the filtered part is : ", goodMeasFilt)
 #       path = "/home/pavd/Desktop/Histograms_Measures/GoodnessValue(Filtered)/"
 #       if(os.path.isdir(path) == False):
 #           os.makedirs(path)
 #       fileName1 = "/home/pavd/Desktop/Histograms_Measures/GoodnessValue(Filtered)/GoodnessValue_05_baseline.txt"
 #       fileName1 = "GoodnessValue_50_baseline.txt"
 #       with open(path + fileName1, "w") as f:
 #           f.write("The goodness measurement of filtered part is " + str(float(goodMeasFilt)))
 #       plt.hist(goodMeasFilt, 5, density=True, facecolor='green', alpha=0.75)
 #       plt.ylabel('Frequency')
 #       plt.xlabel('Distances of the point')
 #       plt.title('Histogram of Goodness Measurement (Filtered Part)')
 #       plt.grid(True)
 #       #plt.show()
 #      path = "/home/pavd/Desktop/Histograms_Measures/GoodnessValue(Filtered)/"
 #       if(os.path.isdir(path) == False):
 #           os.makedirs(path)
 #       histFilename = "Histogram_GoodnessValue_50_baseline.pdf"
 #       plt.savefig(path + histFilename, format = "PDF")
       
#       plt.clf()

        distancesFlattened = d.flatten()
        distancesFilteredFlattened = dFiltered.flatten()
        
        validFlattened = valid.flatten()
        
        validDistances = distancesFlattened[validFlattened]
        validDistancesFiltered = distancesFilteredFlattened[validFlattened]
        print("process 2")
        meanD = np.mean(validDistances)
        meanDFiltered = np.mean(validDistancesFiltered)
        #print("The goodness measurement with sharp edge noise is : ", meanD)
        #path = "/home/pavd/Desktop/Histograms_Measures/GoodnessValue_SharpNoise/"
        #if(os.path.isdir(path) == False):
        #    os.makedirs(path)
        #fileName2 = "/home/pavd/Desktop/Histograms_Measures/GoodnessValue_SharpNoise/GoodnessValue-SharpEdgeNoise_05_baseline.txt"
        #command = "sed -i \"s/fileName2 = \\\"GoodnessValue-SharpEdgeNoise.*/fileName2 = \\\"GoodnessValue-SharpEdgeNoise_" + baseline + "_baseline.txt\\\"/\" ../xpcv_evaluation.py"
        
        #fileName2 = "GoodnessValue-SharpEdgeNoise_50_baseline.txt"
        #with open(path + fileName2, "w") as f:
        #    f.write("The goodness measurement of sharp edge noise is " + str(float(meanD)))
        print("process 3")
        ax = plt.subplot(111)
        plt.hist((validDistances / 0.04), 100, density=True, facecolor='blue', alpha=0.75)
        plt.ylabel('Frequency')
        plt.xlabel('Distances of the points [cm]')
        plt.title('Histogram of Goodness Measurement (with noise) - Param: baseline 50 cm')
        ax.set_ylim(0, 0.2)
        ax.set_xlim(0, 200)
        plt.text(40000, 0.000025, 'Goodness Measurement = ' ,goodMeas)
        plt.axis([0, 100000,0.000000, 0.000090])
        plt.grid(True)
        plt.show()
        path = "/home/pavd/Desktop/Histograms_Measures/"
        if(os.path.isdir(path) == False):
            os.makedirs(path)
        histFilename = "Histogram_Unfiltered_50_baseline.pdf"
        plt.savefig(path + histFilename, format = "PDF")
        plt.close('all')
        print("process 4")
        ax2 = plt.subplot(111)
        plt.hist((validDistancesFiltered / 0.04), 100, density=True, facecolor='blue', alpha=0.75)
        
        plt.ylabel('Frequency')
        plt.xlabel('Distances of the points [cm]')
        plt.title('Histogram of Goodness Measurement (filtered) - Param: baseline 50 cm')
        ax2.set_ylim(0, 0.75)
        if upperBoundary != np.inf:
            ax2.set_xlim(0, upperBoundary / 0.04)
        plt.text(40000, 0.000025, 'Goodness Measurement = ' ,goodMeas)
        plt.axis([0, 100000,0.000000, 0.000090])
        plt.grid(True)
        plt.show()
        path = "/home/pavd/Desktop/Histograms_Measures/"
        if(os.path.isdir(path) == False):
            os.makedirs(path)
        histFilename = "Histogram_Filtered_50_baseline.pdf"
        plt.savefig(path + histFilename, format = "PDF")
        plt.close('all')
        
        
        print("filtered/unfiltered:\t" + str(goodMeasFilt) + "\t" + str(goodMeasSharpNoise))
        
        # sending result
        out = Number_pb2.Float()
        out.value = goodMeasFilt
        self.getPin('MyOutput').sendData(out)
        
        print("Process Over")
            

    def getParams(self,color):
        # finding the plane with the smalest color distance to pointColor
        numOfCols = len(self.colorIDRedList)
        colorThreshold = 0.20
        
        #distancesRed   = np.zeros((color.shape[0], color.shape[1], numOfCols))
        #distancesGreen = np.zeros((color.shape[0], color.shape[1], numOfCols))
        #distancesBlue  = np.zeros((color.shape[0], color.shape[1], numOfCols))
        sqDistance       = np.zeros((color.shape[0], color.shape[1], numOfCols))
        
        curColorIDReds = []
        curColorIDGreens = []
        curColorIDBlues = []

        for i in range(numOfCols):
            curColorIDRed   = np.ones((color.shape[0], color.shape[1])) * self.colorIDRedList[i]
            curColorIDGreen = np.ones((color.shape[0], color.shape[1])) * self.colorIDGreenList[i]
            curColorIDBlue  = np.ones((color.shape[0], color.shape[1])) * self.colorIDBlueList[i]
            sqDistancesRed   = np.square(curColorIDRed   - color[:,:,2])
            sqDistancesGreen = np.square(curColorIDGreen - color[:,:,1])
            sqDistancesBlue  = np.square(curColorIDBlue  - color[:,:,0])
            sqDistance[:,:,i] = sqDistancesRed + sqDistancesGreen + sqDistancesBlue
            curColorIDReds += [curColorIDRed]
            curColorIDGreens += [curColorIDGreen]
            curColorIDBlues += [curColorIDBlue]
        
        index_min =  np.argmin(sqDistance, axis=2)
        min_values = np.min(sqDistance, axis=2)
        
        valid = min_values < colorThreshold
        bestA = np.zeros((color.shape[0], color.shape[1]))
        bestB = np.zeros((color.shape[0], color.shape[1]))
        bestC = np.zeros((color.shape[0], color.shape[1]))
        bestD = np.zeros((color.shape[0], color.shape[1]))

        bestColorCodes = np.zeros((color.shape[0], color.shape[1], 3))

        for i in range(color.shape[0]):
            for j in range(color.shape[1]):
                bestA[i,j] = self.aList[index_min[i,j]]
                bestB[i,j] = self.bList[index_min[i,j]]
                bestC[i,j] = self.cList[index_min[i,j]]
                bestD[i,j] = self.dList[index_min[i,j]]
                bestColorCodes[i,j,2] = self.colorIDRedList[index_min[i,j]]
                bestColorCodes[i,j,1] = self.colorIDGreenList[index_min[i,j]]
                bestColorCodes[i,j,0] = self.colorIDBlueList[index_min[i,j]]
        
        # debug point 
        
        dpI, dpJ = [640, 840]
        
        # debug area
        snippetSize = 100
        
        topY = int(dpI - snippetSize / 2)
        bottomY = int(dpI + snippetSize / 2)
        leftX = int(dpJ - snippetSize / 2)
        rightX = int(dpJ + snippetSize / 2)
        
        print("color.shape:", color.shape)
        
        self.debugSnippet = np.array(color[topY:bottomY, leftX:rightX, :], copy=True)
        print("self.debugSnippet:", self.debugSnippet.shape, self.debugSnippet.dtype)
        
        
        self.debugSnippet = self.debugSnippet * (2**8 - 1)
        print("self.debugSnippet:", self.debugSnippet.shape, self.debugSnippet.dtype)
    
        self.debugSnippet = self.debugSnippet.astype('uint8')
        print("self.debugSnippet:", self.debugSnippet.shape, self.debugSnippet.dtype)
        
        print("debug point:", [dpI, dpJ])
        print("color at debug point:", color[dpI, dpJ,:])
        print("bestA at debug point:", bestA[dpI, dpJ])
        print("bestB at debug point:", bestB[dpI, dpJ])
        print("bestC at debug point:", bestC[dpI, dpJ])
        print("bestD at debug point:", bestD[dpI, dpJ])
        print("index_min for debug point:", index_min[dpI,dpJ])
        
        return valid, bestA, bestB, bestC, bestD, bestColorCodes
        

    #function to read csv file
    def read_csv(self):    
        #my_data = np.loadtxt('Colors_list.csv', names = column_names)
        column_names = ('ObjectID', 'ColorIDRed', 'ColorIDGreen', 'ColorIDBlue' 'A', 'B', 'C', 'D')
        my_data = genfromtxt("/home/pavd/Desktop/mt_omnistereo/Colors_list.csv", delimiter = ';', skip_header=1)
        #print("Colour list display ", my_data)
        self.objectIDList = my_data[:,0].tolist()
        self.colorIDRedList = my_data[:,1].tolist()
        self.colorIDGreenList = my_data[:,2].tolist()
        self.colorIDBlueList = my_data[:,3].tolist()
        self.aList = my_data[:,4].tolist()
        self.bList = my_data[:,5].tolist()
        self.cList = my_data[:,6].tolist()
        self.dList = my_data[:,7].tolist()
  
# function 'setupModule()' must return an instance of xpcv.Module
def setupModule():
    return MyModule()

