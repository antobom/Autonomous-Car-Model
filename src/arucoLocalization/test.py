import numpy as np

def loadCalibrationMatrixCoef():
    f = open("camera_calibration_matrix.csv")

    calibration_matrix = np.zeros((3* 3, ))
    dist_coef = np.zeros((5,))
    line = f.readline()

    for i, str_value in enumerate(line.split(sep=";")):
        if i<9:
            calibration_matrix[i] = float(str_value)
        else:
            dist_coef[i-9] = float(str_value)
    calibration_matrix = calibration_matrix.reshape(3, 3)
    
    return calibration_matrix, dist_coef

c, d = loadCalibrationMatrixCoef()

print(c)
print()
print(d)
