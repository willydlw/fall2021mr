import numpy as np 

def main():
   # Text file data converted to integer data type
   file_array = np.loadtxt("calibrationData.txt", dtype=np.int16)
  
   print(file_array.shape)
   sensor_data = file_array.flatten()
   print(sensor_data.shape)

   samples = int(sensor_data.size / 6)
   ax = sensor_data[0:sensor_data.size:6]
   ay = sensor_data[1:sensor_data.size:6]
   az = sensor_data[2:sensor_data.size:6]
   gx = sensor_data[3:sensor_data.size:6]
   gy = sensor_data[4:sensor_data.size:6]
   gz = sensor_data[5:sensor_data.size:6]


   f = open("biasOffset.txt","w")
   f.write("{:d} ".format(int(0-ax.mean())))
   f.write("{:d} ".format(int(0-ay.mean())))
   f.write("{:d} ".format(int(16384-ay.mean())))
   f.write("{:d} ".format(int(0-gx.mean())))
   f.write("{:d} ".format(int(0-gy.mean())))
   f.write("{:d} ".format(int(0-gz.mean())))
    
   f.close()
   



if __name__ == '__main__':
    main()