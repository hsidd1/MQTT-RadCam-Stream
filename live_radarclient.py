import serial
import time
import numpy as np
#import matplotlib.pyplot as plt
import struct
import json
#import pyqtgraph.opengl as gl
from processModule.serverConnect import connect_mqtt

# Change the configuration file name
#configFileName = 'xwr68xxconfig.cfg'
configFileName = 'ODS_6m_default.cfg'
#configFileName = 'ISK_6m_default.cfg'
#configFileName = 'new_cfg.cfg'
#configFileName = 'demo.cfg'
write_radar = False
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0;

#-------------------------------------------------------------------
# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    
    # Raspberry pi
    #CLIport = serial.Serial('/dev/ttyACM0', 115200)
    #Dataport = serial.Serial('/dev/ttyACM1', 921600)
    
    # Windows
    CLIport = serial.Serial('COM3', 115200)
    Dataport = serial.Serial('COM4', 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i)
        time.sleep(0.01)
        
    return CLIport, Dataport

# ------------------------------------------------------------------

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 3
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1;
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;
                
            digOutSampleRate = int(splitWords[11]);
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1]);
            chirpEndIdx = int(splitWords[2]);
            numLoops = int(splitWords[3]);
            numFrames = int(splitWords[4]);
            framePeriodicity = int(splitWords[5]);

            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
    return configParameters
   
# ------------------------------------------------------------------

def parseCompressedSphericalPointCloudTLV(tlvData, tlvLength):
    pUnitStruct = '5f' # Units for the 5 results to decompress them
    pointStruct = '2bh2H' # Elevation, Azimuth, Doppler, Range, SNR
    pUnitSize = struct.calcsize(pUnitStruct)
    pointSize = struct.calcsize(pointStruct)
    numPoints = int((tlvLength-pUnitSize)/pointSize)
    pointCloud = np.empty((numPoints,5))
    # Parse the decompression factors
    try:
        pUnit = struct.unpack(pUnitStruct, tlvData[:pUnitSize])
    except:
            print('Error: Point Cloud TLV Parser Failed')
            return 0, pointCloud
    # Update data pointer
    tlvData = tlvData[pUnitSize:]

    # Parse each point
    
    for i in range(numPoints):
        try:
            elevation, azimuth, doppler, rng, snr = struct.unpack(pointStruct, tlvData[:pointSize])
        except:
            numPoints = i
            print('Error: Point Cloud TLV Parser Failed')
            break
        
        tlvData = tlvData[pointSize:]
        if (azimuth >= 128):
            print ('Az greater than 127')
            azimuth -= 256
        if (elevation >= 128):
            print ('Elev greater than 127')
            elevation -= 256
        if (doppler >= 32768):
            print ('Doppler greater than 32768')
            doppler -= 65536
        # Decompress values
        pointCloud[i,0] = rng * pUnit[3]          # Range
        pointCloud[i,1] = azimuth * pUnit[1]      # Azimuth
        pointCloud[i,2] = elevation * pUnit[0]    # Elevation
        pointCloud[i,3] = doppler * pUnit[2]      # Doppler
        pointCloud[i,4] = snr * pUnit[4]          # SNR
    
    #Convert spherical to cartesian:
    sphericalPointCloud = pointCloud.copy()
    # Compute X
    # Range * sin (azimuth) * cos (elevation)
    pointCloud[:,0] = sphericalPointCloud[:,0] * np.sin(sphericalPointCloud[:,1]) * np.cos(sphericalPointCloud[:,2]) 
    # Compute Y
    # Range * cos (azimuth) * cos (elevation)
    pointCloud[:,1] = sphericalPointCloud[:,0] * np.cos(sphericalPointCloud[:,1]) * np.cos(sphericalPointCloud[:,2]) 
    # Compute Z
    # Range * sin (elevation)
    pointCloud[:,2] = sphericalPointCloud[:,0] * np.sin(sphericalPointCloud[:,2])
    return numPoints, pointCloud

#-------------------------------------------------------------------
# Funtion to read and parse the incoming data


# Decode TLV Header
def tlvHeaderDecode(data):
    tlvType, tlvLength = struct.unpack('2I', data)
    return tlvType, tlvLength


def readAndParseData14xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength
    
    # Constants
    OBJ_STRUCT_SIZE_BYTES = 12;
    BYTE_VEC_ACC_MAX_SIZE = 2**15;
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
    MMWDEMO_UART_MSG_RANGE_PROFILE   = 2;
    
    
    
    ####################################TLV types:###############################
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1;
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE = 2;
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE = 3;
    MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP = 4;
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP = 5;
    MMWDEMO_OUTPUT_MSG_STATS = 6;
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO = 7;
    MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP = 8;
    MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS = 9;
    
    MMWDEMO_OUTPUT_MSG_SPHERICAL_POINTS = 1000;
    MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST = 1010;
    MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX = 1011;
    MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT = 1012
    MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS = 1020;
    MMWDEMO_OUTPUT_MSG_PRESCENCE_INDICATION = 1021;
    MMWDEMO_OUTPUT_MSG_OCCUPANCY_STATE_MACHINE = 1030;
    
    MMWDEMO_OUTPUT_MSG_VITALSIGNS = 1040;
    
    
    
    
    
    maxBufferSize = 2**15;
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    
    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}
    
    readBuffer = Dataport.read(Dataport.in_waiting)
    print(type(readBuffer))
    msg = bytearray(readBuffer)
    print(type(msg))
    res = client.publish(topic="data/liveradar", payload=str(msg), qos=0)
    status = res[0]
    if status == 0:
        msg_str = readBuffer[:10]
        print(f"LIVE RADAR: Send `{msg_str}` to topic `data/liveradar`\n")
    else:
        print(f"LIVE RADAR: Failed to send radar message to topic data/liveradar")
    #print(readBuffer)
    print('----------------------------------------------------------------------------------------------')
    '''
    with open('data_serial_log.txt', 'a') as file:
        file.write(str(readBuffer)+'\n'+'\n'+'\n')
    '''
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    print('byteCount is:  '+str(byteCount))
    
    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
        
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)
               
        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]
            
            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12:12+4],word)
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    #print(f"magicOK = {magicOK}")

    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2**8, 2**16, 2**24]
        
        # Initialize the pointer index
        idX = 0
        
        # Read the header
        magicNumber = byteBuffer[idX:idX+8]
        print('magicNumber: '+str(magicNumber))
        idX += 8
        version = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        print('version '+str(version))
        idX += 4
        totalPacketLen = np.matmul(byteBuffer[idX:idX+4],word)
        print('totalPacketLen '+str(totalPacketLen))
        idX += 4
        platform = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        print('platform '+str(platform))
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        print('frameNumber '+str(frameNumber))
        idX += 4
        timeCpuCycles = np.matmul(byteBuffer[idX:idX+4],word)
        print('timeCpuCycles '+str(timeCpuCycles))
        idX += 4
        numDetectedObj = np.matmul(byteBuffer[idX:idX+4],word)
        print('numDetectedObj '+str(numDetectedObj))
        idX += 4
        numTLVs = np.matmul(byteBuffer[idX:idX+4],word)
        print('numTLVs '+str(numTLVs))
        idX += 4
        subFrameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        print('subFrameNumber '+str(subFrameNumber))
        idX += 4
        #print(f"magicNumber = {magicNumber} \t version = {version} \t totalPacketLen = {totalPacketLen} \t platform = {platform} \t frameNumber = {frameNumber} ")
        #print(f"timeCpuCycles = {timeCpuCycles} \t\t numDetectedObj = {numDetectedObj} \t numTLVs = {numTLVs} \t\t idX = {idX}")

        # UNCOMMENT IN CASE OF SDK 2
        #subFrameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        #print(numTLVs)
        
        # Read the TLV messages
        for tlvIdx in range(numTLVs):
            
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # Check the header of the TLV message
            #print(f"byteBuffer[idX:idX+4] = {byteBuffer[idX:idX+4]}, word = {word}")
            tlv_type, tlv_length= tlvHeaderDecode(byteBuffer[idX:idX+8])
            
            #tlv_type = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            print('tlv_type is: '+str(tlv_type))
            #tlv_length = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            print('tlv_length is: '+str(tlv_length))
            #print(f"tlv_type = {tlv_type} \t MMWDEMO_UART_MSG_DETECTED_POINTS = {MMWDEMO_UART_MSG_DETECTED_POINTS}")
            # Read the data depending on the TLV message
            
            
            ##########################---Point Cloud TLV---############################
            if tlv_type == MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS:
                
                
                tlvData = byteBuffer[idX:idX+tlv_length]
                dataOK = 1
                idX += tlv_length
                pUnitStruct = '5f' # Units for the 5 results to decompress them
                pointStruct = '2bh2H' # Elevation, Azimuth, Doppler, Range, SNR
                pUnitSize = struct.calcsize(pUnitStruct)
                pointSize = struct.calcsize(pointStruct)
                numPoints = int((tlv_length-pUnitSize)/pointSize)
                pointCloud = np.empty((numPoints,5))
                # Parse the decompression factors
                try:
                    pUnit = struct.unpack(pUnitStruct, tlvData[:pUnitSize])
                except:
                        print('Error: Point Cloud TLV Parser Failed')
                        return 0, pointCloud
                # Update data pointer
                tlvData = tlvData[pUnitSize:]

                # Parse each point
    
                for i in range(numPoints):
                    try:
                        elevation, azimuth, doppler, rng, snr = struct.unpack(pointStruct, tlvData[:pointSize])
                    except:
                        numPoints = i
                        print('Error: Point Cloud TLV Parser Failed')
                        break
        
                    tlvData = tlvData[pointSize:]
                    if (azimuth >= 128):
                        print ('Az greater than 127')
                        azimuth -= 256
                    if (elevation >= 128):
                        print ('Elev greater than 127')
                        elevation -= 256
                    if (doppler >= 32768):
                        print ('Doppler greater than 32768')
                        doppler -= 65536
                    # Decompress values
                    pointCloud[i,0] = rng * pUnit[3]          # Range
                    pointCloud[i,1] = azimuth * pUnit[1]      # Azimuth
                    pointCloud[i,2] = elevation * pUnit[0]    # Elevation
                    pointCloud[i,3] = doppler * pUnit[2]      # Doppler
                    pointCloud[i,4] = snr * pUnit[4]          # SNR
    
                #Convert spherical to cartesian:
                sphericalPointCloud = pointCloud.copy()
                # Compute X
                # Range * sin (azimuth) * cos (elevation)
                pointCloud[:,0] = sphericalPointCloud[:,0] * np.sin(sphericalPointCloud[:,1]) * np.cos(sphericalPointCloud[:,2]) 
                # Compute Y
                # Range * cos (azimuth) * cos (elevation)
                pointCloud[:,1] = sphericalPointCloud[:,0] * np.cos(sphericalPointCloud[:,1]) * np.cos(sphericalPointCloud[:,2]) 
                # Compute Z
                # Range * sin (elevation)
                pointCloud[:,2] = sphericalPointCloud[:,0] * np.sin(sphericalPointCloud[:,2])
                detObj = {"TLV_type":tlv_type,"frame":frameNumber, "x": pointCloud[:,0], "y": pointCloud[:,1], "z": pointCloud[:,2]}
                detObj_log = json.dumps({"TLV_type":int(tlv_type),"frame":int(frameNumber), "x": pointCloud[:,0].tolist(), "y": pointCloud[:,1].tolist(), "z": pointCloud[:,2].tolist()})
                
                with open('tlv_data_log.json', 'a') as file:
                    file.write(str(detObj_log)+',\n')
                
            ##########################---Target List TLV---############################
            elif tlv_type == MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST:            

                targetStruct = 'I27f' 
                targetSize = struct.calcsize(targetStruct)
                numDetectedTargets = int(tlv_length/targetSize)
                targets = np.empty((numDetectedTargets,16))
                tlvData = byteBuffer[idX:idX+tlv_length]
                idX += tlv_length
                for i in range(numDetectedTargets):
                    try:
                        targetData = struct.unpack(targetStruct,tlvData[:targetSize])
                    except:
                        print('ERROR: Target TLV parsing failed')
                        targetData = struct.unpack(targetStruct,tlvData[:targetSize])

                    targets[i,0] = targetData[0] # Target ID
                    targets[i,1] = targetData[1] # X Position
                    targets[i,2] = targetData[2] # Y Position
                    targets[i,3] = targetData[3] # Z Position
                    targets[i,4] = targetData[4] # X Velocity
                    targets[i,5] = targetData[5] # Y Velocity
                    targets[i,6] = targetData[6] # Z Velocity
                    targets[i,7] = targetData[7] # X Acceleration
                    targets[i,8] = targetData[8] # Y Acceleration
                    targets[i,9] = targetData[9] # Z Acceleration
                    targets[i,10] = targetData[26] # G
                    targets[i,11] = targetData[27] # Confidence Level
                    tlvData = tlvData[targetSize:]
                    
               
                
                # Store the data in the detObj dictionary
                detObj = {"TLV_type":tlv_type,"frame":frameNumber,"tid": targets[:,0], "x": targets[:,1], "y": targets[:,2], "z": targets[:,3]}
                detObj_log = json.dumps({"TLV_type":int(tlv_type), "frame":int(frameNumber),"tid": targets[:,0].tolist(), "x": targets[:,1].tolist(), "y": targets[:,2].tolist(), "z": targets[:,3].tolist()})
                if write_radar:
                    with open('data/tlv_data_log.json', 'a') as file:
                        file.write(str(detObj_log)+',\n')
                    
                    with open('data/targets_data_log.json', 'a') as file:
                        file.write(str(detObj_log)+',\n')
                '''
                with open("sample_file.json", "a") as file:
                    json.dump(detObj_log, file)
                #detObj = {"tid": tid, "x": posX, "y": posY, "z": posZ}
                print(detObj)
                '''
                dataOK = 0
        
            ##########################---Target List TLV---############################
            elif tlv_type == MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX:
                            
                word = [1, 2**8, 2**16, 2**24]
                tid = byteBuffer[idX]
                idX += 1
                idX += tlv_length-1
            ##########################---Target List TLV---############################
            elif tlv_type == MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT:
                            
                word = [1, 2**8, 2**16, 2**24]
                tid = byteBuffer[idX]
                idX += 1
                maxZ = np.matmul(byteBuffer[idX:idX+4],word)
                idX += 4
                minZ = np.matmul(byteBuffer[idX:idX+4],word)
                idX += 4
                
                idX += tlv_length-9
  
        # Remove already processed data
        if idX > 0 and byteBufferLength > idX:
            shiftSize = totalPacketLen
               
            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
            byteBufferLength = byteBufferLength - shiftSize
            
            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                

    return dataOK, frameNumber, detObj

# ------------------------------------------------------------------

# Funtion to update the data and display in the plot
def update():
     
    dataOk = 0
    global detObj
    x = []
    y = []
      
    # Read and parse the received data
    dataOk, frameNumber, detObj = readAndParseData14xx(Dataport, configParameters)
    print(f"dataOK = {dataOk}")
    if dataOk and len(detObj["x"]) > 0:
        print(detObj)
        #update_demo(detObj)
    
    return dataOk

#--------------------------    DEMO    ----------------------------------------
# modify below to change number of frames to run in animation
marker_size = 5



'''
def update_demo(det_obj):
    # Create the figure and 3D subplot
    #fig = plt.figure(figsize=(9, 6), dpi=100)
    #ax = fig.add_subplot(111, projection='3d')

    # For axis limits 
    max_x= 14.5406686
    max_y= 16.3837508
    max_z= 15.0282616
    min_x= -14.2102056
    min_y= 0.0
    min_z= -15.5174625
    frame_data = det_obj
    #ax.view_init(elev=i, azim=i)
    x = frame_data['x']
    y = frame_data['y']
    z = frame_data['z']
    tid = frame_data.get('tid', None)
    ax.clear()
    if frame_data['TLV_type'] == 1020:
        point_colour = (0,0,0.5) 
    elif frame_data['TLV_type'] == 1010:
        point_colour = 'red'
    else:
        point_colour = 'green'
    for j in range(len(x)):
        #sc = ax.scatter(x[j], y[j], z[j], c=color, cmap='plasma', s=marker_size, alpha=0.5)
        sc = ax.scatter(x[j], y[j], z[j], color=point_colour, s=marker_size, alpha=1)
        
    # ax.set_xlim3d(-15, 15)
    # ax.set_ylim3d(0, 17)
    # ax.set_zlim3d(-16, 16)
    ax.set_xlim3d(round(min_x-1), round(max_x+1))
    ax.set_ylim3d(round(min_y-1), round(max_y+1))
    ax.set_zlim3d(round(min_z-1), round(max_z+1))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.xaxis.set_pane_color((0.992, 0.965, 0.890, 1.0))  
    ax.yaxis.set_pane_color((0.992, 0.965, 0.890, 1.0))  
    ax.zaxis.set_pane_color((0.992, 0.965, 0.890, 1.0))  

    ax.set_title("Radar Visualization (3D Scatter Plot)")
    text_str = (
    f'Points in frame: {len(x)} \n'
    f'Frame: {frame_data["frame"]} \n'
    f'TLV Type: {frame_data["TLV_type"]} \n'
    f'tid: {tid if tid else "None"}'
    )
    ax.text(0.98, 0.02, 0.02, text_str, transform=ax.transAxes, fontsize=10, ha='right', va='bottom')
    fig.canvas.draw()
    fig.canvas.manager.show() 
    fig.canvas.flush_events() 
'''
# -------------------------    MAIN   -----------------------------------------  

# Configurate the serial port

CLIport, Dataport = serialConfig(configFileName)

# Get the configuration parameters from the configuration file
configParameters = parseConfigFile(configFileName)


# START QtAPPfor the plot

CLIport.write(('sensorStop\n').encode())
'''
CLIport.close()
Dataport.close()
'''

#plt.axis([0, 10, 0, 1])
#plt.show()
    
   
# Main loop 
if __name__ == "__main__":
    detObj = {}  
    frameData = {}    
    currentIndex = 0
    client = connect_mqtt("LIVE RADAR") 
    while True:
        try:
            # Update the data and check if the data is okay
            client.loop_start()
            dataOk = update()
            if dataOk:
                # Store the current frame into frameData
                frameData[currentIndex] = detObj
                currentIndex += 1
            
            time.sleep(0.033) # Sampling frequency of 30 Hz
            client.loop_stop()

        # Stop the program and close everything if Ctrl + c is pressed
        except KeyboardInterrupt:
            CLIport.write(('sensorStop\n').encode())
            CLIport.close()
            Dataport.close()
            #win.close()
            break
        except Exception as e:
            print(e)
            CLIport.write(('sensorStop\n').encode())
            CLIport.close()
            Dataport.close()
            #win.close()
            break