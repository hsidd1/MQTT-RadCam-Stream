import numpy as np
import struct
import json
import datetime
import yaml

with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)
    
radar_data_file = config["Files"]["radar_data_file"]
with open(radar_data_file) as json_file:
    data = json.load(json_file)

# sensorhost format
def rd_process():
    radar_points = []
    for item in data["frames"]:
        num_ob = item["sensorMessage"]["metadata"]["numOfDetectedObjects"]
        detected_points = item["sensorMessage"]["object"]["detectedPoints"]
        timestamp = item["timestamp"]  

        for j in range(num_ob):
            s = dict()
            s["sensorId"] = detected_points[j]["sensorId"]
            s["x"] = detected_points[j]["x"] * 10  # converting to mm
            s["y"] = detected_points[j]["y"] * 10
            s["z"] = detected_points[j]["z"] * 10
            s["timestamp"] = timestamp

            radar_points.append(s)
    return radar_points

data = rd_process()

def tlvHeaderDecode(data):
    tlvType, tlvLength = struct.unpack('2I', data)
    return tlvType, tlvLength

def readbuffer_process(readBuffer: bytearray, sensor_id) -> json:
    byteBuffer = np.zeros(2**15,dtype = 'uint8')
    maxBufferSize = 2**15
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS = 1020
    MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST = 1010
    byteBufferLength = 0
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    print('byteCount is:  '+str(byteCount))
    detObj_log = {}
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
                detObj = {"Sensor_id": sensor_id, "TLV_type":tlv_type,"frame":frameNumber, "x": pointCloud[:,0], "y": pointCloud[:,1], "z": pointCloud[:,2]}
                
                detObj_log = json.dumps({"time":datetime.now().strftime("%H:%M:%S.%f"),"Sensor_id": int(sensor_id), "TLV_type":int(tlv_type),"frame":int(frameNumber), "x": pointCloud[:,0].tolist(), "y": pointCloud[:,1].tolist(), "z": pointCloud[:,2].tolist()})
                
                # with open('tlv_data_log.json', 'a') as file:
                #     file.write(str(detObj_log)+',\n')
                
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
                detObj = {"Sensor_id": sensor_id, "TLV_type":tlv_type,"frame":frameNumber,"tid": targets[:,0], "x": targets[:,1], "y": targets[:,2], "z": targets[:,3]}
                detObj_log = json.dumps({"time":datetime.now().strftime("%H:%M:%S.%f"), "Sensor_id": int(sensor_id), "TLV_type":int(tlv_type), "frame":int(frameNumber),"tid": targets[:,0].tolist(), "x": targets[:,1].tolist(), "y": targets[:,2].tolist(), "z": targets[:,3].tolist()})
    return detObj_log 

def process_radar(radar_payload: list[dict]) -> None:
    # TODO: generate text and data points from radar data to plot over image
    raise NotImplementedError