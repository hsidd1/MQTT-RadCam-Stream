import serial
import time
import numpy as np
import struct
import json
from processModule.serverConnect import connect_mqtt
import yaml
import traceback
from datetime import datetime

"""
Live radar client program for publishing live radar data to MQTT broker.
Note: This program runs as a subprocess of live_receiver.py.
"""

with open("config.yaml", "r") as f:
    yml_config = yaml.safe_load(f)

# MQTT identifiers
TOPIC = yml_config["LiveData"]["radar"]["topic"]
CLIENT_ID = yml_config["LiveData"]["radar"]["client_id"]
# Change the configuration file name
# configFileName = 'xwr68xxconfig.cfg'
configFileName = "ODS_6m_default.cfg"
# configFileName = 'ISK_6m_default.cfg'
# configFileName = 'new_cfg.cfg'
# configFileName = 'demo.cfg'
write_radar = False
CLIport1 = {}
Dataport1 = {}
CLIport2 = {}
Dataport2 = {}
byteBuffer = np.zeros(2**15, dtype="uint8")
byteBuffer2 = np.zeros(2**15, dtype="uint8")
byteBufferLength = 0
byteBufferLength2 = 0


# -------------------------------------------------------------------
# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig1(configFileName):
    global CLIport1
    global Dataport1
    global CLIport2
    global Dataport2
    global yml_config
    # Open the serial ports for the configuration and the data ports

    # Raspberry pi
    # CLIport = serial.Serial('/dev/ttyACM0', 115200)
    # Dataport = serial.Serial('/dev/ttyACM1', 921600)

    # Windows
    str_cliport = yml_config["LiveData"]["radar"]["CLIport"]
    str_dataport = yml_config["LiveData"]["radar"]["DataPort"]
    str_cliport2 = yml_config["LiveData"]["radar"]["CLIport2"]
    str_dataport2 = yml_config["LiveData"]["radar"]["DataPort2"]
    CLIport1 = serial.Serial(str_cliport, 115200)
    Dataport1 = serial.Serial(str_dataport, 921600)
    # on reconnect, wait for the port to come back
    time.sleep(1)
    # flush buffers
    CLIport1.flushInput()
    CLIport1.flushOutput()
    Dataport1.flushInput()
    Dataport1.flushOutput()
    # Read the configuration file and send it to the board
    config = [line.rstrip("\r\n") for line in open(configFileName)]
    for i in config:
        CLIport1.write((i + "\n").encode())
        # print(i)
        time.sleep(0.01)

    CLIport2 = serial.Serial(str_cliport2, 115200)
    Dataport2 = serial.Serial(str_dataport2, 921600)
    # on reconnect, wait for the port to come back
    time.sleep(1)
    # flush buffers
    CLIport2.flushInput()
    CLIport2.flushOutput()
    Dataport2.flushInput()
    Dataport2.flushOutput()

    for i in config:
        CLIport2.write((i + "\n").encode())
        # print(i)
        time.sleep(0.01)

    return CLIport1, Dataport1, CLIport2, Dataport2


"""
def serialConfig2(configFileName):
    
    global CLIport2
    global Dataport2
    global yml_config
    # Open the serial ports for the configuration and the data ports
    
    # Raspberry pi
    #CLIport = serial.Serial('/dev/ttyACM0', 115200)
    #Dataport = serial.Serial('/dev/ttyACM1', 921600)
    
    # Windows
    str_cliport2 = yml_config["LiveData"]["radar"]["CLIport2"]
    str_dataport2 = yml_config["LiveData"]["radar"]["DataPort2"]
    CLIport2 = serial.Serial(str_cliport2, 115200)
    Dataport2 = serial.Serial(str_dataport2, 921600)
    # on reconnect, wait for the port to come back
    time.sleep(1)
    # flush buffers
    CLIport2.flushInput()
    CLIport2.flushOutput()
    Dataport2.flushInput()
    Dataport2.flushOutput()
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport2.write((i+'\n').encode())
        #print(i)
        time.sleep(0.01)
        
    return CLIport2, Dataport2
"""
# ------------------------------------------------------------------


# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = (
        {}
    )  # Initialize an empty dictionary to store the configuration parameters

    # Read the configuration file and send it to the board
    config = [line.rstrip("\r\n") for line in open(configFileName)]
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
            numAdcSamplesRoundTo2 = 1

            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2

            digOutSampleRate = int(splitWords[11])

        # Get the information about the frame configuration
        elif "frameCfg" in splitWords[0]:
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            numFrames = int(splitWords[4])
            framePeriodicity = int(splitWords[5])

    # Combine the read data to obtain the configuration parameters
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
        2 * freqSlopeConst * 1e12 * numAdcSamples
    )
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
        2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"]
    )
    configParameters["dopplerResolutionMps"] = 3e8 / (
        2
        * startFreq
        * 1e9
        * (idleTime + rampEndTime)
        * 1e-6
        * configParameters["numDopplerBins"]
        * numTxAnt
    )
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (
        2 * freqSlopeConst * 1e3
    )
    configParameters["maxVelocity"] = 3e8 / (
        4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt
    )

    return configParameters


# ------------------------------------------------------------------


def parseCompressedSphericalPointCloudTLV(tlvData, tlvLength):
    pUnitStruct = "5f"  # Units for the 5 results to decompress them
    pointStruct = "2bh2H"  # Elevation, Azimuth, Doppler, Range, SNR
    pUnitSize = struct.calcsize(pUnitStruct)
    pointSize = struct.calcsize(pointStruct)
    numPoints = int((tlvLength - pUnitSize) / pointSize)
    pointCloud = np.empty((numPoints, 5))
    # Parse the decompression factors
    try:
        pUnit = struct.unpack(pUnitStruct, tlvData[:pUnitSize])
    except:
        print("Error: Point Cloud TLV Parser Failed")
        return 0, pointCloud
    # Update data pointer
    tlvData = tlvData[pUnitSize:]

    # Parse each point

    for i in range(numPoints):
        try:
            elevation, azimuth, doppler, rng, snr = struct.unpack(
                pointStruct, tlvData[:pointSize]
            )
        except:
            numPoints = i
            print("Error: Point Cloud TLV Parser Failed")
            break

        tlvData = tlvData[pointSize:]
        if azimuth >= 128:
            print("Az greater than 127")
            azimuth -= 256
        if elevation >= 128:
            print("Elev greater than 127")
            elevation -= 256
        if doppler >= 32768:
            print("Doppler greater than 32768")
            doppler -= 65536
        # Decompress values
        pointCloud[i, 0] = rng * pUnit[3]  # Range
        pointCloud[i, 1] = azimuth * pUnit[1]  # Azimuth
        pointCloud[i, 2] = elevation * pUnit[0]  # Elevation
        pointCloud[i, 3] = doppler * pUnit[2]  # Doppler
        pointCloud[i, 4] = snr * pUnit[4]  # SNR

    # Convert spherical to cartesian:
    sphericalPointCloud = pointCloud.copy()
    # Compute X
    # Range * sin (azimuth) * cos (elevation)
    pointCloud[:, 0] = (
        sphericalPointCloud[:, 0]
        * np.sin(sphericalPointCloud[:, 1])
        * np.cos(sphericalPointCloud[:, 2])
    )
    # Compute Y
    # Range * cos (azimuth) * cos (elevation)
    pointCloud[:, 1] = (
        sphericalPointCloud[:, 0]
        * np.cos(sphericalPointCloud[:, 1])
        * np.cos(sphericalPointCloud[:, 2])
    )
    # Compute Z
    # Range * sin (elevation)
    pointCloud[:, 2] = sphericalPointCloud[:, 0] * np.sin(sphericalPointCloud[:, 2])
    return numPoints, pointCloud


# -------------------------------------------------------------------
# Funtion to read and parse the incoming data


# Decode TLV Header
def tlvHeaderDecode(data):
    tlvType, tlvLength = struct.unpack("2I", data)
    return tlvType, tlvLength


def readAndParseData(Dataport1, configParameters, client, sensor_id):
    global byteBuffer, byteBufferLength

    ####################################TLV types:###############################

    MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST = 1010
    MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX = 1011
    MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT = 1012
    MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS = 1020

    maxBufferSize = 2**15
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

    # Initialize variables
    magicOK = 0  # Checks if magic number has been read
    dataOK = 0  # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}
    detObj_log = None
    readBuffer = Dataport1.read(Dataport1.in_waiting)
    print(
        "----------------------------------------------------------------------------------------------"
    )
    """
    with open('data_serial_log.txt', 'a') as file:
        file.write(str(readBuffer)+'\n'+'\n'+'\n')
    """
    byteVec = np.frombuffer(readBuffer, dtype="uint8")
    byteCount = len(byteVec)
    # print('byteCount is:  '+str(byteCount))

    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength : byteBufferLength + byteCount] = byteVec[
            :byteCount
        ]
        byteBufferLength = byteBufferLength + byteCount

    # Check that the buffer has some data
    if byteBufferLength > 16:
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc : loc + 8]
            if np.all(check == magicWord):
                startIdx.append(loc)

        # Check that startIdx is not empty
        if startIdx:
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[: byteBufferLength - startIdx[0]] = byteBuffer[
                    startIdx[0] : byteBufferLength
                ]
                byteBuffer[byteBufferLength - startIdx[0] :] = np.zeros(
                    len(byteBuffer[byteBufferLength - startIdx[0] :]), dtype="uint8"
                )
                byteBufferLength = byteBufferLength - startIdx[0]

            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0

            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12 : 12 + 4], word)
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    # print(f"magicOK = {magicOK}")

    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2**8, 2**16, 2**24]

        # Initialize the pointer index
        idX = 0

        # Read the header
        magicNumber = byteBuffer[idX : idX + 8]
        # print('magicNumber: '+str(magicNumber))
        idX += 8
        version = format(np.matmul(byteBuffer[idX : idX + 4], word), "x")
        # print('version '+str(version))
        idX += 4
        totalPacketLen = np.matmul(byteBuffer[idX : idX + 4], word)
        # print('totalPacketLen '+str(totalPacketLen))
        idX += 4
        platform = format(np.matmul(byteBuffer[idX : idX + 4], word), "x")
        # print('platform '+str(platform))
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX : idX + 4], word)
        # print('frameNumber '+str(frameNumber))
        idX += 4
        timeCpuCycles = np.matmul(byteBuffer[idX : idX + 4], word)
        # print('timeCpuCycles '+str(timeCpuCycles))
        idX += 4
        numDetectedObj = np.matmul(byteBuffer[idX : idX + 4], word)
        # print('numDetectedObj '+str(numDetectedObj))
        idX += 4
        numTLVs = np.matmul(byteBuffer[idX : idX + 4], word)
        # print('numTLVs '+str(numTLVs))
        idX += 4
        subFrameNumber = np.matmul(byteBuffer[idX : idX + 4], word)
        # print('subFrameNumber '+str(subFrameNumber))
        idX += 4
        # print(f"magicNumber = {magicNumber} \t version = {version} \t totalPacketLen = {totalPacketLen} \t platform = {platform} \t frameNumber = {frameNumber} ")
        # print(f"timeCpuCycles = {timeCpuCycles} \t\t numDetectedObj = {numDetectedObj} \t numTLVs = {numTLVs} \t\t idX = {idX}")

        # UNCOMMENT IN CASE OF SDK 2
        # subFrameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        # print(numTLVs)

        # Read the TLV messages
        for tlvIdx in range(numTLVs):
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # Check the header of the TLV message
            # print(f"byteBuffer[idX:idX+4] = {byteBuffer[idX:idX+4]}, word = {word}")
            tlv_type, tlv_length = tlvHeaderDecode(byteBuffer[idX : idX + 8])

            # tlv_type = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            # print('tlv_type is: '+str(tlv_type))
            # tlv_length = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            # print('tlv_length is: '+str(tlv_length))
            # print(f"tlv_type = {tlv_type} \t MMWDEMO_UART_MSG_DETECTED_POINTS = {MMWDEMO_UART_MSG_DETECTED_POINTS}")
            # Read the data depending on the TLV message

            ##########################---Point Cloud TLV---############################
            if tlv_type == MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS:  # 1020
                tlvData = byteBuffer[idX : idX + tlv_length]
                dataOK = 1
                idX += tlv_length
                pUnitStruct = "5f"  # Units for the 5 results to decompress them
                pointStruct = "2bh2H"  # Elevation, Azimuth, Doppler, Range, SNR
                pUnitSize = struct.calcsize(pUnitStruct)
                pointSize = struct.calcsize(pointStruct)
                numPoints = int((tlv_length - pUnitSize) / pointSize)
                pointCloud = np.empty((numPoints, 5))
                # Parse the decompression factors
                try:
                    pUnit = struct.unpack(pUnitStruct, tlvData[:pUnitSize])
                except:
                    print("Error: Point Cloud TLV Parser Failed")
                    return 0, pointCloud
                # Update data pointer
                tlvData = tlvData[pUnitSize:]

                # Parse each point

                for i in range(numPoints):
                    try:
                        elevation, azimuth, doppler, rng, snr = struct.unpack(
                            pointStruct, tlvData[:pointSize]
                        )
                    except:
                        numPoints = i
                        print("Error: Point Cloud TLV Parser Failed")
                        break

                    tlvData = tlvData[pointSize:]
                    if azimuth >= 128:
                        print("Az greater than 127")
                        azimuth -= 256
                    if elevation >= 128:
                        print("Elev greater than 127")
                        elevation -= 256
                    if doppler >= 32768:
                        print("Doppler greater than 32768")
                        doppler -= 65536
                    # Decompress values
                    pointCloud[i, 0] = rng * pUnit[3]  # Range
                    pointCloud[i, 1] = azimuth * pUnit[1]  # Azimuth
                    pointCloud[i, 2] = elevation * pUnit[0]  # Elevation
                    pointCloud[i, 3] = doppler * pUnit[2]  # Doppler
                    pointCloud[i, 4] = snr * pUnit[4]  # SNR

                # Convert spherical to cartesian:
                sphericalPointCloud = pointCloud.copy()
                # Compute X
                # Range * sin (azimuth) * cos (elevation)
                pointCloud[:, 0] = (
                    sphericalPointCloud[:, 0]
                    * np.sin(sphericalPointCloud[:, 1])
                    * np.cos(sphericalPointCloud[:, 2])
                )
                # Compute Y
                # Range * cos (azimuth) * cos (elevation)
                pointCloud[:, 1] = (
                    sphericalPointCloud[:, 0]
                    * np.cos(sphericalPointCloud[:, 1])
                    * np.cos(sphericalPointCloud[:, 2])
                )
                # Compute Z
                # Range * sin (elevation)
                pointCloud[:, 2] = sphericalPointCloud[:, 0] * np.sin(
                    sphericalPointCloud[:, 2]
                )
                detObj = {
                    "time": datetime.now().strftime("%H:%M:%S.%f"),
                    "Sensor_id": int(sensor_id),
                    "TLV_type": tlv_type,
                    "frame": frameNumber,
                    "x": pointCloud[:, 0],
                    "y": pointCloud[:, 1],
                    "z": pointCloud[:, 2],
                }
                detObj_log = json.dumps(
                    {
                        "time": datetime.now().strftime("%H:%M:%S.%f"),
                        "Sensor_id": int(sensor_id),
                        "TLV_type": int(tlv_type),
                        "frame": int(frameNumber),
                        "x": pointCloud[:, 0].tolist(),
                        "y": pointCloud[:, 1].tolist(),
                        "z": pointCloud[:, 2].tolist(),
                    }
                )
                if write_radar:
                    with open("data/tlv_data_log.json", "a") as file:
                        file.write(str(detObj_log) + ",\n")

            ##########################---Target List TLV---############################
            elif tlv_type == MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST:  # 1010
                targetStruct = "I27f"
                targetSize = struct.calcsize(targetStruct)
                numDetectedTargets = int(tlv_length / targetSize)
                targets = np.empty((numDetectedTargets, 16))
                tlvData = byteBuffer[idX : idX + tlv_length]
                idX += tlv_length
                for i in range(numDetectedTargets):
                    try:
                        targetData = struct.unpack(targetStruct, tlvData[:targetSize])
                    except:
                        print("ERROR: Target TLV parsing failed")
                        targetData = struct.unpack(targetStruct, tlvData[:targetSize])

                    targets[i, 0] = targetData[0]  # Target ID
                    targets[i, 1] = targetData[1]  # X Position
                    targets[i, 2] = targetData[2]  # Y Position
                    targets[i, 3] = targetData[3]  # Z Position
                    targets[i, 4] = targetData[4]  # X Velocity
                    targets[i, 5] = targetData[5]  # Y Velocity
                    targets[i, 6] = targetData[6]  # Z Velocity
                    targets[i, 7] = targetData[7]  # X Acceleration
                    targets[i, 8] = targetData[8]  # Y Acceleration
                    targets[i, 9] = targetData[9]  # Z Acceleration
                    targets[i, 10] = targetData[26]  # G
                    targets[i, 11] = targetData[27]  # Confidence Level
                    tlvData = tlvData[targetSize:]

                # Store the data in the detObj dictionary
                # detObj = {"Sensor_id": sensor_id, "TLV_type":tlv_type,"frame":frameNumber, "x": pointCloud[:,0], "y": pointCloud[:,1], "z": pointCloud[:,2]}
                try:
                    detObj_log = json.dumps(
                        {
                            "time": datetime.now().strftime("%H:%M:%S.%f"),
                            "Sensor_id": int(sensor_id),
                            "TLV_type": int(tlv_type),
                            "frame": int(frameNumber),
                            "x": pointCloud[:, 0].tolist(),
                            "y": pointCloud[:, 1].tolist(),
                            "z": pointCloud[:, 2].tolist(),
                        }
                    )
                except UnboundLocalError:
                    detObj_log = None
                if write_radar:
                    with open("data/tlv_data_log.json", "a") as file:
                        file.write(str(detObj_log) + ",\n")

                    with open("data/targets_data_log.json", "a") as file:
                        file.write(str(detObj_log) + ",\n")
                """
                with open("sample_file.json", "a") as file:
                    json.dump(detObj_log, file)
                #detObj = {"tid": tid, "x": posX, "y": posY, "z": posZ}
                print(detObj)
                """
                dataOK = 0

            ##########################---Target List TLV---############################
            elif tlv_type == MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX:  # 1011
                word = [1, 2**8, 2**16, 2**24]
                tid = byteBuffer[idX]
                idX += 1
                idX += tlv_length - 1
            ##########################---Target List TLV---############################
            elif tlv_type == MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT:
                word = [1, 2**8, 2**16, 2**24]
                tid = byteBuffer[idX]
                idX += 1
                maxZ = np.matmul(byteBuffer[idX : idX + 4], word)
                idX += 4
                minZ = np.matmul(byteBuffer[idX : idX + 4], word)
                idX += 4

                idX += tlv_length - 9

        # Remove already processed data
        if idX > 0 and byteBufferLength > idX:
            shiftSize = totalPacketLen

            byteBuffer[: byteBufferLength - shiftSize] = byteBuffer[
                shiftSize:byteBufferLength
            ]
            byteBuffer[byteBufferLength - shiftSize :] = np.zeros(
                len(byteBuffer[byteBufferLength - shiftSize :]), dtype="uint8"
            )
            byteBufferLength = byteBufferLength - shiftSize

            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
        print(detObj_log)
        if detObj_log is not None:
            res = client.publish(topic=TOPIC, payload=(detObj_log), qos=0)
            # else:
            #     res = client.publish(topic=TOPIC, payload="Empty", qos=0)
            status = res[0]
            if status == 0:
                # msg_str = str(detObj_log)[:10] + "..."
                msg_str = "detObj_log"
                print(f"{CLIENT_ID}: Send `{msg_str}` to topic `{TOPIC}`\n\n")
            else:
                print(f"{CLIENT_ID}: Failed to send radar message to topic `{TOPIC}`\n")
    return dataOK, frameNumber, detObj


# ------------------------------------------------------------------
def readAndParseData2(Dataport2, configParameters, client, sensor_id):
    global byteBuffer2, byteBufferLength2

    ####################################TLV types:###############################

    MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST = 1010
    MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX = 1011
    MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT = 1012
    MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS = 1020

    maxBufferSize = 2**15
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

    # Initialize variables
    magicOK = 0  # Checks if magic number has been read
    dataOK = 0  # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}
    detObj_log = None
    readBuffer = Dataport2.read(Dataport2.in_waiting)
    print(
        "----------------------------------------------------------------------------------------------"
    )
    """
    with open('data_serial_log.txt', 'a') as file:
        file.write(str(readBuffer)+'\n'+'\n'+'\n')
    """
    byteVec = np.frombuffer(readBuffer, dtype="uint8")
    byteCount = len(byteVec)
    # print('byteCount is:  '+str(byteCount))

    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength2 + byteCount) < maxBufferSize:
        byteBuffer2[byteBufferLength2 : byteBufferLength2 + byteCount] = byteVec[
            :byteCount
        ]
        byteBufferLength2 = byteBufferLength2 + byteCount

    # Check that the buffer has some data
    if byteBufferLength2 > 16:
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer2 == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer2[loc : loc + 8]
            if np.all(check == magicWord):
                startIdx.append(loc)

        # Check that startIdx is not empty
        if startIdx:
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength2:
                byteBuffer2[: byteBufferLength2 - startIdx[0]] = byteBuffer2[
                    startIdx[0] : byteBufferLength2
                ]
                byteBuffer2[byteBufferLength2 - startIdx[0] :] = np.zeros(
                    len(byteBuffer2[byteBufferLength2 - startIdx[0] :]), dtype="uint8"
                )
                byteBufferLength2 = byteBufferLength2 - startIdx[0]

            # Check that there have no errors with the byte buffer length
            if byteBufferLength2 < 0:
                byteBufferLength2 = 0

            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer2[12 : 12 + 4], word)
            # Check that all the packet has been read
            if (byteBufferLength2 >= totalPacketLen) and (byteBufferLength2 != 0):
                magicOK = 1
    # print(f"magicOK = {magicOK}")

    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2**8, 2**16, 2**24]

        # Initialize the pointer index
        idX = 0

        # Read the header
        magicNumber = byteBuffer2[idX : idX + 8]
        # print('magicNumber: '+str(magicNumber))
        idX += 8
        version = format(np.matmul(byteBuffer2[idX : idX + 4], word), "x")
        # print('version '+str(version))
        idX += 4
        totalPacketLen = np.matmul(byteBuffer2[idX : idX + 4], word)
        # print('totalPacketLen '+str(totalPacketLen))
        idX += 4
        platform = format(np.matmul(byteBuffer2[idX : idX + 4], word), "x")
        # print('platform '+str(platform))
        idX += 4
        frameNumber = np.matmul(byteBuffer2[idX : idX + 4], word)
        # print('frameNumber '+str(frameNumber))
        idX += 4
        timeCpuCycles = np.matmul(byteBuffer2[idX : idX + 4], word)
        # print('timeCpuCycles '+str(timeCpuCycles))
        idX += 4
        numDetectedObj = np.matmul(byteBuffer2[idX : idX + 4], word)
        # print('numDetectedObj '+str(numDetectedObj))
        idX += 4
        numTLVs = np.matmul(byteBuffer2[idX : idX + 4], word)
        # print('numTLVs '+str(numTLVs))
        idX += 4
        subFrameNumber = np.matmul(byteBuffer2[idX : idX + 4], word)
        # print('subFrameNumber '+str(subFrameNumber))
        idX += 4
        # print(f"magicNumber = {magicNumber} \t version = {version} \t totalPacketLen = {totalPacketLen} \t platform = {platform} \t frameNumber = {frameNumber} ")
        # print(f"timeCpuCycles = {timeCpuCycles} \t\t numDetectedObj = {numDetectedObj} \t numTLVs = {numTLVs} \t\t idX = {idX}")

        # UNCOMMENT IN CASE OF SDK 2
        # subFrameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        # print(numTLVs)

        # Read the TLV messages
        for tlvIdx in range(numTLVs):
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # Check the header of the TLV message
            # print(f"byteBuffer[idX:idX+4] = {byteBuffer[idX:idX+4]}, word = {word}")
            tlv_type, tlv_length = tlvHeaderDecode(byteBuffer2[idX : idX + 8])

            # tlv_type = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            # print('tlv_type is: '+str(tlv_type))
            # tlv_length = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            # print('tlv_length is: '+str(tlv_length))
            # print(f"tlv_type = {tlv_type} \t MMWDEMO_UART_MSG_DETECTED_POINTS = {MMWDEMO_UART_MSG_DETECTED_POINTS}")
            # Read the data depending on the TLV message

            ##########################---Point Cloud TLV---############################
            if tlv_type == MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS:  # 1020
                tlvData = byteBuffer2[idX : idX + tlv_length]
                dataOK = 1
                idX += tlv_length
                pUnitStruct = "5f"  # Units for the 5 results to decompress them
                pointStruct = "2bh2H"  # Elevation, Azimuth, Doppler, Range, SNR
                pUnitSize = struct.calcsize(pUnitStruct)
                pointSize = struct.calcsize(pointStruct)
                numPoints = int((tlv_length - pUnitSize) / pointSize)
                pointCloud = np.empty((numPoints, 5))
                # Parse the decompression factors
                try:
                    pUnit = struct.unpack(pUnitStruct, tlvData[:pUnitSize])
                except:
                    print("Error: Point Cloud TLV Parser Failed")
                    return 0, pointCloud
                # Update data pointer
                tlvData = tlvData[pUnitSize:]

                # Parse each point

                for i in range(numPoints):
                    try:
                        elevation, azimuth, doppler, rng, snr = struct.unpack(
                            pointStruct, tlvData[:pointSize]
                        )
                    except:
                        numPoints = i
                        print("Error: Point Cloud TLV Parser Failed")
                        break

                    tlvData = tlvData[pointSize:]
                    if azimuth >= 128:
                        print("Az greater than 127")
                        azimuth -= 256
                    if elevation >= 128:
                        print("Elev greater than 127")
                        elevation -= 256
                    if doppler >= 32768:
                        print("Doppler greater than 32768")
                        doppler -= 65536
                    # Decompress values
                    pointCloud[i, 0] = rng * pUnit[3]  # Range
                    pointCloud[i, 1] = azimuth * pUnit[1]  # Azimuth
                    pointCloud[i, 2] = elevation * pUnit[0]  # Elevation
                    pointCloud[i, 3] = doppler * pUnit[2]  # Doppler
                    pointCloud[i, 4] = snr * pUnit[4]  # SNR

                # Convert spherical to cartesian:
                sphericalPointCloud = pointCloud.copy()
                # Compute X
                # Range * sin (azimuth) * cos (elevation)
                pointCloud[:, 0] = (
                    sphericalPointCloud[:, 0]
                    * np.sin(sphericalPointCloud[:, 1])
                    * np.cos(sphericalPointCloud[:, 2])
                )
                # Compute Y
                # Range * cos (azimuth) * cos (elevation)
                pointCloud[:, 1] = (
                    sphericalPointCloud[:, 0]
                    * np.cos(sphericalPointCloud[:, 1])
                    * np.cos(sphericalPointCloud[:, 2])
                )
                # Compute Z
                # Range * sin (elevation)
                pointCloud[:, 2] = sphericalPointCloud[:, 0] * np.sin(
                    sphericalPointCloud[:, 2]
                )
                detObj = {
                    "time": datetime.now().strftime("%H:%M:%S.%f"),
                    "Sensor_id": int(sensor_id),
                    "TLV_type": tlv_type,
                    "frame": frameNumber,
                    "x": pointCloud[:, 0],
                    "y": pointCloud[:, 1],
                    "z": pointCloud[:, 2],
                }
                detObj_log = json.dumps(
                    {
                        "time": datetime.now().strftime("%H:%M:%S.%f"),
                        "Sensor_id": int(sensor_id),
                        "TLV_type": int(tlv_type),
                        "frame": int(frameNumber),
                        "x": pointCloud[:, 0].tolist(),
                        "y": pointCloud[:, 1].tolist(),
                        "z": pointCloud[:, 2].tolist(),
                    }
                )
                if write_radar:
                    with open("data/tlv_data_log.json", "a") as file:
                        file.write(str(detObj_log) + ",\n")

            ##########################---Target List TLV---############################
            elif tlv_type == MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST:  # 1010
                targetStruct = "I27f"
                targetSize = struct.calcsize(targetStruct)
                numDetectedTargets = int(tlv_length / targetSize)
                targets = np.empty((numDetectedTargets, 16))
                tlvData = byteBuffer2[idX : idX + tlv_length]
                idX += tlv_length
                for i in range(numDetectedTargets):
                    try:
                        targetData = struct.unpack(targetStruct, tlvData[:targetSize])
                    except:
                        print("ERROR: Target TLV parsing failed")
                        targetData = struct.unpack(targetStruct, tlvData[:targetSize])

                    targets[i, 0] = targetData[0]  # Target ID
                    targets[i, 1] = targetData[1]  # X Position
                    targets[i, 2] = targetData[2]  # Y Position
                    targets[i, 3] = targetData[3]  # Z Position
                    targets[i, 4] = targetData[4]  # X Velocity
                    targets[i, 5] = targetData[5]  # Y Velocity
                    targets[i, 6] = targetData[6]  # Z Velocity
                    targets[i, 7] = targetData[7]  # X Acceleration
                    targets[i, 8] = targetData[8]  # Y Acceleration
                    targets[i, 9] = targetData[9]  # Z Acceleration
                    targets[i, 10] = targetData[26]  # G
                    targets[i, 11] = targetData[27]  # Confidence Level
                    tlvData = tlvData[targetSize:]

                # Store the data in the detObj dictionary
                # detObj = {"Sensor_id": sensor_id, "TLV_type":tlv_type,"frame":frameNumber, "x": pointCloud[:,0], "y": pointCloud[:,1], "z": pointCloud[:,2]}
                try:
                    detObj_log = json.dumps(
                        {
                            "time": datetime.now().strftime("%H:%M:%S.%f"),
                            "Sensor_id": int(sensor_id),
                            "TLV_type": int(tlv_type),
                            "frame": int(frameNumber),
                            "x": pointCloud[:, 0].tolist(),
                            "y": pointCloud[:, 1].tolist(),
                            "z": pointCloud[:, 2].tolist(),
                        }
                    )
                except UnboundLocalError:
                    detObj_log = None
                if write_radar:
                    with open("data/tlv_data_log.json", "a") as file:
                        file.write(str(detObj_log) + ",\n")

                    with open("data/targets_data_log.json", "a") as file:
                        file.write(str(detObj_log) + ",\n")
                """
                with open("sample_file.json", "a") as file:
                    json.dump(detObj_log, file)
                #detObj = {"tid": tid, "x": posX, "y": posY, "z": posZ}
                print(detObj)
                """
                dataOK = 0

            ##########################---Target List TLV---############################
            elif tlv_type == MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX:  # 1011
                word = [1, 2**8, 2**16, 2**24]
                tid = byteBuffer2[idX]
                idX += 1
                idX += tlv_length - 1
            ##########################---Target List TLV---############################
            elif tlv_type == MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT:
                word = [1, 2**8, 2**16, 2**24]
                tid = byteBuffer2[idX]
                idX += 1
                maxZ = np.matmul(byteBuffer2[idX : idX + 4], word)
                idX += 4
                minZ = np.matmul(byteBuffer2[idX : idX + 4], word)
                idX += 4

                idX += tlv_length - 9

        # Remove already processed data
        if idX > 0 and byteBufferLength2 > idX:
            shiftSize = totalPacketLen

            byteBuffer2[: byteBufferLength2 - shiftSize] = byteBuffer2[
                shiftSize:byteBufferLength2
            ]
            byteBuffer2[byteBufferLength2 - shiftSize :] = np.zeros(
                len(byteBuffer2[byteBufferLength2 - shiftSize :]), dtype="uint8"
            )
            byteBufferLength2 = byteBufferLength2 - shiftSize

            # Check that there are no errors with the buffer length
            if byteBufferLength2 < 0:
                byteBufferLength2 = 0
        print(detObj_log)
        if detObj_log is not None:
            res = client.publish(topic=TOPIC, payload=(detObj_log), qos=0)
            # else:
            #     res = client.publish(topic=TOPIC, payload="Empty", qos=0)
            status = res[0]
            if status == 0:
                # msg_str = str(detObj_log)[:10] + "..."
                msg_str = "detObj_log"
                print(f"{CLIENT_ID}: Send `{msg_str}` to topic `{TOPIC}`\n\n")
            else:
                print(f"{CLIENT_ID}: Failed to send radar message to topic `{TOPIC}`\n")
    return dataOK, frameNumber, detObj


# ------------------------------------------------------------------
# Funtion to update the data and display in the plot
def update(client):
    dataOk = 0
    global detObj
    x = []
    y = []

    # Read and parse the received data
    dataOk, frameNumber, detObj = readAndParseData(
        Dataport1, configParameters, client, sensor_id=1
    )
    time.sleep(0.05)
    dataOk2, frameNumber2, detObj2 = readAndParseData2(
        Dataport2, configParameters, client, sensor_id=2
    )
    # print(f"dataOK = {dataOk}")
    # if dataOk and len(detObj["x"]) > 0:
    # print(detObj)
    # update_demo(detObj)

    return dataOk, dataOk2


# -------------------------    MAIN   -----------------------------------------

# Configurate the serial port
if __name__ == "__main__":
    CLIport1, Dataport1, CLIport2, Dataport2 = serialConfig1(configFileName)
    status = (
        CLIport1.isOpen(),
        CLIport2.isOpen(),
        Dataport1.isOpen(),
        Dataport2.isOpen(),
    )
    print(status)
    time.sleep(0.1)
    # CLIport2, Dataport2 = serialConfig2(configFileName)

    # Get the configuration parameters from the configuration file
    configParameters = parseConfigFile(configFileName)

    CLIport1.write(("sensorStart\n").encode())
    CLIport2.write(("sensorStart\n").encode())

    detObj = {}
    client = connect_mqtt(CLIENT_ID)
    while True:
        try:
            # Update the data and check if the data is okay
            client.loop_start()
            dataOk1, dataOk2 = update(client)

            time.sleep(0.05)
            client.loop_stop()

        # Stop the program and close everything if Ctrl + c is pressed
        except KeyboardInterrupt:
            print("LIVE RADAR: Process Terminated...")
            break
        except Exception as e:
            print("LIVE RADAR: Something went wrong...")
            # print(e)
            print(traceback.format_exc())
            break
    CLIport1.write(("sensorStop\n").encode())
    CLIport2.write(("sensorStop\n").encode())
    CLIport1.close()
    CLIport2.close()
    Dataport1.close()
    Dataport2.close()
    print(f"CLIport status: {CLIport1.is_open}")
    print(f"Dataport status: {Dataport1.is_open}")
    print(f"CLIport status: {CLIport2.is_open}")
    print(f"Dataport status: {Dataport2.is_open}")
