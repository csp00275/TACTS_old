import serial

simple=serial.Serial('COM5',115200,timeout=1)



while (True):
    while (simple.inWaiting()==0):
        pass
    SdataPacket = simple.readline()
    SdataPacket = str(SdataPacket, "utf-8")
    SsplitPacket = SdataPacket.split(" ")

    theta0 = float(SsplitPacket[0])
    theta1 = float(SsplitPacket[1])
    print("theta0=",theta0,"theta1=",theta1)






