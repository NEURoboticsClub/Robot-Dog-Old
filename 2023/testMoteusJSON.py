import json

def parsingJson(fileName):
    #while True:
    f = open(fileName)
    allData = json.load(f) #load json file data as a library
    for i in allData:
        data = allData[i]
        #print(data)
        motorId = data[0]['motor-id']
        #['motor-id']
        pos = data[0]['position']
        vel = data[0]['velocity']
        torque = data[0]['torque']
        self.setAttributes(canId=motorId, pos=pos, velocity=vel, torque=torque)
        #print("MotorID= " + str(motorId) + " pos= " + str(pos) + " vel= " + str(vel) + " torque= " + str(torque) )

parsingJson("exampleMotorInput.JSON")
