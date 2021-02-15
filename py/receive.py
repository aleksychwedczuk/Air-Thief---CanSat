#AirOS Component - Receive/Parse Middlend
#Reads data sent by CanSat and parses it.
#Opens local server that sends JSON to the Frontend.
#@Alto

import serial
from flask import Flask, jsonify, request, after_this_request
from datetime import datetime
import time

#Instantiate app
app = Flask(__name__)

#Setup SerialListener
PORT = 'COM11'

#Config
CAN_SINKRATE_MAX = 9 #maximum fall speed
CAN_SINKRATE_MIN = 2 #minimum fall speed

LAT = -999
LON = -999

ALT = 0

LABELS = ["0:00.00" for i in range(10)]
SERIES = [0 for i in range(10)]

ITER = 0
cur = ""

@app.route('/telemetry', methods=['GET'])
def serveTelemetry():
    try:
        ser = serial.Serial(PORT, 9600, timeout=1000000)
    except:
        print("WAITING!!! :D")
    
    @after_this_request
    def add_header(response):
        response.headers.add('Access-Control-Allow-Origin', '*')
        return response

    #AC1320317EY+521983
    
    try:
        try:
            cur = ser.readline().decode().strip()
        except:
            return {"ERROR": "TRUE"}

        print(cur)

        now = datetime.now()
        current_time = now.strftime("%H:%M.%S")
        
        error = {"A": "NULErr",
                 "B": "SDCErr",
                 "C": "RFCErr",
                 "D": "ALTErr",
                 "E": "GPSErr",
                 "F": "TELErr",
                 "G": "RESErr",
                 "H": "FFAErr"}[cur[0]]

        mode = { "A": "STBY",
                 "B": "ACTI",
                 "C": "SMPL"}[cur[1]]

        modenice = { "A": "Standby",
                     "B": "Active",
                     "C": "Sample"}[cur[1]]

        if (cur[8].isdigit() or cur[8] in "+-"):
            spd = cur[8]
        else:
            spd = str(ord("a") - 55)

        if (cur[9].isdigit()):
            head = str(int(cur[9]) * 10)
        else:
            head = str((ord("a") - 55) * 10)

        if (cur[10] == "Y"):
            global LAT
            LAT = int(cur[11:19])
        else:
            global LON
            LON = int(cur[11:19])

        global ALT
        try:
            deltalt = int(cur[4:8]) - ALT
        except:
            deltalt = 0

        global LABELS
        global SERIES

        try:
            ALT = int(cur[4:8]) #regen alt
        except:
            ALT = 0


        LABELS = [current_time] + LABELS[:-1]
        SERIES = [ALT] + SERIES[:-1]
        
        jsonResp = {'ERROR'    : error,
                    'MODE'     : mode,
                    'SAMPLING' : cur[2:4],
                    'ALT'      : cur[4:8],
                    'VELO'     : spd,
                    'DIR'      : head,
                    'LAT'      : LAT,
                    'LON'      : LON,
                    'RAW'      : cur[:-3],
                    'FALL'     : deltalt,
                    'MODENICE' : modenice,
                    'DATA'     : {
                        'labels' : LABELS,
                        'series' : [SERIES]
                    },
                    'RSSI'     : cur[19:22]}

        print(jsonResp)
        ser.close()
        return jsonify(jsonResp)
        
    except: #############
        print("NO ADDITIONAL DATA\n")
        ser.close()
        return "ERR"
        #ser.close()
        #time.sleep(3)
        #ser = serial.Serial(PORT, 9600, timeout=1000000)
        
        

if __name__ == '__main__':
    app.run(host='localhost', port=4170)
