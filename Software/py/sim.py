#AirOS Component - Receive/Parse Middlend
#Reads data sent by CanSat and parses it.
#Opens local server that sends JSON to the Frontend.
#@Alto

import serial
from flask import Flask, jsonify, request, after_this_request
from datetime import datetime
import time

import random

#Instantiate app
app = Flask(__name__)

#Setup SerialListener
PORT = 'COM11'

#Config
CAN_SINKRATE_MAX = 9 #maximum fall speed
CAN_SINKRATE_MIN = 2 #minimum fall speed

LAT = 521983
LON = 210843

ALT = 2500

LABELS = ["0:00.00" for i in range(20)]
SERIES = [0 for i in range(20)]

ITER = 0
cur = ""

SAMPLE = 0

ERRORBURN = 0

@app.route('/telemetry', methods=['GET'])
def serveTelemetry():
    global ITER
    global ERRORBURN
    ITER += 1
    try:
        #ser = serial.Serial(PORT, 9600, timeout=1000000)
        pass
    except:
        print("WAITING!!! :D")
    
    @after_this_request
    def add_header(response):
        response.headers.add('Access-Control-Allow-Origin', '*')
        return response

    #AC1320317EY+521983
    
    try:
        if (ITER % 5 == 0):
            pass
        else:
            return "ERR"

        print(cur)

        now = datetime.now()
        current_time = now.strftime("%H:%M.%S")


        if (ERRORBURN > 0):
            ERRORBURN -= 1
            error = "SDCErr"
        else:
            if (random.randint(0, 10) == 0):
                error = "SDCErr"
                ERRORBURN = random.randint(3, 5)
            else:
                error = "NULErr"
        errornice = {"NULErr": "NO ERROR",
                     "SDCErr": "SD Card Error",
                     "RFCErr": "LoRa Error",
                     "ALTErr": "Altimeter Error",
                     "GPSErr": "GPS Error",
                     "F": "Telemetry Error",
                     "G": "In-Flight RESET",
                     "H": "No-Chute Error"}[error]
        
        global ALT

        if (ALT > 500):
            mode = 'SMPL'
        else:
            mode = 'ACTI'
        
        modenice = {"STBY": "Standby",
                     "ACTI": "Active",
                     "SMPL": "Sample",
                     "ALTErr": "Altimeter Error",
                     "E": "GPS Error",
                     "F": "Telemetry Error",
                     "G": "In-Flight RESET",
                     "H": "No-Chute Error"}[mode]

        
        

        spd = random.randint(0, 5)

        head = random.randint(0, 36) * 10

        global LAT
        global LON

        LAT += random.randint(-2, 3)
        LON += random.randint(-2, 4)

        global LABELS
        global SERIES

        

        global SAMPLE

        if (ALT > 500):
            SAMPLE += random.randint(0, 1)

        deltalt = random.randint(5, 8)

        ALT = ALT - deltalt


        LABELS = [current_time] + LABELS[:-1]
        SERIES = [ALT] + SERIES[:-1]
        
        jsonResp = {'ERROR'    : error,
                    'MODE'     : mode,
                    'SAMPLING' : SAMPLE,
                    'ALT'      : ALT,
                    'VELO'     : spd,
                    'DIR'      : head,
                    'LAT'      : LAT,
                    'LON'      : LON,
                    'RAW'      : 'AC1320317EY+521983',
                    'FALL'     : deltalt,
                    'MODENICE' : modenice,
                    'DATA'     : {
                        'labels' : LABELS,
                        'series' : [SERIES]
                    },
                    'RSSI'     : -1 * random.randint(20, 80),
                    'ERRORNICE': errornice}

        print(jsonResp)
        #ser.close()
        return jsonify(jsonResp)
        
    except ZeroDivisionError: #############
        print("NO ADDITIONAL DATA\n")
        #ser.close()
        return "ERR"
        #ser.close()
        #time.sleep(3)
        #ser = serial.Serial(PORT, 9600, timeout=1000000)
    
        
        

if __name__ == '__main__':
    app.run(host='localhost', port=4170)
