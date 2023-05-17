'''
# Connect the 3D Arduino-based LiDAR scanner performing the scan and launch this file.
# Use : 
#   python stepperScanner-serial2CSV.py --port the_port_of_connected_arduino_board --baudrate 115200
# 
# Serial command :
#    s : make a 3D scan
#    y : make a 2D scan
'''
import csv
from time import sleep

import serial
import argparse

def main (args) :

    print( '--- Connecting to port : ' + args.port + ' at ' + str( args.baudrate )  +' bauds' )
    print( 'D')
    
    dataSerie = serial.Serial( port = args.port, baudrate = args.baudrate)

    listG = []
    nb_of_scan = 0

    ## Double while True loops need to be tested
    while True :
        data = input()
        #print( data )
        dataSerie.write(str.encode(data))
        
        if( data == 's' or data == 'y' ) :
            nb_of_scan += 1
            dimension = ''
            suffix = ''
            # This has been committed on the branch "saved" for further integration if needed
            if( nb_of_scan == 1 ):
                suffix = '-background'
            elif( nb_of_scan == 2 ):
                suffix = '-with_objects'
            ###
            if( data == 's' ):
                dimension = '3D'
            elif( data =='y' ):
                dimension == '2D'
            filename = "stepperscanner-" + dimension + suffix + ".csv"

            while True:
                # sleep(0.1)

                mesure = dataSerie.readline().strip()
                mesureUtf = mesure.decode("utf-8")
                mesure1 = mesureUtf.split()

                print(mesure1)

                if mesureUtf == "stop":

                    with open( filename, 'w' ) as f :
                        writer = csv.writer( f )
                        print( len( listG ) )
                        writer.writerows( listG )
                        break
                else: ## TODO : add all elements not just xYZ and then during parsing put them in the right dict keys
                    #listG.append( [ mesure1[ 6 ], mesure1[ 7 ], mesure1[ 8 ] + ";"] )
                    listG.append( mesureUtf )
        #else :
        #    while True:
        #        bytesToRead = dataSerie.inWaiting()
        #        print( dataSerie.read( bytesToRead ) )


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser( description = 'Receive serial data from scanner and record it into a csv file.' )
    parser.add_argument( "-p", "--port",
                        help = "Serial port used to connect Arduino Uno USB (no default)",
                        default = "" )
    parser.add_argument( "-b", "--baudrate", 
                        help = "Speed of the serial communication (default is 115200)",
                        type = int, default = "115200" )
    parser.add_argument( "-f", "--file",
                        help = "Path of the file you want to record the data in",
                        type= str, default = "./scanning_data.csv" )
    args = parser.parse_args()

    main( args )