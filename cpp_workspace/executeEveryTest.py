#!/usr/bin/python
OUTPUT_FILENAME = "mtz.results"
INPUT_METHOD = "mtz"

k = {"g01.dat" : [2,5,10], "g02.dat" : [4,10,20], "g03.dat" : [10,25,50], "g04.dat" : [14,35], "g05.dat" : [20,50], "g06.dat" : [40,100]}

import subprocess

subprocess.call( ['rm %s.results' % INPUT_METHOD], shell=True)
subprocess.call( ['touch', '%s.results' % INPUT_METHOD] )

for eachFile in k.keys():
  for eachK in k[eachFile]:
    p = subprocess.call(
        ['./kmst -m %s -f data/%s -k %d' % (INPUT_METHOD, eachFile, eachK)], shell=True )
    #out, err = p.communicate() 
    
    subprocess.call( ['cat cplex_output >> %s.results' % INPUT_METHOD], shell=True )
    #    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    #out, err = p.communicate()
