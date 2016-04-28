"""
@author: <seantrott@icsi.berkeley.edu

A simple program to output n-tuples using Analyzer+Specializer. Not reliant on any packages other than Jython.


"""

from robot_specializer import *
from nluas.ntuple_decoder import *
import traceback
import pprint

decoder = NtupleDecoder()

analyzer = Analyzer("http://localhost:8090")
rs = RobotSpecializer(analyzer)

while True:
    text = input("> ")
    if not text:
        pass
    elif text == "q":
        quit()
    elif text == "d":
        rs.debug_mode = True
    else:
        try:
            semspecs = analyzer.parse(text)
            for fs in semspecs:
                try:
                    ntuple = rs.specialize(fs)
                    pprint.pprint(ntuple)
                    #decoder.pprint_ntuple(ntuple)
                    break
                except Exception as e:
                    traceback.print_exc()
                    print(e)
        except Exception as e:
            traceback.print_exc()
            print(e)
