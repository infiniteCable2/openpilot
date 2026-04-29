#!/usr/bin/env python3
"""
Curvatured - Minimal test version
Tests if the mere EXISTENCE of this process causes locationd pipeline lag
"""
import time
import cereal.messaging as messaging

def main():
    pm = messaging.PubMaster(['liveCurvatureParameters'])
    sm = messaging.SubMaster(['carState'])

    while True:
        sm.update()
        time.sleep(0.25)  # 4Hz
        msg = messaging.new_message('liveCurvatureParameters')
        msg.valid = True
        msg.liveCurvatureParameters.version = 1
        pm.send('liveCurvatureParameters', msg)

if __name__ == "__main__":
    main()
