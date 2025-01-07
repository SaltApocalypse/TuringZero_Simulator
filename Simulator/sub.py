import tbkpy._core as tbkpy
import time
import pickle


def control(msg):
    re = pickle.loads(msg)
    print(f"{time.time()}: {re}")


def main():
    info = tbkpy.EPInfo()
    info.msg_name = "RPM"
    info.name = "RPM"
    register = tbkpy.Subscriber(info, control)
    while True:
        time.sleep(0.1)
 

main()
