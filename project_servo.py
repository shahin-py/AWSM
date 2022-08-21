import cv2
import os
import sys, getopt
import signal
import time
from edge_impulse_linux.image import ImageImpulseRunner
import RPi.GPIO as GPIO
import serial
runner = None

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18, GPIO.IN)
GPIO.setup(17, GPIO.IN)
GPIO.setup(27, GPIO.IN)
GPIO.setup(24, GPIO.IN)
g = GPIO.input(18)
i = GPIO.input(27)
h = GPIO.input(17)
k = GPIO.input(24)
b = 0
# Motor Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.output(22, GPIO.LOW)
GPIO.output(23, GPIO.LOW)
servo1 = GPIO.PWM(26, 50)  # Servo initialize


def forward():
    GPIO.output(22, GPIO.HIGH)
    GPIO.output(23, GPIO.LOW)


def reverse():
    GPIO.output(22, GPIO.LOW)
    GPIO.output(23, GPIO.HIGH)


def motor():
    servo1.start(0)
    servo1.ChangeDutyCycle(7)
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)
    time.sleep(1.5)

    # turn back to 0 degrees
    servo1.ChangeDutyCycle(2)
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)
    servo1.stop()


# if you don't want to see a camera preview, set this to False


show_camera = False
if (sys.platform == 'linux' and not os.environ.get('DISPLAY')):
    show_camera = False


def now():
    return round(time.time() * 1000)


def get_webcams():
    port_ids = []
    for port in range(5):
        print("Looking for a camera in port %s:" %port)
        camera = cv2.VideoCapture(port)
        if camera.isOpened():
            ret = camera.read()[0]
            if ret:
                backendName =camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print("Camera %s (%s x %s) found in port %s " %(backendName,h,w, port))
                port_ids.append(port)
            camera.release()
    return port_ids

def sigint_handler(sig, frame):
    print('Interrupted')
    if (runner):
        runner.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

def help():
    print('python classify.py <path_to_model.eim> <Camera port ID, only required when more than 1 camera is present>')

def main(argv):
    try:
        opts, args = getopt.getopt(argv, "h", ["--help"])
    except getopt.GetoptError:
        help()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            help()
            sys.exit()

    if len(args) == 0:
        help()
        sys.exit(2)

    model = args[0]

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    print('MODEL: ' + modelfile)

    with ImageImpulseRunner(modelfile) as runner:
        try:
            while(True):
                
                model_info = runner.init()
                print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
                labels = model_info['model_parameters']['labels']
                if len(args)>= 2:
                    videoCaptureDeviceId = int(args[1])
                else:
                    port_ids = get_webcams()
                    if len(port_ids) == 0:
                        raise Exception('Cannot find any webcams')
                    if len(args)<= 1 and len(port_ids)> 1:
                        raise Exception("Multiple cameras found. Add the camera port ID as a second argument to use to this script")
                    videoCaptureDeviceId = int(port_ids[0])

                camera = cv2.VideoCapture(videoCaptureDeviceId)
                ret = camera.read()[0]
                if ret:
                    backendName = camera.getBackendName()
                    w = camera.get(3)
                    h = camera.get(4)
                    print("Camera %s (%s x %s) in port %s selected." %(backendName,h,w, videoCaptureDeviceId))
                    camera.release()
                else:
                    raise Exception("Couldn't initialize selected camera.")

                next_frame = 0   # limit to ~10 fps here

                for res, img in runner.classifier(videoCaptureDeviceId):
                    if (next_frame > now()):
                        time.sleep((next_frame - now()) / 1000)

                    # print('classification runner response', res)

                    if "classification" in res["result"].keys():
                        print('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
                        for label in labels:
                            score = res['result']['classification'][label]
                            print('%s: %.2f\t' % (label, score), end='')
                        print('', flush=True)

                    elif "bounding_boxes" in res["result"].keys():
                        print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                        for bb in res["result"]["bounding_boxes"]:
                            print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                            img = cv2.rectangle(img, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)
                            a=(bb['label'])
                            b='Paper'
                            c='Plastic'
                            
                            if (c==a):
                                print(a)
                                g=GPIO.input(18)
                                h=GPIO.input(17)
                                
                                while(g==True):
                                    
                                    reverse()
                                    g=GPIO.input(18)
                                    
                                
                                GPIO.output(22,GPIO.LOW)
                                GPIO.output(23,GPIO.LOW)
                                time.sleep(1)
                                motor()
                                
                                GPIO.output(26,GPIO.LOW)

                                time.sleep(1)
                                while(h==True):
                                    
                                    forward()
                                    h=GPIO.input(17)
                            
                                GPIO.output(22,GPIO.LOW)
                                GPIO.output(23,GPIO.LOW)
                            elif (b==a):
                                print(a)
                                i=GPIO.input(27)
                                h=GPIO.input(17)
                                
                                print(i)
                                while(i==True):
                                    
                                    reverse()
                                    i=GPIO.input(27)
                                    
                                
                                GPIO.output(22,GPIO.LOW)
                                GPIO.output(23,GPIO.LOW)
                                time.sleep(1)
                                motor()
                                

                                time.sleep(1)
                                while(h==True):
                                    
                                    forward()
                                    h=GPIO.input(17)
                            
                                GPIO.output(22,GPIO.LOW)
                                GPIO.output(23,GPIO.LOW)
                    if (show_camera):
                        cv2.imshow('edgeimpulse', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                        if cv2.waitKey(1) == ord('q'):
                            break

                    next_frame = now() + 100
        finally:
            if (runner):
                runner.stop()

if __name__ == "__main__":
   main(sys.argv[1:])
