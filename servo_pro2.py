import cv2
import os
import sys, getopt
import numpy as np
from edge_impulse_linux.image import ImageImpulseRunner
import signal
import time
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

def help():
    print('python classify-image.py <path_to_model.eim> <path_to_image.jpg>')

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

    if len(args) != 2:
        help()
        sys.exit(2)

    model = args[0]

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    print('MODEL: ' + modelfile)

    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
            labels = model_info['model_parameters']['labels']

            img = cv2.imread(args[1])
            if img is None:
                print('Failed to load image', args[1])
                exit(1)

            # imread returns images in BGR format, so we need to convert to RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # get_features_from_image also takes a crop direction arguments in case you don't have square images
            features, cropped = runner.get_features_from_image(img)

            # the image will be resized and cropped, save a copy of the picture here
            # so you can see what's being passed into the classifier
            cv2.imwrite('debug.jpg', cv2.cvtColor(cropped, cv2.COLOR_RGB2BGR))

            res = runner.classify(features)

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

        finally:
            if (runner):
                runner.stop()

if __name__ == "__main__":
   main(sys.argv[1:])