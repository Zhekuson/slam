import torch
from MapBuilder import MapBuilder
import numpy as np
from RobotException import RobotException
from vision.object_detection import ObjectDetectionModel
from vision.depth import DepthEstimationModel
import matplotlib.pyplot as plt
import torchvision
import albumentations.pytorch.transforms as TT
import albumentations as A
from PIL import ImageDraw
import time
from threading import Lock
class ImageProcessor():
    def __init__(self, map_builder_callback):
        """
        docstring
        """
        self.map_builder_callback = map_builder_callback
        self.depth_model = DepthEstimationModel.load_model()
        self.detection_model = ObjectDetectionModel.load_model()
        self.limit = 0.95
        self.last_update_time = 0
        self.transform = A.Compose([TT.ToTensor()])
        self.counter = 0
        self.lock = Lock()

    def subscribe(self, msg):
        print("Images count "+ str(self.counter))
        self.counter += 1
        self.lock.acquire()
        if self.last_update_time != None and time.time() - self.last_update_time > 3:
            
            if not msg[0]:
                raise RobotException("ERROR IN GETTING IMAGE FROM SCANNER")
            input = np.asarray(bytearray(msg[2]), dtype=np.uint8)
            image = input.reshape((msg[1][0], msg[1][1], 3))
            
            tensor = self.transform(image = image, bboxes = None, labels=None)["image"]
            detection_predict = self.detection_model([tensor])
            if(detection_predict[0]['boxes'].size()[0]!=0):
                
                image = torchvision.transforms.ToPILImage()(tensor)
                draw = ImageDraw.Draw(image)

                for index, box in enumerate(detection_predict[0]['boxes']):
                    if(detection_predict[0]['scores'][index] > self.limit):
                        draw.rectangle([(box[0], box[1]), (box[2], box[3])], outline='white')
                  
                plt.imshow(image)
                plt.show()
            
            self.map_builder_callback(image)
            self.last_update_time = time.time() 
        self.lock.release()