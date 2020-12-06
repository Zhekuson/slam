import torch
from torchvision.transforms.transforms import Resize
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
import cv2
from matplotlib.colors import Normalize
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
        self.transform1 = A.Compose([TT.ToTensor()])
        self.counter = 0
        self.lock = Lock()

    def subscribe(self, msg):
        self.lock.acquire()
        if self.last_update_time != None and time.time() - self.last_update_time > 3:
            
            if not msg[0]:
                raise RobotException("ERROR IN GETTING IMAGE FROM SCANNER")
            input = np.asarray(bytearray(msg[2]), dtype=np.uint8)
            image = input.reshape((msg[1][0], msg[1][1], 3))
            
            tensor = self.transform(image = image, bboxes = None, labels=None)["image"]
            tensor2 = self.transform1(image = image, bboxes = None, labels = None)["image"]
            

            depth_tensor = self.depth_model(torch.reshape(tensor2,(1,tensor2.size()[0],
                tensor2.size()[1],tensor2.size()[2])).float())
            start_time = time.time()
            depth_tensor = (depth_tensor - torch.min(depth_tensor)) / (torch.max(depth_tensor) - torch.min(depth_tensor))
            end_time = time.time()
            print("depth estimation took: "+str(end_time-start_time)+' sec')
            depth_image = torchvision.transforms.ToPILImage()(depth_tensor.squeeze(0))
            print(depth_image.size)
            
            #plt.imshow(depth_image,cmap='gray')
            #plt.show()
            start_time = time.time()
            detection_predict = self.detection_model([tensor])
            end_time = time.time()
            print("detection took: "+str(end_time-start_time)+' sec')
            if(detection_predict[0]['boxes'].size()[0]!=0):
                
                image = torchvision.transforms.ToPILImage()(tensor)
                print(image.size)
                draw = ImageDraw.Draw(image)

                for index, box in enumerate(detection_predict[0]['boxes']):
                    if(detection_predict[0]['scores'][index] > self.limit):
                        draw.rectangle([(box[0], box[1]), (box[2], box[3])], outline='white')
                  
                #plt.imshow(image)
                #plt.show()

            self.map_builder_callback(image)
            self.last_update_time = time.time() 
        self.lock.release()