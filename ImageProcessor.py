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
import torch.nn.functional as F
from slam_threading.ParallelExecutor import ParallelExecutor
from slam_threading.ParallelExecutor import Job
from utils import clear_folder


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
class ImageProcessor():
    def __init__(self, map_builder_callback):
        self.map_builder_callback = map_builder_callback
        self.depth_model = DepthEstimationModel.load_model()
        self.detection_model = ObjectDetectionModel.load_model()
        self.limit = 0.95
        self.last_update_time = 0
        self.transform = A.Compose([TT.ToTensor()])
        self.transform_depth = A.Compose([TT.ToTensor()])
        self.counter = 0
        self.parallel_executor = ParallelExecutor()
        self.lock = Lock()
        self.detection_images_folder = './images/detection/'
        self.depth_images_folder = './images/depth_estimation/'
        clear_folder(self.detection_images_folder)
        clear_folder(self.depth_images_folder)



    def subscribe(self, msg):
        self.lock.acquire()

        if self.last_update_time != None and time.time() - self.last_update_time > 3:
            
            if not msg[0]:
                raise RobotException("Error in getting image from visual sensor")

            input = np.asarray(bytearray(msg[2]), dtype=np.uint8)
            image = input.reshape((msg[1][0], msg[1][1], 3))
            tensor = self.transform(image = image, bboxes = None, labels=None)["image"].to(device)

            detection_job = Job(lambda: self.detect_objects_and_save_images(tensor), "Detect objects")
            depth_est_job = Job(lambda: self.estimate_depth_and_save_image(tensor), "Estimating depth")
            result = self.parallel_executor.execute([detection_job, depth_est_job])
            
            self.map_builder_callback(result)
            self.last_update_time = time.time() 

        self.lock.release()


    def detect_objects(self, tensor):
        detection_predict = self.detection_model([tensor])
        suitable_boxes = []

        if detection_predict[0]['boxes'].size()[0]!=0:
            for index, box in enumerate(detection_predict[0]['boxes']):
                if detection_predict[0]['scores'][index] > self.limit:
                    suitable_boxes.append(box)

        return suitable_boxes
                

    def detect_objects_and_save_images(self, tensor):
        detection_predict = self.detection_model([tensor])
        suitable_boxes = []
        if detection_predict[0]['boxes'].size()[0]!=0:
            image = torchvision.transforms.ToPILImage()(tensor)
            draw = ImageDraw.Draw(image)

            for index, box in enumerate(detection_predict[0]['boxes']):
                if detection_predict[0]['scores'][index] > self.limit:
                    suitable_boxes.append(box)
                    draw.rectangle([(box[0], box[1]), (box[2], box[3])], outline='white')
                
            fig, ax = plt.subplots( nrows=1, ncols=1 )
            ax.imshow(image)
            fig.savefig(self.detection_images_folder + str(self.last_update_time) + '.png')

        return suitable_boxes


    def estimate_depth(self, tensor):
        shape = (1, tensor.size()[0], tensor.size()[1], tensor.size()[2])
        depth_tensor = self.depth_model(torch.reshape(tensor, shape).float().to(device))
        depth_tensor = (depth_tensor - torch.min(depth_tensor)) / (torch.max(depth_tensor) - torch.min(depth_tensor))
        return F.interpolate(depth_tensor, size = (tensor.size()[1], tensor.size()[2]))


    def estimate_depth_and_save_image(self, tensor):
        shape = (1, tensor.size()[0], tensor.size()[1], tensor.size()[2])
        depth_tensor = self.depth_model(torch.reshape(tensor, shape).float().to(device))
        depth_tensor = (depth_tensor - torch.min(depth_tensor)) / (torch.max(depth_tensor) - torch.min(depth_tensor))
        depth_tensor = F.interpolate(depth_tensor, size = (tensor.size()[1], tensor.size()[2]))
        depth_image = torchvision.transforms.ToPILImage()(depth_tensor.squeeze(0))

        fig, ax = plt.subplots( nrows=1, ncols=1 )
        ax.imshow(depth_image, cmap='gray')
        fig.savefig(self.depth_images_folder + str(self.last_update_time) + '.png')

        return depth_tensor