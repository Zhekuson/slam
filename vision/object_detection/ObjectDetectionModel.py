import torchvision
from torchvision.models.detection import fasterrcnn_resnet50_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor 
import torch
import torch.nn as nn
class ObjectDetectionModel(nn.Module):
    def _get_detection_model(self, num_classes):
        model = fasterrcnn_resnet50_fpn(pretrained=True)
        # get the number of input features for the classifier
        in_features = model.roi_heads.box_predictor.cls_score.in_features
        # replace the pre-trained head with a new one
        model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
        return model


def get_device():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return device


def get_model(LOAD_DIR = 'vision/object_detection/'):
    model = ObjectDetectionModel()._get_detection_model(num_classes=2)
    model.load_state_dict(torch.load(LOAD_DIR +'detection_model.pt', map_location='cpu'))
    model.eval()
    print("Object detection model loaded")
    return model
