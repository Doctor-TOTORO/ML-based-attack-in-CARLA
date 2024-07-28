from classify import predict
from simba import SimBA
import argparse
import os
import platform
import sys
from pathlib import Path
from models import yolo
from PIL import Image
import torchvision.transforms as transforms

import torch
import torch.nn.functional as F

FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.join(os.path.relpath(ROOT, Path.cwd()), "yolov5"))  # relative

model = yolo.Model(cfg=ROOT / "models/yolov5m.yaml")
classify_model = yolo.ClassificationModel(cfg=ROOT / "models/yolov5m.yaml", model=model)



opt = {
        "weights": str(ROOT / "yolov5s-cls.pt"),
        "source": str(ROOT),
        "data": str(ROOT / "data/coco128.yaml"),
        "imgsz": (224, 224),
        "device": "",
        "view_img": False,
        "save_txt": False,
        "nosave": False,
        "augment": False,
        "visualize": False,
        "update": False,
        "project": str(ROOT / "runs/predict-cls"),
        "name": "exp",
        "exist_ok": False,
        "half": False,
        "dnn": False,
        "vid_stride": 1
    }
model.eval()
image_path = os.path.join(os.getcwd(), "005286.png")
image = Image.open(image_path).convert('RGB')
transform = transforms.ToTensor()
tensor = transform(image)
results = classify_model(tensor.unsqueeze(0))
print(results)
attacker = SimBA(classify_model, 224)
_, max_index = torch.max(results, dim=1)
fake_image = attacker.simba_single(tensor, max_index)
print("Done")