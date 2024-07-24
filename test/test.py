import time
import os

import yolov5
import torchvision.transforms as transforms
from torchvision.transforms import ToPILImage
from PIL import Image
import torch
from simba import SimBA
from l2_attack_black import attack
from hsja import hsja
import numpy as np
import predict
import carla

# load city
subfolder_name1 = 'city_attack'
subfolder_name2 = 'simba'
subfolder_name3 = 'freeway_attack'
subfolder_path1 = os.path.join(os.getcwd(), subfolder_name3)
subfolder_path2 = os.path.join(os.getcwd(), subfolder_name3, subfolder_name2)
subfolder_path3 = os.path.join(os.getcwd(), subfolder_name1)
subfolder_path4 = os.path.join(os.getcwd(), subfolder_name1, subfolder_name2)
files = os.listdir(subfolder_path1)
image_files = [file for file in files if file.endswith('.png')]
images = []
for i in range(1, 24):
    image_path = os.path.join(subfolder_path3, f'{i}.png')
    image = Image.open(image_path).convert('RGB')
    images.append(image)

# simba
model = YOLO('yolov5n.pt')
count = 0
'''
for image in images:
    count += 1
    transform = transforms.ToTensor()
    tensor = transform(image)
    task_map = model.task_map
    classification_model = task_map["classify"]["model"]
    clf_model = classification_model(cfg="yolov8n-cls.yaml", ch=3, nc=80, verbose=True)
    clf_model.eval()
    attacker = SimBA(clf_model, 640)
    results = clf_model(tensor.unsqueeze(0))
    _, max_index = torch.max(results, dim=1)
    fake_image = attacker.simba_single(tensor, max_index)
    img_pil = transforms.ToPILImage()(fake_image)
    img_pil.save(os.path.join(subfolder_path4, f'{count}.png'))
    print(f"finished {count} images")
'''
transform = transforms.ToTensor()
tensor = transform(images[0])
task_map = model.task_map
classification_model = task_map["classify"]["model"]
clf_model = classification_model(cfg="yolov5n-cls.yaml", ch=3, nc=80, verbose=True)
clf_model.eval()
attacker = SimBA(clf_model, 640)
results = clf_model(tensor.unsqueeze(0))
_, max_index = torch.max(results, dim=1)
print(f"finished {count} images")
print("Done")


# hsja
'''
model = YOLO('yolov8n.pt')
img = Image.open('005286.png').resize((640, 640))
img_rgb = img.convert('RGB')
task_map = model.task_map
classification_model = task_map["classify"]["model"]
clf_model = classification_model(cfg="yolov8n-cls.yaml", ch=3, nc=80, verbose=True)
clf_model.eval()
start_time = time.time()
fake_image = hsja(clf_model, np.array(img_rgb))
print(f"Over: {time.time() - start_time}")
print(type(fake_image))
'''

# zoo
'''
tf.compat.v1.disable_v2_behavior()
model = YOLO('yolov8n.pt')
img = Image.open('005286.png').resize((640, 640))
img_rgb = img.convert('RGB')
transform = transforms.ToTensor()
tensor = transform(img_rgb)
task_map = model.task_map
classification_model = task_map["classify"]["model"]
clf_model = classification_model(cfg="yolov8n-cls.yaml", ch=3, nc=80, verbose=True)
clf_model.eval()
tensor_np = tensor.detach().numpy()
tf.compat.v1.disable_eager_execution()
sess = tf.compat.v1.Session()
start = time.time()
results = clf_model(tensor.unsqueeze(0))
_, max_index = torch.max(results, dim=1)
fake_image = attack(inputs=np.array(img_rgb), targets=max_index.item(), model=clf_model, targeted=False, use_log=False, use_tanh=True, solver="newton", device="cpu")
img_pil = transforms.ToPILImage()(torch.from_numpy(fake_image))
'''