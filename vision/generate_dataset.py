#!/usr/bin/python3
"""!
:file: main.py
:brief: Script for generation of facial emotion dataset using stable diffusion
:detail: Based on https://github.com/computervisioneng/create-synthetic-dataset-emotion-recognition/tree/main
:author: Joseph Smith
:requirements:
- Requires cuda enabled GPU
- pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
- pip install --upgrade diffusers[torch] transformers matplotlib
"""

import os
import random
import torch
import matplotlib.pyplot as plt
from diffusers import DiffusionPipeline

if not torch.cuda.is_available():
    print("CUDA not enabled on GPU")
    exit(-1)

# Create the diffusion pipeline object
pipeline = DiffusionPipeline.from_pretrained("runwayml/stable-diffusion-v1-5", 
                                             torch_dtype=torch.float16)
pipeline.to("cuda")

# Create folders if they don't already exist
os.makedirs('./img/happy', exist_ok=True)
os.makedirs('./img/neutral', exist_ok=True)
os.makedirs('./img/sad', exist_ok=True)

# Create prompt choices
NUM_OF_IMAGES = 200
ETHNICITIES = ['a latino', 'a european', 'an african', 'a middle eastern',
               'an indian', 'an asian']
GENDERS = ['male', 'female']
EMOTION_PROMPTS = {'happy': 'smiling, happy face expression',
                   'neutral': 'neutral face expression, straight face',
                   'sad': 'frowning, sad face expression, crying'}

for img_num in range(NUM_OF_IMAGES):
  for emotion in EMOTION_PROMPTS.keys():
    emotion_prompt = EMOTION_PROMPTS[emotion]
    ethnicity = random.choice(ETHNICITIES)
    gender = random.choice(GENDERS)

    prompt = f"Medium-shot portrait of {ethnicity} {gender}, {emotion_prompt}, " \
              'front view, looking at the camera, color photography, ' \
              'photorealistic, hyperrealistic, realistic, incredibly detailed, ' \
              'crisp focus, digital art, depth of field, 50mm, 8k'
    neg_prompt = '3d, cartoon, anime, sketches, (worst quality:2), ' \
                 '(low quality:2), (normal quality:2), lowres, normal quality, ' \
                 '((monochrome)), ((grayscale)) Low Quality, Worst Quality, ' \
                 'plastic, fake, disfigured, deformed, blurry, bad anatomy, ' \
                 'blurred, watermark, grainy, signature'

    img = pipeline(prompt, negative_prompt=neg_prompt).images[0]
    img.save(f"./img/{emotion}/{str(img_num).zfill(4)}.png")
