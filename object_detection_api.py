from fastapi import FastAPI, UploadFile, File
from ultralytics import YOLO
from PIL import Image
import numpy as np
import io

app = FastAPI()

model = YOLO("obj_det_model.pt")

@app.post("/detect")
async def detect(file: UploadFile = File(...)):


    image_bytes = await file.read()
    pil_image = Image.open(io.BytesIO(image_bytes))


    img_np = np.array(pil_image)


    results = model.predict(source=img_np, save=False)


    detections = results[0].boxes.data.tolist() 
    return {"detections": detections}
