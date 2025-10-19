import cv2
import torch
import time
from flask import Flask, request, jsonify

device = torch.device("cuda")
img_path = r"../NAO_golf/save_pictures/mark.jpg"
model = torch.hub.load(r'../NAO_mark', 'custom',
                       path=r'C:\Users\jdf20\Desktop\HNIT-NAO\NAO_mark\best.pt', source="local")
model.to(device)
model.eval()

app = Flask(__name__)


@app.route("/index", methods=["GET"])  #route
def process_image():
    # 构建要返回的JSON数据
    result = call_yolo()
    # 返回JSON响应
    return jsonify(result)




def call_yolo():
    results = model(img_path)
    boxes = results.xyxy[0].tolist()

    img = cv2.imread(img_path)
    img_copy = img.copy()
    if boxes:
        lx, ly, rx, ry, confidence, no = boxes[0]
        pt1 = (int(lx), int(ly))  # 左上角顶点坐标
        pt2 = (int(rx), int(ry))  # 右下角顶点坐标
        cv2.rectangle(img_copy, pt1, pt2, (255, 0, 255), 2)
        cv2.imwrite(r"../NAO_golf/save_pictures/yolov5.jpg", img_copy)
        return {
            "pt1": pt1,
            "pt2": pt2
        }
    cv2.imwrite(r"../NAO_golf/save_pictures/yolov5.jpg", img_copy)
    return None


if __name__ == "__main__":
    app.run(host="127.0.0.1")
