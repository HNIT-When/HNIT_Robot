import cv2
import time
from flask import Flask, request, jsonify
from naoqi import ALProxy
import cv2
import numpy as np
from PIL import Image
import io
img_path = r"../NAO_golf/save_pictures/mark.jpg"


app = Flask(__name__)


@app.route("/index", methods=["GET"])  #route
def process_image():
    result = detect_naomark_in_image("192.168.31.63",img_path)
    return jsonify(result)


def detect_naomark_in_image(robot_ip, img_path,robot_port=9559):
    try:

        vision_proxy = ALProxy("ALVision", robot_ip, robot_port)

        vision_proxy.setParam(vision_proxy.kMarkerDetectionType, "ALMarker")


        img = cv2.imread(image_path)
        if img is None:
            print("CAN NOT READ THE IMG")
            return None

        img_copy = img.copy()

        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(img_rgb)


        img_byte_arr = io.BytesIO()
        pil_img.save(img_byte_arr, format='JPEG')
        img_byte_arr = img_byte_arr.getvalue()




        height, width = img.shape[:2]


        markers = vision_proxy.detectMarkers()

        if markers and len(markers) > 0:

            for marker in markers:
                marker_id, _, _, _, img_coords = marker
                x, y, marker_width, marker_height = img_coords


                lx = int(x - marker_width / 2)
                ly = int(y - marker_height / 2)
                rx = int(x + marker_width / 2)
                ry = int(y + marker_height / 2)


                lx = max(0, min(lx, width))
                ly = max(0, min(ly, height))
                rx = max(0, min(rx, width))
                ry = max(0, min(ry, height))

                pt1 = (lx, ly)
                pt2 = (rx, ry)


                cv2.rectangle(img_copy, pt1, pt2, (0, 255, 0), 2)

                cv2.imwrite(r"../NAO_golf/save_pictures/yolov5.jpg", img_copy)

                return {
                    "pt1": pt1,
                    "pt2": pt2,
                    "marker_id": marker_id,
                    "confidence": 1.0
                }


        cv2.imwrite(r"../NAO_golf/save_pictures/yolov5.jpg", img_copy)
        return None

    except Exception as e:
        print("Naomark WRONG:")
        return None



if __name__ == "__main__":
    app.run(host="127.0.0.1")
