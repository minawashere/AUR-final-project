import cv2, time
import numpy as np
from qreader import QReader

cap = cv2.VideoCapture (1)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

qreader = QReader()

prev = 0
new = 0

while (cap.isOpened ()):

    _, img = cap.read ();

    img = cv2.resize(img, (400, 400), fx = 0, fy = 0,interpolation = cv2.INTER_CUBIC)

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    decoded_text, detections = qreader.detect_and_decode(image=img, return_detections=True)

    new = time.time ()
    print (f"fps = {1/(new - prev)}")
    prev = new

    for i, qr in enumerate(decoded_text):
        print(f"QR Code {i+1}: {qr}")
        
        # Get the bounding box for each QR code (as a numpy array)
        bbox = np.int32(detections[i]["bbox_xyxy"])

        print (f"({bbox[0]}, {bbox[1]}), ({bbox[2]}, {bbox[3]})")

        cv2.rectangle (img, (bbox[0] - 1, bbox[1] - 1), (bbox[2] + 1, bbox[3] + 1), (0, 255, 0), 2)

        cv2.putText(img, qr, (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Image with QR Codes", img)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()