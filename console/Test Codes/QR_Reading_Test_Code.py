import cv2
import numpy as np
from qreader import QReader

cap = cv2.VideoCapture (1)

while (cap.isOpened ()):

    _, img = cap.read ();

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    qreader = QReader()

    decoded_text, detections = qreader.detect_and_decode(image=img, return_detections=True)

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