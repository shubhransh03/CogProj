import cv2
import os

save_dir = "dataset/images/train"
os.makedirs(save_dir, exist_ok=True)

cap = cv2.VideoCapture(0)

count = 0

print("Press SPACE to capture image")
print("Press Q to quit")

while True:

    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Capture Samples", frame)

    key = cv2.waitKey(1)

    if key == 32:   # SPACE
        filename = f"{save_dir}/img_{count}.jpg"
        cv2.imwrite(filename, frame)
        print("Saved", filename)
        count += 1

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()