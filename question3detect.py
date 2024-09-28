import cv2
from ultralytics import YOLO

model = YOLO("ODLC.pt")

def findODLC(img_fn):
    image = cv2.imread(img_fn)
    results = model.predict(image)
    for result in results:
        #print("boxes", result.boxes)
        #print("names", result.names)
        result_cls = result.boxes.cls.tolist()
        #print(result_cls)
        shape_cls, letter_cls = result_cls[0], result_cls[1]
        shape = result.names[shape_cls]
        letter = result.names[letter_cls]
    return (shape, letter)

def main():
    path_to_img = "testimg/test4.png"
    print(findODLC(path_to_img))

if __name__ == "__main__":
    main()