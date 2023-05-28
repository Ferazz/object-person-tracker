import ultralytics.yolo.engine.results
from ultralytics import YOLO
import cv2
from typing import List, Tuple
import math

def main():
    model = YOLO("/models/yolov8n.pt")
    # accepts all formats - image/dir/Path/URL/video/PIL/ndarray. 0 for webcam
    vid: cv2.VideoCapture = cv2.VideoCapture(0)

    while True:
        ret, frame = vid.read()
        if not ret:
            continue
        
        should_fire = False
        person_middle_points: List[Tuple[int, int]] = []
        control_errors: List[Tuple[int, int]] = []
        height, width = frame.shape[:2]

        results: List[ultralytics.yolo.engine.results.Results] = model.predict(source=frame, verbose=False)
        for result in results:
            # Results always have length 1?
            # Detection
            #print(result)
            # names is a dictionary with all classifiers and their id's 
            names = result.names
            boxes = result.boxes
            #print(boxes)
            if boxes:
                #print(boxes)
                for box, cls in zip(boxes.xyxy, boxes.cls):
                    # cls is tensor[0.] so needs to be turned into integer number for indexing
                    if names[int(cls)] == 'person':
                        person_middle_points.append(
                            (int(box[0] + box[2])//2, int(box[1] + box[3])//2)
                        )

            frame = result.plot(img=frame)

        center_coordinates = [width//2, height//2]
        radius = 30
        draw_reticle(frame, center_coordinates, radius)

        for person_mid_point in person_middle_points:
            if calc_distance(person_mid_point, center_coordinates) < radius:
                should_fire = True
            cv2.circle(frame, person_mid_point, 2, (0,0,0), 2)
            x_error = center_coordinates[0] - person_mid_point[0]
            y_error = center_coordinates[1] - person_mid_point[1]
            control_errors.append((x_error, y_error))
        
        error_offset = 30
        cv2.putText(frame, "Error", (width - 100, error_offset), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))

        for i in range(len(control_errors)):
            error_offset += 15
            cv2.putText(frame, f"x: {control_errors[i][0]} | y: {control_errors[i][1]}", (width - 140, error_offset), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))

        
        border_thickness = 8
        if should_fire:
            cv2.rectangle(frame, (0,0), (width, height), (0,255,0), border_thickness)
        else:
            cv2.rectangle(frame, (0,0), (width, height), (0,0,255), border_thickness)

        cv2.imshow('frame', frame)
        cv2.waitKey(1)


def draw_reticle(frame, center_coordinates: List[int], radius):
    cv2.circle(frame, center_coordinates, radius=radius, color=(0,0,0), thickness=1)

def calc_distance(pt1, pt2):
    x_dist = pow(pt1[0] - pt2[0], 2)
    y_dist = pow(pt1[1] - pt2[1], 2)
    return math.sqrt(x_dist + y_dist)

if __name__ == "__main__":
    main()