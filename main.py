import argparse
import math
import cv2
import numpy as np
import mediapipe as mp


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=int, default=1)

    args = parser.parse_args()

    return args


def dist_3d(punto1, punto2):
    accum = (punto1[0] - punto2[0])**2 + (punto1[1] - punto2[1])**2 + (punto1[2] - punto2[2])**2
    return math.sqrt(accum) # indice a pulgar


def obtener_marcadores(image, landmarks):
    """Dibuja marcadores en los dedos"""
    im_height, im_width, _ = image.shape
    marcadores = []
    for idx, landmark in enumerate(landmarks.landmark):
        if landmark.visibility < 0 or landmark.presence < 0:
            continue

        landmark_x = min(int(landmark.x * im_width), im_width - 1)
        landmark_y = min(int(landmark.y * im_height), im_height - 1)

        if idx in (8, 4):    # pulgar
            cv2.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            marcadores.append((landmark_x, landmark_y, landmark.z))

    return image, marcadores


def main():
    args = get_args()

    cap_device = args.device

    cap = cv2.VideoCapture(cap_device)

    mp_hands = mp.solutions.hands

    hands = mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5
    )

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        image = cv2.flip(frame, 1)
        results = hands.process(image)
        if results.multi_hand_landmarks is not None:
            for hand_landmarks in results.multi_hand_landmarks:
                image, marcadores = obtener_marcadores(image, hand_landmarks)
                if len(marcadores) == 2 and dist_3d(*marcadores) < 50:
                    print(dist_3d(*marcadores))
                    cv2.putText(image, "follow", (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow("hands", image)
        key = cv2.waitKey(1)
        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()