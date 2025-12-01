import os.path
import glob
import cv2
import numpy as np

class camera():

    def __init__(self,port):
        self.port = port

    def calibration(self, path):
        # Define number of inner corners per chessboard row and column
        nb_vertical = 5    # number of inner corners along the width
        nb_horizontal = 5  # number of inner corners along the height

        # prepare object points, like (0,0,0), (1,0,0) ....,(nb_vertical-1, nb_horizontal-1, 0)
        objp = np.zeros((nb_horizontal*nb_vertical, 3), np.float32)
        objp[:, :2] = np.mgrid[0:nb_vertical, 0:nb_horizontal].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images
        objpoints = []  # 3D points in real world space
        imgpoints = []  # 2D points in image plane

        images = glob.glob(os.path.join(path, "*.png"))
        assert images

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (nb_vertical, nb_horizontal), None)

            # If found, add object points and refined image points
            if ret:
                objpoints.append(objp)

                # Refine corner positions for sub-pixel accuracy
                corners2 = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (nb_vertical, nb_horizontal), corners2, ret)
                cv2.imshow('img', img)
                cv2.waitKey(500)

        cv2.destroyAllWindows()
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        imgs = cv2.imread(os.path.join(path, 'board_frame_17.png'))
        h,  w = imgs.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        return newcameramtx


    def capture_board(self, path):  

        cam = cv2.VideoCapture(self.port)
        cv2.namedWindow("test")
        img_counter = 0

        while True:
            ret, frame = cam.read()
            if not ret:
                print("failed to grab frame")
                break
            cv2.imshow("test", frame)

            k = cv2.waitKey(1)
            if k%256 == 27:
                # ESC pressed
                print("Escape hit, closing...")
                break
            elif k%256 == 32:
                # SPACE pressed
                img_name = "board_frame_{}.png".format(img_counter)
                cv2.imwrite(os.path.join(path, img_name), frame)
                print("{} captured".format(img_name))
                img_counter += 1

        cam.release()

        cv2.destroyAllWindows()

if __name__ == "__main__":
    # 4 for robot (for me obviously)
    port = 4
    image_path= '/home/adam/Desktop/Robotics/Final_Project/Chess_board_imgs'
    cam = camera(port)
    #cam.capture_board(image_path)
    cam_matrix = cam.calibration(image_path)
    print(cam_matrix)