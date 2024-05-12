import cv2
import time
from libcamera.camera import Camera
from libcamera.stream import Stream
from OpticalFlowShowcase import *

usage_text = '''
Hit followings to switch to:
1 - Dense optical flow by HSV color image (default);
2 - Dense optical flow by lines;
3 - Dense optical flow by warped image;
4 - Lucas-Kanade method.

Hit 's' to save image.

Hit 'f' to flip image horizontally.

Hit ESC to exit.
'''

def main():
    ## private routines
    def change(key, prevFrame):
        message, type = {
            ord('1'): ('==> Dense_by_hsv', 'dense_hsv'),
            ord('2'): ('==> Dense_by_lines', 'dense_lines'),
            ord('3'): ('==> Dense_by_warp', 'dense_warp'),
            ord('4'): ('==> Lucas-Kanade', 'lucas_kanade')
        }.get(key, ('==> Dense_by_hsv', 'dense_hsv'))
        print(message)
        of = CreateOpticalFlow(type)
        of.set1stFrame(prevFrame)
        return of
    
    ## main starts here
    flipImage = True
    of = None

    with Camera() as camera:
        camera.configure()
        camera.allocate_buffers(1)

        stream = camera.stream()

        stream.register_buffers(camera.buffers())
        stream.start()

        cv2.namedWindow("preview")

        ## main work
        for cameraFrame in stream:
            # get array & clear the stream in preparation for the next frame
            frame = cameraFrame.buffer.data
            stream.queue_buffer(cameraFrame)

            if of is None:
                of = change('1', frame)
                continue

            ### flip
            if flipImage:
                frame = cv2.flip(frame, 1)

            ### do it
            img = of.apply(frame)
            cv2.imshow("preview", img)

            ### key operation
            key = cv2.waitKey(1)
            if key == 27:         # exit on ESC
                print('Closing...')
                break
            elif key == ord('s'):   # save
                cv2.imwrite('img_raw.png', frame)
                cv2.imwrite('img_w_flow.png', img)
                print("Saved raw frame as 'img_raw.png' and displayed as 'img_w_flow.png'")
            elif key == ord('f'):   # save
                flipImage = not flipImage
                print("Flip image: " + {True:"ON", False:"OFF"}.get(flipImage))
            elif ord('1') <= key and key <= ord('4'):
                of = change(key, frame)

    ## finish
    cv2.destroyWindow("preview")

if __name__ == '__main__':
    print(usage_text)
    main()
