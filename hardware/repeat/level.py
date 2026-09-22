import cv2, numpy as np, sys
cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")); cap.set(3, 1280); cap.set(4, 720)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1); cap.set(cv2.CAP_PROP_EXPOSURE, 5000); cap.set(cv2.CAP_PROP_GAIN, 100)
for _ in range(12): cap.read()
fr = [cap.read()[1].astype(np.float32) for _ in range(10)]
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3); cap.release()
m = np.mean(fr, 0); print(sys.argv[1], "mean level %.2f  p99 %.1f  frame-noise %.2f" % (m.mean(), np.percentile(m, 99), np.std(fr[0] - fr[1]) / 1.414))
