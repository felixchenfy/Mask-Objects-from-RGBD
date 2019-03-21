
import cv2
import numpy as np

# filename="depth_image.png"
filename="data/016/depth/depth00001.png"
depth = cv2.imread(filename,cv2.IMREAD_UNCHANGED)
# print depth.astype(np.float)
# print depth.mean()
# cv2.imshow("rgb + depth", depth)
# depth_in_color = cv2.cvtColor((depth/20).astype(np.uint8),cv2.COLOR_GRAY2RGB)
# cv2.imshow("rgb + depth", depth_in_color)
cv2.waitKey()
cv2.destroyAllWindows()
