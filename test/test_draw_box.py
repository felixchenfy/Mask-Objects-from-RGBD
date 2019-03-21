
import cv2
import numpy as np


def drawLineToImage(img_display, p0, p1, color=None, line_width=2):
    if color is None:
        color=[0,0,255] # red
    if type(p0)==list:
        p0=(p0[0],p0[1])
    if type(p1)==list:
        p1=(p1[0],p1[1])
    cv2.line(img_display,p0,p1,color,line_width)

def drawBoxToImage(img_display, p0, p1, color=None, line_width=2):
    x1=p0[0]
    y1=p0[1]
    x2=p1[0]
    y2=p1[1]
    xs=[x1,x2,x2,x1,x1]
    ys=[y1,y1,y2,y2,y1]

    colors=['r','g','b']
    colors_dict={'b':[255,0,0],'g':[0,255,0],'r':[0,0,255]}
    if color==None:
        color=colors_dict['r']
    if type(color)!=list:
        color=colors_dict[color]

    for i in range(4):
        drawLineToImage(img_display, (xs[i],ys[i]),  (xs[i+1],ys[i+1]), color, line_width=2)

def drawDotToImage(img_display, uv, color=None, radius=1):
    x=int(uv[0])
    y=int(uv[1])
    # for i in range(-radius,radius+1):
        # for j in range(-radius,radius+1):
    img_display[x-5:x+5,y-5:y+5]=[255,0,0]
    
def showImg(I):
    cv2.imshow('img', I)
    cv2.waitKey()
    cv2.destroyAllWindows()

file="/home/feiyu/baxterws/src/winter_prj/mask_objects_from_rgbd/data/image00001.png"
img = cv2.imread(file,cv2.IMREAD_UNCHANGED)
drawBoxToImage(img, [50, 50], [200, 100], color='b')
img[0,0]=np.array([0,0,0])
drawDotToImage(img, [100,300])
showImg(img)