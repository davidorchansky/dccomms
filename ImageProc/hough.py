import cv2
import numpy as np


import time

print("hello")

img = cv2.imread('frame01176_r.png_100.pgm')

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

start = time.time()
gray2 = cv2.GaussianBlur(gray,(5,5),1,1)
#gray2 = cv2.blur(gray,(5,5),0)
end = time.time()
print("Noise reduction: " + str(end - start))

#gray3 = cv2.cvtColor(gray2,cv2.COLOR_BGR2GRAY)
start = time.time()
edges = cv2.Canny(gray2,40,120,apertureSize = 3)
end = time.time()
print("Canny: " + str(end - start))


lines = cv2.HoughLines(edges,1,np.pi/180,200)

for rho,theta in lines[0]:
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))

    cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

cv2.imwrite("no-noise.pgm",gray2)
cv2.imwrite("edges.pgm",edges)
cv2.imwrite('houghlines3.pgm',img)
