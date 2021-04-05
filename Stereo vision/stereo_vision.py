#importing all 
import numpy as np
import cv2
from numpy.linalg import inv
f=36
b=96
cont_x=[]
cont_y=[]
k1=np.array([[699.58,0,300.192],[0,703.98,189.777],[0,0,1]])
m1=inv(k1)
k2=np.array([[699.58,0,300.192],[0,703.98,189.777],[0,0,1]])
m2=inv(k2)
start=8
end=42
re_start=0.2
re_end=1
from scipy.interpolate import interp1d
cv2.namedWindow('image1')
cv2.namedWindow('image2')



def nothing():
    pass




# Creating all Tackbars
cv2.createTrackbar('Low_L','image1',0,255,nothing)
cv2.createTrackbar('Low_A','image1',0,255,nothing)
cv2.createTrackbar('Low_B','image1',0,255,nothing)

# # Starting a video capture from the ip web cam
capture1=cv2.VideoCapture(2)
capture2=cv2.VideoCapture(3) 

capture1.set(cv2.CAP_PROP_AUTOFOCUS, 0)
capture2.set(cv2.CAP_PROP_AUTOFOCUS, 0)

# In an infinite loop we repeatedly sends the request 
# according to position of ball in the frame

while True:
    # Reading image from caputre 
    ret1,image1=capture1.read()
    ret2,image2=capture2.read()
    blr1=cv2.GaussianBlur(image1,(21,21),5)
    blr2=cv2.GaussianBlur(image2,(21,21),5)
    # (21,21) is the size of the gaussian kernel

    # Converting from BGR to LAB 
    lab_image1=cv2.cvtColor(blr1,cv2.COLOR_BGR2LAB)
    lab_image2=cv2.cvtColor(blr2,cv2.COLOR_BGR2LAB)
    # Masking the image to get only the ball
    # Getting lower value
    lower=np.asarray([cv2.getTrackbarPos('Low_L','image1'),cv2.getTrackbarPos('Low_A','image1'),cv2.getTrackbarPos('Low_B','image1')],dtype="uint8")
    # lower=np.asarray([122,141,92],dtype="uint8")
    upper=np.asarray([cv2.getTrackbarPos('High_L','image1'),cv2.getTrackbarPos('High_A','image1'),cv2.getTrackbarPos('High_B','image1')],dtype="uint8")
    # upper=np.asarray([255,255,255],dtype="uint8")
    masked_image1=cv2.inRange(lab_image1,lower,upper)
    masked_image2=cv2.inRange(lab_image2,lower,upper)
    
#     # gray1= cv2.cvtColor(masked_image1,cv2.COLOR_BGR2GRAY)
#     # gray2= cv2.cvtColor(masked_image2,cv2.COLOR_BGR2GRAY)


    sift = cv2.xfeatures2d.SIFT_create()
    kp1, des1 = sift.detectAndCompute(masked_image1,None)
    kp2, des2 = sift.detectAndCompute(masked_image2,None)
    
     

# BFMatcher with default params
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1,des2,k=2)
# Apply ratio test
    good = []
    for m, n in matches:
            img2_idx = m.trainIdx
            img1_idx = m.queryIdx
            (x1, y1) = kp1[img1_idx].pt
            (x2, y2) = kp2[img2_idx].pt
            if (abs(y1-y2)<3):

                print("x1",x1)
                print("y1",y1)
                print("x2",x2)
                print("y2",y2)
                im1=np.array([[x1],[y1],[1]])
                im2=np.array([[x2],[y2],[1]])
                w1=np.matmul(m1, im1)
                w2=np.matmul(m2, im2)
                print("im1:",im1)
                print("im2:",im2)
                disparity=abs(w1[0]-w2[0])


                print("w1:",w1)  
                print("w2:",w2) 
          
         
                print("disparity",disparity)
                distance=(36*b)/disparity
                distance=distance/1000
                print("distance",distance)




                



                # print("des1",des1[0,0])
                # print("des2",des2[0,0])
                # print(des2[10])
                
                #print("disparity:",disparity) 

                if m.distance < 0.9*n.distance:
                 good.append([m])
                 # try:
                 #     if 8<distance<=42:
                 #      g = interp1d([start,end],[re_start,re_end])
                 #      print("corrected",g(distance))

                 #     elif 42<=distance<=94:
                 #      start=43
                 #      end=126
                 #      re_start=1
                 #      re_end=2
                 #      print("corrected",g(distance))

                 #     elif 94<=distance<=135:
                 #      start=94
                 #      end=135
                 #      re_start=2
                 #      re_end=3
                 #      g = interp1d([start,end],[re_start,re_end])
                 #      print("corrected",g(distance)) 
                 #     else :
                 #       print("out_of_range") 
                 # except IndexError:
                 #  print('hl')        
             # print("x1_axis:",x1)  
                
# # cv.drawMatchesKnn expects list of lists as matches.
                
                # img3 = cv2.drawMatchesKnn(image1,kp1,image2,kp2,good,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                # cv2.imshow("img3",img3)
# #                # print(img3.shape)
            



#     lineThickness = 2
#     # cv2.line(image1, (0, 240), (640, 240), (0,255,0), lineThickness)
#     # cv2.line(image1, (320, 0), (320, 480), (0,255,0), lineThickness)
#     # cv2.line(image2, (0, 240), (640, 240), (0,255,0), lineThickness)
#     # cv2.line(image2, (320, 0), (320, 480), (0,255,0), lineThickness)
#     # cv2.line(masked_image1, (0, 240), (640, 240), (255,255,255), lineThickness)
#     # cv2.line(masked_image1, (320, 0), (320, 480), (255,255,255), lineThickness)
#     # cv2.line(masked_image2, (0, 240), (640, 240), (255,255,255), lineThickness)
#     # cv2.line(masked_image2, (320, 0), (320, 480),(255,255,255), lineThickness)
        
    cv2.imshow("image1",masked_image1)
    cv2.imshow("image2",masked_image2)
    

#     # cv2.imshow("image1_masked",masked_image1)
#     # cv2.imshow("image2_masked",masked_image2)


    # Press q to quit
    if cv2.waitKey(1)==ord("q"):
        break
