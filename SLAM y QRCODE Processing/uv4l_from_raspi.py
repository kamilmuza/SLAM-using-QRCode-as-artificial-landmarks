__author__ = 'admin'

import cv2
import time
stream_web = "http://192.168.112.100:8080/stream/video.mjpeg"
i = 0


def main():
#    output = open("\plot\SLAMQR\sensor.dat",'w')
    capture = cv2.VideoCapture(0)
    #capture.open(stream_web)
    print "Preparing Camera"
    i = 0
    first = time.time()
    #cv2.namedWindow("Pictures")
    while i < 350:                
        (grabbed, img) = capture.read()
        time.sleep(0.2)
        #cv2.imshow("Pictures", img)        
        timed = time.clock()
        path = 'Pic%.5f.png' %timed
        #if i > 300:
        #    pics_array2.append(img)
        #else:
        pics_array.append(img)    
        #a = take_photo(path, img)
        i = i + 1
        print "Sample number %d"%i
    end = time.time()- first    
    capture.release()
    cv2.destroyAllWindows()
    print "Process Finish in %d seconds"%end
    for a in range(350):
        # if a > 300:
            # pic = pics_array2[a]
        # else:
        pic = pics_array[a]  

        print 'Copying sample number: %i to rpi SD CARD'%a        
        cv2.imwrite("Pic%03d.png"%a, pic)


if __name__ == "__main__":
    
    pics_array= []
    pics_array2= []
    main()
