import cv2
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import math
def segment_by_angle_kmeans(lines, k=2, **kwargs):
    # Define criteria = (type, max_iter, epsilon)
    default_criteria_type = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER
    criteria = kwargs.get('criteria', (default_criteria_type, 10, 1.0))
    flags = kwargs.get('flags', cv2.KMEANS_RANDOM_CENTERS)
    attempts = kwargs.get('attempts', 10)
    angles = np.array([line[0][1] for line in lines])
    
    # multiply the angles by two and find coordinates of that angle
    pts = np.array([[np.cos(2*angle), np.sin(2*angle)]
                    for angle in angles], dtype=np.float32)

    # run kmeans on the coords
    labels, centers = cv2.kmeans(pts, k, None, criteria, attempts, flags)[1:]
    labels = labels.reshape(-1)  # transpose to row vec

    # segment lines based on their kmeans label
    segmented = defaultdict(list)
    for i, line in zip(range(len(lines)), lines):
        segmented[labels[i]].append(line)
    segmented = list(segmented.values())
    return segmented

def intersection(line1, line2):
    rho1, theta1 = line1[0]
    rho2, theta2 = line2[0]
    A = np.array([
        [np.cos(theta1), np.sin(theta1)],
        [np.cos(theta2), np.sin(theta2)]
    ])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    x0, y0 = int(np.round(x0)), int(np.round(y0))
    return [[x0, y0]]


def segmented_intersections(lines):
    intersections = []
    for i, group in enumerate(lines[:-1]):
        for next_group in lines[i+1:]:
            for line1 in group:
                for line2 in next_group:
                    intersections.append(intersection(line1, line2)) 

    return intersections

def brick_boi(image):
    x_offset = 0.045
    y_offset = 0.022
    Angular_offset = 0
    filename = '%s.jpg' %(image)
    img = cv2.imread(filename)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 5)
    adapt_type = cv2.ADAPTIVE_THRESH_GAUSSIAN_C
    thresh_type = cv2.THRESH_BINARY_INV
    bin_img = cv2.adaptiveThreshold(blur, 255, adapt_type, thresh_type, 11, 2)
    rho, theta, thresh = 1, np.pi/180, 350
    lines = cv2.HoughLines(bin_img, rho, theta, thresh)
    segmented = segment_by_angle_kmeans(lines)
    intersections = segmented_intersections(segmented)
    for i in range (0,len(intersections)):
        array = np.array(intersections[i])
        plt.plot(array[0,0], array[0,1], 'bo')
    newintersections = []
    for i in range (0,len(intersections)):
        newintersections.append(intersections[i][0])
    kmeans = KMeans(n_clusters=4, random_state=0).fit(newintersections)
    centers4=kmeans.cluster_centers_
    centers=centers4[0:3]
    line1 = [centers[0,0]-centers[1,0], centers[0,1]-centers[1,1]]
    line2 = [centers[0,0]-centers[2,0], centers[0,1]-centers[2,1]]
    line3 = [centers[1,0]-centers[2,0], centers[1,1]-centers[2,1]]
    line1len = math.sqrt((line1[0]**2)+(line1[1]**2))
    line2len = math.sqrt((line2[0]**2)+(line2[1]**2))
    line3len = math.sqrt((line3[0]**2)+(line3[1]**2))
    linelen = [line1len, line2len, line3len]
   
    values = [0,1,2]
    values.remove(linelen.index(max(linelen)))
    values.remove(linelen.index(min(linelen)))
    if (values[0] == 0):
        shortestline = [centers[0,0]-centers[1,0], centers[0,1]-centers[1,1]]
        angle = np.arctan2(shortestline[0], shortestline[1])
        scalefactor = 0.2/line1len
        
    if (values[0] == 1):
        shortestline = [centers[0,0]-centers[2,0], centers[0,1]-centers[2,1]]
        angle = np.arctan2(shortestline[0], shortestline[1])
        scalefactor = 0.2/line2len
    
    if (values[0] == 2):
        shortestline = [centers[1,0]-centers[2,0], centers[1,1]-centers[2,1]]
        angle = np.arctan2(shortestline[0], shortestline[1])
        scalefactor = 0.2/line3len
        
    if (angle >= math.pi/2):
        angle = angle - math.pi
        
    if (angle <= -math.pi/2):
        angle = angle + math.pi
    averagecenters = [0,0]
    angle = -angle + Angular_offset
    
    averagecenters[0] = (centers4[0,0] + centers4[1,0]+ centers4[2,0]+ centers4[3,0])/4
    averagecenters[1] = (centers4[0,1] + centers4[1,1]+ centers4[2,1]+ centers4[3,1])/4
    
    yerror = y_offset -(scalefactor * (averagecenters[0]- img.shape[1]/2))
    xerror = x_offset + scalefactor * (averagecenters[1]- img.shape[0]/2)
    plt.plot(averagecenters[0],averagecenters[1], 'ro')
    plt.plot(img.shape[1]/2,img.shape[0]/2, 'ro')
    
    for i in range (0, len(centers)):
        plt.plot(centers[i][0],centers[i][1], 'ro')
    plt.imshow(img)
    
    cv2.waitKey(0)
    return(xerror, yerror, angle)


