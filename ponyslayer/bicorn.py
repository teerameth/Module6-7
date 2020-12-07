import numpy as np
import cv2
from math import sqrt
from scipy.ndimage import generic_filter
import imutils
from skimage.morphology import skeletonize
def immask(c, frame):
    mask = np.ones(frame.shape[:2], dtype="uint8") * 255
    cv2.drawContours(mask, [c], 0, 0, -1)
    return 255 - mask
def implot(c, frame, mode = 0, size = 1):
    if mode: canvas = np.zeros_like(frame, np.uint8)
    else: canvas = frame.copy()
    for i in range(len(c)):
        center = (c[i][0][0], c[i][0][1])
        cv2.circle(canvas, center, size, (255, 255, 255), -1)
        cv2.imshow("Preview Plot", canvas)
        cv2.waitKey(100)
    return canvas
def implotline(c, frame, mode = 0, size = 5):
    if mode: canvas = np.zeros_like(frame, np.uint8)
    else: canvas = frame.copy()
    for i in range(len(c)):
        center = (c[i][0][0], c[i][0][1])
        if i != 0:
            cv2.line(canvas, (c[i-1][0][0], c[i-1][0][1]), (c[i][0][0], c[i][0][1]), (0, 255, 0), 2)
        cv2.circle(canvas, center, size, (255, 255, 255), -1)
        # cv2.imshow("Preview Plot", canvas)
        # cv2.waitKey(0)
    return canvas
def implotlineXY(points, frame, mode = 0, size = 5):
    if mode: canvas = np.zeros_like(frame, np.uint8)
    else: canvas = frame.copy()
    for i in range(len(points)):
        center = (int(points[i][0]), int(points[i][1]))
        if i != 0:
            cv2.line(canvas, (int(points[i-1][0]), int(points[i-1][1])), (int(points[i][0]), int(points[i][1])), (0, 255, 0), 2)
        cv2.circle(canvas, center, size, (255, 255, 255), -1)
        # cv2.imshow("Preview Plot", canvas)
        # cv2.waitKey(1)
    return canvas
# Line ends filter
def lineEnds(P):
    """Central pixel and just one other must be set to be a line end"""
    return 255 * ((P[4]==255) and np.sum(P)==510)
path_mask_opening = None
def euclidean(p1, p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
def medianCanny(img, thresh1, thresh2):
    median = np.median(img)
    img = cv2.Canny(img, int(thresh1 * median), int(thresh2 * median))
    return img
def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result
def padding_image(img, color):
    s = max(img.shape[0:2])
    f = np.ones((s, s, 3),np.uint8) * color
    ax,ay = (s - img.shape[1])//2,(s - img.shape[0])//2 # Getting the centering position
    f[ay:img.shape[0]+ay,ax:ax+img.shape[1]] = img
    return f
def padding_image_gray(img, color):
    s = max(img.shape[0:2])
    f = np.ones((s, s),np.uint8) * color
    ax,ay = (s - img.shape[1])//2,(s - img.shape[0])//2 # Getting the centering position
    f[ay:img.shape[0]+ay,ax:ax+img.shape[1]] = img
    return f
def findContourCenter(c):
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    rect = np.asarray(approx, dtype="float32")
    M = cv2.moments(c)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return (cX, cY)
def generate_trajectory3D(src, waypoints2D, min_height, max_height, gradient_crop_ratio, min_intensity_range, path_cnt_approx): # Use points to sample intensity
    if len(src.shape) == 3: src = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    src = cv2.GaussianBlur(src,(9,9),0)
    # cv2.imshow("ABC", src)
    # cv2.waitKey(0)
    waypoints3D = []
    intensity_buffer = []
    for (x, y) in waypoints2D: intensity_buffer.append(255 - src[y][x])
    height_range = max_height - min_height
    hist, _ = np.histogram(intensity_buffer, 256, [0, 256])
    cdf = hist.cumsum()
    # print(cdf)
    cdf_max = cdf[-1]
    cdf_thresh_min = int(cdf_max * (1 - gradient_crop_ratio) / 2)
    cdf_thresh_max = int(cdf_max * (1 + gradient_crop_ratio) / 2)
    sorted_intensity = sorted(intensity_buffer)
    intensity_thresh_min = sorted_intensity[cdf_thresh_min]
    intensity_thresh_max = sorted_intensity[cdf_thresh_max]
    intensity_range = intensity_thresh_max - intensity_thresh_min
    if intensity_range < min_intensity_range: z_buffer = [-1 for i in range(len(waypoints2D))] # This is not gradient (z=-1 -> Hold height)
    else:
        z_buffer = []
        for intensity in intensity_buffer:
            if intensity <= intensity_thresh_min: z_buffer.append(min_height)
            elif intensity >= intensity_thresh_max: z_buffer.append(max_height)
            else: z_buffer.append((intensity - intensity_thresh_min) * (height_range) / intensity_range + min_height)
    for i, (x, y) in enumerate(waypoints2D): waypoints3D.append((x/2, y/2, z_buffer[i]))
    waypoints3D_approx = []
    for point_approx in path_cnt_approx:
        best = waypoints3D[0]
        best_distance = euclidean((point_approx[0][0]/2, point_approx[0][1]/2), (best[0], best[1]))
        for candidate in waypoints3D:
            distance = euclidean((point_approx[0][0]/2, point_approx[0][1]/2), (candidate[0], candidate[1]))
            if distance < best_distance:
                best_distance = distance
                best = candidate
        waypoints3D_approx.append((point_approx[0][0]/2, point_approx[0][1]/2, best[2]))
    return waypoints3D_approx
def getPathMask(mask, startMask, chessMask):
    path_mask = cv2.bitwise_or(mask, chessMask)
    path_mask = cv2.bitwise_or(path_mask, startMask)
    path_mask = cv2.bitwise_not(path_mask)
    path_mask_opening = cv2.morphologyEx(path_mask, cv2.MORPH_OPEN, np.ones((15, 15)))
    # cv2.imshow("A", path_mask_opening)
    # cv2.waitKey(0)
    return path_mask_opening
def getPath(path_mask_opening, visualize=False):
    min_area = 20000
    contours, hierarchy = cv2.findContours(path_mask_opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    path_contours = []
    ## Filter noise out by area ##
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # print(area)
        poly_points = cv2.approxPolyDP(cnt, 0.03 * path_mask_opening.shape[0], False)  # approximate polygon
        m = immask(poly_points, np.zeros_like(path_mask_opening))
        # imshow(m)
        if area > min_area: # Pass minimum area
            path_contours.append(poly_points)
            cv2.imwrite("X:/pathMask" + str(len(path_contours)) + ".png", cv2.drawContours(np.zeros_like(path_mask_opening), [poly_points], -1, 255, -1))
    print("Path count: " + str(len(path_contours)))
    ## Skeletonize ##
    path_masks = [immask(cnt, path_mask_opening) for cnt in path_contours]
    path_skeletons = []
    canvas = np.zeros((800, 800, 3), dtype=np.uint8)
    for i in range(len(path_masks)):
        skeleton = skeletonize(path_masks[i], method='lee')
        path_skeletons.append(skeleton)
        if visualize:
            cnt = cv2.findContours(path_masks[i], cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
            cv2.drawContours(skeleton, cnt, -1, 100, 1)
            cv2.imshow("Skeleton", skeleton)
            cv2.waitKey(0)
    ## List 2D points from skeleton & Approximate ##
    # end_points = []
    path_cnts = []
    path_cnts_approx = []
    for skeleton in path_skeletons:
        points = np.column_stack(np.where(skeleton.transpose() != 0))  # get array of points
        end_of_line_canvas = generic_filter(skeleton, lineEnds, (3, 3))  # find end of line
        end_of_line_points = np.column_stack(np.where(end_of_line_canvas.transpose() != 0))  # get points from canvas
        # print(end_of_line_points)
        points = sortPoint(points, start=end_of_line_points[0])
        points_cnt = points.reshape((points.shape[0], 1,points.shape[1]))
        poly_points = cv2.approxPolyDP(points_cnt, 0.02 * path_mask_opening.shape[0], False)  # approximate polygon
        canvas = implotlineXY(points, canvas)
        if visualize:
            # cv2.imshow("END OF LINES", cv2.dilate(end_of_line, np.ones((3, 3)), iterations=10))
            # cv2.polylines(canvas, points, 0, (0, 255, 0), 1)
            cv2.imshow("Path Approx", canvas)
            cv2.waitKey(0)
        path_cnts.append(points)
        path_cnts_approx.append(poly_points)
    cv2.destroyAllWindows()
    cv2.imwrite("X:/generatePath.png", canvas)
    return path_cnts, path_cnts_approx, canvas
def getPath3D(src, path_cnts, path_cnts_approx, min_height=100, max_height=200, gradient_crop_ratio=0.8, min_intensity_range=15):
    ## Generate 3D Waypoints from Gradient ##
    waypoints3D_approx_list = []
    for i in range(len(path_cnts)):
        path_cnt = path_cnts[i]
        path_cnt_approx = path_cnts_approx[i]
        canvas = np.zeros((800, 800, 3))
        canvas = implotline(path_cnt_approx, canvas)
        waypoints3D_approx = generate_trajectory3D(src, path_cnt, min_height, max_height, gradient_crop_ratio, min_intensity_range, path_cnt_approx)
        waypoints3D_approx_list.append(waypoints3D_approx)
    # print(path_cnts_approx)
    # print(waypoints3D_approx_list)
    return waypoints3D_approx_list
def sortPoint(points, start):
    # Convert array to list (Be able to use remove())
    start = (start[0], start[1])
    point_list = []
    for point in points:
        point_list.append((point[0], point[1]))
    points = list(points)
    new_points = []
    best = {}
    present_point = start
    new_points.append(start)
    point_list.remove(start)
    while len(point_list):
        best['vector'] = point_list[0]
        best['cost'] = euclidean(present_point, best['vector'])
        for point in point_list:
            cost = euclidean(point, present_point)
            if best['cost'] > cost: # Update better value (Nearest to present point)
                best['vector'] = point
                best['cost'] = cost
        new_points.append(best['vector'])
        point_list.remove(best['vector'])
        present_point = best['vector']
    canvas = np.zeros((800, 800, 3))
    canvas = implotlineXY(new_points, canvas)
    return np.asarray(new_points, dtype=np.int32)
def solve_intersection(p1a, p1b, p2a, p2b):
    (x0, y0) = p1a
    (x1, y1) = p1b
    (x2, y2) = p2a
    (x3, y3) = p2b
    m1 = (y1-y0)/(x1-x0)
    m2 = (y3-y2)/(x3-x2)
    c1 = y0 - m1*x0
    c2 = y2 - m2*x2
    x = (c2-c1)/(m1-m2)
    y = m1*x + c1
    return (x, y)
def generateCommand(startMask, stopMask, path_cnts, waypoints3D_approx_list, hover_height=400):
    command_list = []
    startContour = cv2.findContours(startMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0][0]
    startCenter = findContourCenter(startContour)
    stopContour = cv2.findContours(stopMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0][0]
    stopCenter = findContourCenter(stopContour)
    if len(path_cnts) == 1: # One path -> simple
        path_cnt = path_cnts[0]
        distanceA = euclidean(path_cnt[0], startCenter)
        distanceB = euclidean(path_cnt[-1], stopCenter)
        distanceC = euclidean(path_cnt[-1], startCenter)
        distanceD = euclidean(path_cnt[0], stopCenter)
        if distanceA+distanceB < distanceC+distanceD: # path_cnt[0] is near startMarker, path_cnt[-1] is near stopMarker
            command_list.append((startCenter[0] / 2, startCenter[1] / 2, hover_height))
            command_list.append((startCenter[0]/2, startCenter[1]/2, waypoints3D_approx_list[0][0][2]))
            command_list += waypoints3D_approx_list[0]
            command_list.append((stopCenter[0]/2, stopCenter[1]/2, waypoints3D_approx_list[0][-1][2]))
            command_list.append((stopCenter[0] / 2, stopCenter[1] / 2, hover_height))
        else:
            command_list.append((startCenter[0] / 2, startCenter[1] / 2, hover_height))
            command_list.append((startCenter[0] / 2, startCenter[1] / 2, waypoints3D_approx_list[0][-1][2]))
            command_list += reversed(waypoints3D_approx_list[0])
            command_list.append((stopCenter[0] / 2, stopCenter[1] / 2, waypoints3D_approx_list[0][0][2]))
            command_list.append((stopCenter[0] / 2, stopCenter[1] / 2, hover_height))
        canvas = np.zeros((800, 800, 3))
        command_double = []
        for command in command_list[1:-1]:
            command_double.append((int(command[0]*2), int(command[1]*2), int(command[2]*2)))
            cv2.putText(canvas, str(int(command[2])), (int(command[0]*2), int(command[1]*2)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
        canvas = implotlineXY(command_double, canvas)
        return command_list, canvas
    if len(path_cnts) == 2: # Two path -> interpolate 2nd marker
        A, B = path_cnts[0], path_cnts[1]
        best = {'cost':99999999, 'command': None, 'marker_index': None}
        markerB_index = None
        for A_direction in [0, 1]: # 0 = Normal, 1 = reversed
            for B_direction in [0, 1]: # 0 = Normal, 1 = reversed
                for order in [0, 1]: # 0 = A -> B, 1 = B -> A
                    points = [startCenter]
                    command_list = []
                    command_list.append((startCenter[0] / 2, startCenter[1] / 2, hover_height))
                    if order == 0: # A first
                        if A_direction == 0:
                            points += [A[0], A[-1]]
                            command_list.append((startCenter[0] / 2, startCenter[1] / 2, waypoints3D_approx_list[0][0][2]))
                            command_list += waypoints3D_approx_list[0]
                        else:
                            points += [A[-1], A[0]]
                            command_list.append((startCenter[0] / 2, startCenter[1] / 2, waypoints3D_approx_list[0][-1][2]))
                            command_list += reversed(waypoints3D_approx_list[0])
                            markerB_index = len(command_list)
                        if B_direction == 0:
                            points += [B[0], B[-1]]
                            command_list += waypoints3D_approx_list[1]
                            command_list.append((stopCenter[0] / 2, stopCenter[1] / 2, waypoints3D_approx_list[1][-1][2]))
                        else:
                            points += [B[-1], B[0]]
                            command_list += reversed(waypoints3D_approx_list[1])
                            command_list.append((stopCenter[0] / 2, stopCenter[1] / 2, waypoints3D_approx_list[1][0][2]))
                    else: # B first
                        if B_direction == 0:
                            points += [B[0], B[-1]]
                            command_list.append((startCenter[0] / 2, startCenter[1] / 2, waypoints3D_approx_list[1][0][2]))
                            command_list += waypoints3D_approx_list[1]
                        else:
                            points += [B[-1], B[0]]
                            command_list.append((startCenter[0] / 2, startCenter[1] / 2, waypoints3D_approx_list[1][-1][2]))
                            command_list += reversed(waypoints3D_approx_list[1])
                        markerB_index = len(command_list)
                        if A_direction == 0:
                            points += [A[0], A[-1]]
                            command_list += waypoints3D_approx_list[0]
                            command_list.append((stopCenter[0] / 2, stopCenter[1] / 2, waypoints3D_approx_list[0][-1][2]))
                        else:
                            points += [A[-1], A[0]]
                            command_list += reversed(waypoints3D_approx_list[0])
                            command_list.append((stopCenter[0] / 2, stopCenter[1] / 2, waypoints3D_approx_list[0][0][2]))
                    command_list.append((stopCenter[0] / 2, stopCenter[1] / 2, hover_height))
                    points.append(stopCenter)
                    cost = euclidean(points[0], points[1]) + euclidean(points[2], points[3]) + euclidean(points[4], points[5])
                    if cost < best['cost']: # Update best value
                        best['cost'] = cost
                        best['command'] = command_list.copy()
                        best['marker_index'] = markerB_index
                        # canvas1 = np.zeros((800, 800, 3))
                        # canvas1 = implotlineXY(best['command'], canvas1)
                        # cv2.imshow("A", canvas1)
                        # cv2.waitKey(0)
        i = best['marker_index']
        middleMarker = solve_intersection((best['command'][i-2][0], best['command'][i-2][1]),
                                          (best['command'][i-1][0], best['command'][i-1][1]),
                                          (best['command'][i][0], best['command'][i][1]),
                                          (best['command'][i+1][0], best['command'][i+1][1]))
        best['command'].insert(i, (middleMarker[0], middleMarker[1], best['command'][i][2]))
        canvas = np.zeros((800, 800, 3))
        command_double = []
        for command in best['command'][1:-1]:
            command_double.append((int(command[0]*2), int(command[1]*2), int(command[2]*2)))
            cv2.putText(canvas, str(int(command[2])), (int(command[0]*2), int(command[1]*2)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))
        canvas = implotlineXY(command_double, canvas)
        return best['command'], canvas