import numpy as np
import cv2
from math import sqrt
import imutils
from .transform import four_point_transform

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
def rotate_image_white(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    mask = np.ones_like(image)*255
    mask = cv2.warpAffine(mask, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    mask = cv2.bitwise_not(mask)
    result = cv2.bitwise_or(mask, result)
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

class ContourProcessor():
    def set_param(self, image_resolution, image_size, marker_size, marker_size_error, min_path_length, max_path_length, min_path_width, max_path_width):
        self.image_resolution = image_resolution
        self.image_size = image_size
        self.marker_size = marker_size
        self.marker_size_error = marker_size_error
        self.min_path_length = min_path_length
        self.max_path_length = max_path_length
        self.min_path_width = min_path_width
        self.max_path_width = max_path_width
        self.marker_size_pixel = image_resolution / image_size * marker_size
        self.marker_area = self.marker_size_pixel**2
    def get_center(self, c):
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        rect = np.asarray(approx, dtype="float32")
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)
    def is_contour_rect(self, c): # return True if contour is rectangle
        peri = cv2.arcLength(c, True) # approximate the contour
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        return len(approx) == 4
    def is_contour_marker(self, c):
        if not self.is_contour_rect(c): return False
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        actual_area = cv2.contourArea(approx)
        if actual_area == 0: return False # Avoid divided by zero
        expected_area = (self.marker_size/self.image_size*self.image_resolution)**2
        error = abs(expected_area - actual_area) / actual_area
        if error > self.marker_size_error: return False
        match = 100 * (1 - error)
        # rect = cv2.boundingRect(approx)
        # s_area = rect[2] * rect[3]
        return match
    def get_marker_contours(self, cnts):
        buffer = []
        trash = []
        for i, c in enumerate(cnts):
            match = self.is_contour_marker(c)
            if match:
                dup = False
                for passed in buffer:  # exclude duplicated rect
                    if cv2.pointPolygonTest(passed, self.get_center(c), False) != -1:  # Check if point inside contour
                        dup = True
                if not dup: # If not duplicate rect
                    peri = cv2.arcLength(c, True)
                    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
                    buffer.append(np.array(approx, dtype=np.int32))
                    print("Match {}%".format(round(match, 2)))
            else: trash.append(c)
        return buffer, trash
    def get_path_contours(self, cnts, Markers, frame):
        buffer = []
        areas = []
        for c in cnts:
            ########################################################################################
            ### Minimum area of path contour limited at minimum path width * minimum path length ###
            ########################################################################################
            area = cv2.contourArea(c)
            if area < (self.image_resolution / self.image_size * self.min_path_width)*(self.image_resolution / self.image_size * self.min_path_length): continue
            ###################################################################################
            ### Path needed to be near a least one Marker (Near = not far than marker size) ###
            ###################################################################################
            distanceMin = 99999999
            for Marker in Markers:
                candidate_distance = abs(cv2.pointPolygonTest(c, Marker.center, True))
                if candidate_distance < distanceMin: distanceMin = candidate_distance
            if distanceMin > (self.image_resolution / self.image_size * self.marker_size): continue
            #################################################################
            ### Path must not overlap Marker more than 50% of Marker area ###
            #################################################################
            canvas = np.zeros(frame.shape[:2], dtype="uint8")
            mask_path = cv2.drawContours(canvas.copy(), [c], 0, 255, -1)
            _isMarker = False
            for Marker in Markers:
                mask_marker = cv2.drawContours(canvas.copy(), [Marker.contour], 0, 255, -1)
                intersection = np.logical_and(mask_path, mask_marker) # mask of intersection
                intersect_count = np.count_nonzero(intersection == True) # intersection count
                if intersect_count > (self.marker_size_pixel**2) / 2: _isMarker = True # if intersection is more than 50% of marker area
            if _isMarker: continue

            buffer.append(c)
            areas.append(area)
        ############################################################################################
        ### Number of Path need to match with number of Marker -> Choose only N fist largest area###
        ############################################################################################
        buffer = [x for _, x in sorted(zip(areas, buffer), reverse=True)]
        buffer = buffer[:len(Markers)] # Choose only big contours
        # bestMatch = 9999
        # for Marker in Markers:
        #     candidate_match = cv2.matchShapes(Marker.contour, c, 1, 0)# Lower = better match
        #     if bestMatch > candidate_match:
        #         bestMatch = candidate_match
        # if bestMatch < 1: continue
        return buffer

    def findContourCenter(self, c):
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        self.rect = np.asarray(approx, dtype="float32")
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)
    def minDistance(self, cnt1, cnt2): # Find minimum distance between 2 contours
        distanceMin = 99999999
        point = None
        for A in cnt1:
            for x, y in A:
                distance = abs(cv2.pointPolygonTest(cnt2, (x, y), True))
                if (distance < distanceMin):
                    distanceMin = distance
                    point = (x, y)
        return distanceMin, point
    def getPointArray(self, c):
        buffer = []
        for item in c:
            buffer.append((item[0][0], item[0][1]))
        return np.array(buffer, dtype = "float32")
    def nearestPoint(self, point, c):
        distanceMin = 99999999
        best = None
        for item in c:
            candidate = item[0]
            distance = euclidean(candidate, point)
            if distanceMin > distance:
                best = candidate
                distanceMin = distance
        return tuple(best)
    def findIndex(self, c, point):
        array = self.getPointArray(c)
        for i in range(len(array)):
            if point[0] == array[i][0] and point[1] == array[i][1]: return i
        return -1
    def findTemplate(self, image, template_raw):
        template_original = cv2.cvtColor(template_raw, cv2.COLOR_BGR2GRAY)
        template_original = cv2.Canny(template_original, 50, 200)
        (tH, tW) = template_original.shape[:2]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found = None
        for rotation in range(0, 359, 5):
            resized = image
            template = rotate_image(template_original, rotation)
            r = gray.shape[1] / float(resized.shape[1])
            edged = cv2.Canny(resized, 50, 200)
            result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
            (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
            if visualize:
                clone = np.dstack([edged, edged, edged])
                cv2.rectangle(clone, (maxLoc[0], maxLoc[1]), (maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2)
                cv2.imshow("Template", template)
                cv2.imshow("Visualize", clone)
                cv2.waitKey(10)
            if found is None or maxVal > found[0]:
                found = (maxVal, maxLoc, r)
        (_, maxLoc, r) = found
        (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
        (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))
        cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2) # draw a bounding box around the detected result and display the image
        # imshow(image)
        return ((startX, startY), (endX, endY))
    def findTemplateMarker(self, imageSet_raw, template_raw):
        imageSet_gray = [cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY) for img in imageSet_raw]
        imageSet_canny = [cv2.Canny(img, 50, 200) for img in imageSet_gray]
        template_gray = cv2.cvtColor(template_raw.copy(), cv2.COLOR_BGR2GRAY)
        template_canny = cv2.Canny(template_gray, 50, 200)
        (tH, tW) = template_canny.shape[:2]
        ### Padding ###
        imageSet = [padding_image(img) for img in imageSet_canny]
        best = {"index":None, "score":0}
        for i in range(len(imageSet)): best[str(i)] = 0
        for rotation in range(0, 359, step):
            rotated_template = rotate_image(template.copy(), rotation)
            for i in range(len(imageSet)):
                result = cv2.matchTemplate(imageSet[i], template, cv2.TM_CCOEFF) # Get score (for cv2.TM_CCOEFF more mean better)
                (_, score, _, _) = cv2.minMaxLoc(result)
                if score > best["score"]:
                    best["index"] = i
                    best["score"] = score
                if score > best[str(i)]: best[str(i)] = score
        print(best)
        best_index = int(best["index"])
        best_score = best[str(best_index)]
        return(best["index"])
    def findChessBoard(self, imageRaw):
        img = imutils.resize(imageRaw, height=400)
        

# template = padding_image(template_canny)
_cp = ContourProcessor()

class Marker():
    def __init__(self, c):
        self.contour = c
        self.center = _cp.findContourCenter(c)
        self.bbox = cv2.boundingRect(self.contour)
    def getImage(self, img):
        return four_point_transform(img, _cp.getPointArray(self.contour))

class Path():
    def __init__(self, c):
        self.contour = c
        self.waypoints = None
        self.duo_list = None
        self.waypoints3D = None
    def set_start(self, point):
        self.start = point
    def generate_trajectory2D(self, Markers, marker_size_pixel, marker_size_error, step_size = 5):
        start_index = _cp.findIndex(self.contour, self.start)
        ## Reorder array
        array = _cp.getPointArray(self.contour)
        points = array.tolist()
        points = points[start_index:] + points[0:start_index]
        counter1 = 0
        counter2 = len(points) - 1
        duo_list = []
        middle = []
        while counter2 > counter1:
            best = {'index':counter2, 'cost':99999}
            near_marker = False
            for marker in Markers:
                distance = -cv2.pointPolygonTest(marker.contour, (points[counter1][0], points[counter1][1]), measureDist=True)
                if distance < marker_size_pixel*marker_size_error: near_marker = True
            if near_marker: pass
            else:
                for i in range(counter1 + int((counter2 - counter1)/2), counter2):
                    near_marker = False
                    for marker in Markers:
                        distance = -cv2.pointPolygonTest(marker.contour, (points[i][0], points[i][1]), measureDist=True)
                        if distance < marker_size_pixel * marker_size_error: near_marker = True
                    candidate = {'index': i, 'cost': euclidean(points[i], points[counter1])}
                    if candidate['cost'] < best['cost'] and not near_marker: best = candidate
                point1 = (int(points[counter1][0]), int(points[counter1][1]))
                point2 = (int(points[best['index']][0]), int(points[best['index']][1]))
                duo_list.append([point1, point2])
                middle.append((int((point1[0]+point2[0])/2), int((point1[1]+point2[1])/2)))
                counter2 = best['index']
            counter1 += step_size
        return duo_list[:-1], middle[:-1]
    def generate_trajectory2D_rev2(self, Markers, marker_size_pixel, marker_size_error, step_size = 5):
        start_index = _cp.findIndex(self.contour, self.start)
        duo_list = []
        middle = []
        ## Reorder array
        array = _cp.getPointArray(self.contour)
        points = array.tolist()
        points = points[start_index:] + points[0:start_index]
        counter1 = 0
        counter2 = len(points) - 1
        while counter2 > counter1:
            best = {'index':counter2, 'cost':99999}
            near_marker = False
            for marker in Markers:
                distance = -cv2.pointPolygonTest(marker.contour, (points[counter1][0], points[counter1][1]), measureDist=True)
                if distance < marker_size_pixel*marker_size_error: near_marker = True
            if near_marker: pass
            else:
                for i in range(counter1 + int((counter2 - counter1)/2), counter2):
                    near_marker = False
                    for marker in Markers:
                        distance = -cv2.pointPolygonTest(marker.contour, (points[i][0], points[i][1]), measureDist=True)
                        if distance < marker_size_pixel * marker_size_error: near_marker = True
                    candidate = {'index': i, 'cost': euclidean(points[i], points[counter1])}
                    if candidate['cost'] < best['cost'] and not near_marker: best = candidate
                point1 = (int(points[counter1][0]), int(points[counter1][1]))
                point2 = (int(points[best['index']][0]), int(points[best['index']][1]))
                duo_list.append([point1, point2])
                middle.append((int((point1[0]+point2[0])/2), int((point1[1]+point2[1])/2)))
                counter2 = best['index']
            counter1 += step_size
        ## Flip array
        points.reverse()
        counter1 = 0
        counter2 = len(points) - 1
        while counter2 > counter1:
            best = {'index': counter2, 'cost': 99999}
            near_marker = False
            for marker in Markers:
                distance = -cv2.pointPolygonTest(marker.contour, (points[counter1][0], points[counter1][1]),
                                                 measureDist=True)
                if distance < marker_size_pixel * marker_size_error: near_marker = True
            if near_marker:
                pass
            else:
                for i in range(counter1 + int((counter2 - counter1) / 2), counter2):
                    near_marker = False
                    for marker in Markers:
                        distance = -cv2.pointPolygonTest(marker.contour, (points[i][0], points[i][1]), measureDist=True)
                        if distance < marker_size_pixel * marker_size_error: near_marker = True
                    candidate = {'index': i, 'cost': euclidean(points[i], points[counter1])}
                    if candidate['cost'] < best['cost'] and not near_marker: best = candidate
                point1 = (int(points[counter1][0]), int(points[counter1][1]))
                point2 = (int(points[best['index']][0]), int(points[best['index']][1]))
                duo_list.append([point1, point2])
                middle.append((int((point2[0] + point1[0]) / 2), int((point1[1] + point2[1]) / 2)))
                counter2 = best['index']
            counter1 += step_size
        return duo_list[:-1], middle[:-1]
    def generate_trajectory2D_rev3(self, Markers, marker_size_pixel, marker_size_error, step_size = 5):
        start_index = _cp.findIndex(self.contour, self.start)
        duo_list = []
        middle = []
        distance_list = []
        ## Reorder array
        array = _cp.getPointArray(self.contour)
        points = array.tolist()
        points = points[start_index:] + points[0:start_index]
        counter1 = 0
        counter2 = len(points) - 1
        while counter2 > counter1:
            best = {'index':counter2, 'cost':99999}
            near_marker = False
            for marker in Markers:
                distance = -cv2.pointPolygonTest(marker.contour, (points[counter1][0], points[counter1][1]), measureDist=True)
                if distance < marker_size_pixel*marker_size_error: near_marker = True
            if near_marker: pass
            else:
                for i in range(counter1 + int((counter2 - counter1)/2), counter2):
                    near_marker = False
                    for marker in Markers:
                        distance = -cv2.pointPolygonTest(marker.contour, (points[i][0], points[i][1]), measureDist=True)
                        if distance < marker_size_pixel * marker_size_error: near_marker = True
                    candidate = {'index': i, 'cost': euclidean(points[i], points[counter1])}
                    if candidate['cost'] < best['cost'] and not near_marker: best = candidate
                point1 = (int(points[counter1][0]), int(points[counter1][1]))
                point2 = (int(points[best['index']][0]), int(points[best['index']][1]))
                duo_list.append([point1, point2])
                middle.append((int((point2[0]+point1[0])/2), int((point1[1]+point2[1])/2)))
                distance_list.append(best['cost'])
                counter2 = best['index']
            counter1 += step_size
        # Sample middle of path to get path width (assume pixel = distance)
        path_width = distance_list[int(len(distance_list)/2)] / step_size
        path_width = int(path_width/2) # ?????????????????
        if path_width == 0: path_width = 1 # Avoid [:0] happen
        self.waypoints = middle[:-path_width]
        self.duo_list = duo_list[:-path_width]
        return self.duo_list, self.waypoints
    def generate_trajectory3D(self, src, min_height, max_height, gradient_crop_ratio, min_intensity_range): # Use points to sample intensity
        if self.waypoints == None or self.duo_list == None:
            print("Generate 2D trajectory first!")
            return 0
        self.waypoints3D = []
        intensity_buffer = []
        for (x, y) in self.waypoints: intensity_buffer.append(src[y][x])
        height_range = max_height - min_height
        hist, bins = np.histogram(intensity_buffer, 256, [0, 256])
        cdf = hist.cumsum()
        #     print(cdf)
        cdf_max = cdf[-1]
        cdf_thresh_min = int(cdf_max * (1 - gradient_crop_ratio) / 2)
        cdf_thresh_max = int(cdf_max * (1 + gradient_crop_ratio) / 2)
        sorted_intensity = sorted(intensity_buffer)
        intensity_thresh_min = sorted_intensity[cdf_thresh_min]
        intensity_thresh_max = sorted_intensity[cdf_thresh_max]
        intensity_range = intensity_thresh_max - intensity_thresh_min
        if intensity_range < min_intensity_range: z_buffer = [-1 for i in range(len(self.waypoints))] # This is not gradient (z=-1 -> Hold height)
        else:
            z_buffer = []
            for intensity in intensity_buffer:
                if intensity <= intensity_thresh_min: z_buffer.append(min_height)
                elif intensity >= intensity_thresh_max: z_buffer.append(max_height)
                else: z_buffer.append((intensity - intensity_thresh_min) * (height_range) / intensity_range + min_height)
        for i, (x, y) in enumerate(self.waypoints): self.waypoints3D.append((x, y, z_buffer[i]))
        return self.waypoints3D
    def generate_trajectory3D_rev2(self, src, min_height, max_height, gradient_crop_ratio, min_intensity_range): # Use points to sample intensity
        self.waypoints3D = []
        intensity_buffer = []
        for (x, y) in self.waypoints: intensity_buffer.append(src[y][x])
        height_range = max_height - min_height
        hist, bins = np.histogram(intensity_buffer, 256, [0, 256])
        cdf = hist.cumsum()
        #     print(cdf)
        cdf_max = cdf[-1]
        cdf_thresh_min = int(cdf_max * (1 - gradient_crop_ratio) / 2)
        cdf_thresh_max = int(cdf_max * (1 + gradient_crop_ratio) / 2)
        sorted_intensity = sorted(intensity_buffer)
        intensity_thresh_min = sorted_intensity[cdf_thresh_min]
        intensity_thresh_max = sorted_intensity[cdf_thresh_max]
        intensity_range = intensity_thresh_max - intensity_thresh_min
        if intensity_range < min_intensity_range: z_buffer = [-1 for i in range(len(self.waypoints))] # This is not gradient (z=-1 -> Hold height)
        else:
            z_buffer = []
            for intensity in intensity_buffer:
                if intensity <= intensity_thresh_min: z_buffer.append(min_height)
                elif intensity >= intensity_thresh_max: z_buffer.append(max_height)
                else: z_buffer.append((intensity - intensity_thresh_min) * (height_range) / intensity_range + min_height)
        for i, (x, y) in enumerate(self.waypoints): self.waypoints3D.append((x, y, z_buffer[i]))
        return self.waypoints3D
    def generate_trajectory3D_rev3(self, src, min_height, max_height, gradient_crop_ratio, min_intensity_range, path_cnts_approx): # Use points to sample intensity
        self.waypoints3D = []
        intensity_buffer = []
        for (x, y) in self.waypoints: intensity_buffer.append(255 - src[y][x])
        height_range = max_height - min_height
        hist, bins = np.histogram(intensity_buffer, 256, [0, 256])
        cdf = hist.cumsum()
        #     print(cdf)
        cdf_max = cdf[-1]
        cdf_thresh_min = int(cdf_max * (1 - gradient_crop_ratio) / 2)
        cdf_thresh_max = int(cdf_max * (1 + gradient_crop_ratio) / 2)
        sorted_intensity = sorted(intensity_buffer)
        intensity_thresh_min = sorted_intensity[cdf_thresh_min]
        intensity_thresh_max = sorted_intensity[cdf_thresh_max]
        intensity_range = intensity_thresh_max - intensity_thresh_min
        if intensity_range < min_intensity_range: z_buffer = [-1 for i in range(len(self.waypoints))] # This is not gradient (z=-1 -> Hold height)
        else:
            z_buffer = []
            for intensity in intensity_buffer:
                if intensity <= intensity_thresh_min: z_buffer.append(min_height)
                elif intensity >= intensity_thresh_max: z_buffer.append(max_height)
                else: z_buffer.append((intensity - intensity_thresh_min) * (height_range) / intensity_range + min_height)
        for i, (x, y) in enumerate(self.waypoints): self.waypoints3D.append((x, y, z_buffer[i]))
        waypoints3D_approx = []
        for point_approx in path_cnts_approx:
            best = self.waypoints3D[0]
            best_distance = euclidean((point_approx[0][0], point_approx[0][1]), (best[0], best[1]))
            for candidate in self.waypoints3D:
                distance = euclidean((point_approx[0][0], point_approx[0][1]), (candidate[0], candidate[1]))
                if distance < best_distance:
                    best_distance = distance
                    best = candidate
            waypoints3D_approx.append(best)
        return waypoints3D_approx