from enum import Enum
import mapFile
import numpy as np
class editorMode(Enum):
    MOVE = 1
    ADD = 2
    REMOVE = 3
    LANE_CONNECT_LEFT = 4
    LANE_CONNECT_RIGHT = 5
    TIMEKEEPING_START = 6
    TIMEKEEPING_FINISH = 7
    TIMEKEEPING_SECTOR = 8
class landmarkType(Enum):
    UNDEFINED = 0
    BLUE = 1
    YELLOW = 2
    ORANGE = 3
    BIG_ORANGE = 4
    INVISIBLE = 5
    TIMEKEEPING = 6

class guiLogic():
  def __init__(self, *args, **kwargs):
    self.trajectory_json_data = None
    self.editorMode = editorMode.ADD
    self.landmarkType = landmarkType.UNDEFINED
    self.graphicsView = None
    self.cones = []
    self.coneColorMap = {}
    self.lanesConnectionLeft = []
    self.lanesConnectionRight = []
    self.timeKeepingGates = []
    self.startPosition = np.array([0,0,0])
    self.startOrientation = np.array([0,0,0])
    d2r = np.pi / 180.0
    self.originGeodeticCoordinates = np.array([51.001745, 13.641794,210])
    self.originENURotation = d2r * np.array([0,0,45])
    self.width = 0
    self.dist_cones = 0
    self.v_max = 0
    self.ax_max = 0
    self.ay_max = 0
  # https://stackoverflow.com/questions/27161533/find-the-shortest-distance-between-a-point-and-line-segments-not-line
  def lineseg_dists(self, p, a, b):
      """Cartesian distance from point to line segment

      Edited to support arguments as series, from:
      https://stackoverflow.com/a/54442561/11208892

      Args:
          - p: np.array of single point, shape (2,) or 2D array, shape (x, 2)
          - a: np.array of shape (x, 2)
          - b: np.array of shape (x, 2)
      """
      # normalized tangent vectors
      d_ba = b - a
      d = np.divide(d_ba, (np.hypot(d_ba[:, 0], d_ba[:, 1])
                            .reshape(-1, 1)))

      # signed parallel distance components
      # rowwise dot products of 2D vectors
      s = np.multiply(a - p, d).sum(axis=1)
      t = np.multiply(p - b, d).sum(axis=1)

      # clamped parallel distance
      h = np.maximum.reduce([s, t, np.zeros(len(s))])

      # perpendicular distance component
      # rowwise cross products of 2D vectors  
      d_pa = p - a
      c = d_pa[:, 0] * d[:, 1] - d_pa[:, 1] * d[:, 0]

      return np.hypot(h, c)

  def getClosestInd(self, point, list):
    bestInd = 0
    bestDist = 999
    for i in range(len(list)):
      dist = np.linalg.norm(list[i][0]-point)
      if(dist < bestDist):
        bestDist = dist
        bestInd = i
    return bestInd

  def getMinTrackWidth(self):
      if len(self.lanesConnectionLeft) == 0 or len(self.lanesConnectionRight) == 0:
          return 0

      minDistance = 999
      offsetLane = 0
      toBeCheckedConesLeft = self.lanesConnectionLeft[:-1]
      toBeCheckedConesRight = self.lanesConnectionRight[:-1]

      for i in range(-offsetLane, len(toBeCheckedConesLeft) - 1):
          p1 = np.array(toBeCheckedConesLeft[i-2][0])  # Convert to numpy array
          p2 = np.array(toBeCheckedConesLeft[i-1][0])  # Convert to numpy array
          p3 = np.array(toBeCheckedConesLeft[i][0])    # Convert to numpy array

          tangential = p3 - p1  # Now the subtraction can be done between numpy arrays
          tangential = (1 / np.linalg.norm(tangential)) * tangential

          projectionDistance = 4
          normal = np.array([-tangential[1], tangential[0], 0])
          normal = projectionDistance * normal

          closestInd = self.getClosestInd(p2 + normal, toBeCheckedConesRight)
          indicesToCheck = np.linspace(closestInd - 5, closestInd + 5, num=11, dtype=int)
          
          a = []
          b = []
          for j in indicesToCheck:
              a.append(toBeCheckedConesRight[j % len(toBeCheckedConesRight)][0][0:2])
              b.append(toBeCheckedConesRight[(j+1) % len(toBeCheckedConesRight)][0][0:2])

          minDistance = min(minDistance, min(self.lineseg_dists(p2[0:2], np.array(a), np.array(b))))

      return minDistance  
  
  def getTrackLength(self):
      if len(self.lanesConnectionLeft) == 0 or len(self.lanesConnectionRight) == 0:
          return 0
      leftLength = 0
      rightLength = 0
      offsetLane = 0

      for i in range(-offsetLane, len(self.lanesConnectionLeft)):
          p1 = np.array(self.lanesConnectionLeft[i-1][0])
          p2 = np.array(self.lanesConnectionLeft[i][0])
          dist = np.linalg.norm(p1 - p2)
          leftLength += dist

      for i in range(-offsetLane, len(self.lanesConnectionRight)):
          p1 = np.array(self.lanesConnectionRight[i-1][0])
          p2 = np.array(self.lanesConnectionRight[i][0])
          dist = np.linalg.norm(p1 - p2)
          rightLength += dist

      return 0.5 * (leftLength + rightLength)

  def getMaxLaneDistance(self):
      if len(self.lanesConnectionLeft) == 0 or len(self.lanesConnectionRight) == 0:
          return 0

      maxDist = 0
      offsetLane = -1

      # For the left side
      for i in range(-offsetLane, len(self.lanesConnectionLeft)):
          p1 = np.array(self.lanesConnectionLeft[i-1][0])  # Convert to numpy array
          p2 = np.array(self.lanesConnectionLeft[i][0])    # Convert to numpy array
          dist = np.linalg.norm(p1 - p2)
          maxDist = max(maxDist, dist)

      # For the right side
      for i in range(-offsetLane, len(self.lanesConnectionRight)):
          p1 = np.array(self.lanesConnectionRight[i-1][0])  # Convert to numpy array
          p2 = np.array(self.lanesConnectionRight[i][0])    # Convert to numpy array
          dist = np.linalg.norm(p1 - p2)
          maxDist = max(maxDist, dist)

      return maxDist

  # https://stackoverflow.com/questions/41144224/calculate-curvature-for-3-points-x-y
  def getCurvature(self, point1, point2, point3):
    # Calculating length of all three sides
    len_side_1 = round( np.linalg.norm(point1-point2), 2)
    len_side_2 = round( np.linalg.norm(point2-point3), 2)
    len_side_3 = round( np.linalg.norm(point1-point3), 2)

    # sp is semi-perimeter
    sp = (len_side_1 + len_side_2 + len_side_3) / 2 

    # Calculating area using Herons formula
    area = np.sqrt(max(sp * (sp - len_side_1) * (sp - len_side_2) * (sp - len_side_3),0.00001))

    # Calculating curvature using Menger curvature formula
    curvature = (4 * area) / max(len_side_1 * len_side_2 * len_side_3,0.0000001)

    return curvature

  def getMinOuterRadius(self):
      if len(self.lanesConnectionLeft) == 0 or len(self.lanesConnectionRight) == 0:
          return 0

      minRadius = 999
      offsetLane = 0
      toBeCheckedConesLeft = self.lanesConnectionLeft[:-1]
      toBeCheckedConesRight = self.lanesConnectionRight[:-1]

      for i in range(-offsetLane, len(toBeCheckedConesLeft) - 2):
          # Convert the tuples to numpy arrays for mathematical operations
          p1 = np.array(toBeCheckedConesLeft[i-2][0])
          p2 = np.array(toBeCheckedConesLeft[i-1][0])
          p3 = np.array(toBeCheckedConesLeft[i][0])

          # Calculate the tangential vector by subtracting the positions
          tangential = p3 - p1
          tangential = (1 / np.linalg.norm(tangential)) * tangential

          projectionDistance = 4
          normal = np.array([-tangential[1], tangential[0], 0])
          normal = projectionDistance * normal

          curvature = self.getCurvature(p1,p2,p3)

          if(curvature < 0.05):
            continue
          closestInd = self.getClosestInd(p2 + normal, toBeCheckedConesRight)
          minRadius = min(minRadius, 1.0/curvature)

      return minRadius
  
  def readMapFile(self, path):
    self.cones = []
    self.coneColorMap = {}
    self.lanesConnectionLeft = []
    self.lanesConnectionRight = []
    self.timeKeepingGates = []
    conesUnknown, left, right, tk, connected, start, earthToTrack, orangeCones = mapFile.readPCD(path)
    self.startPosition = start[0]
    self.startOrientation = start[1]
    self.originGeodeticCoordinates = earthToTrack[0]
    self.originENURotation = earthToTrack[1]
    for i in conesUnknown:
      self.cones.append([i[0], mapFile.stringToLandmarkType(i[1])])
    for i in left:
      l = [i[0], mapFile.stringToLandmarkType(i[1])]
      self.cones.append(l)
      self.lanesConnectionLeft.append(l)
    for i in right:
      l = [i[0], mapFile.stringToLandmarkType(i[1])]
      self.cones.append(l)
      self.lanesConnectionRight.append(l)
    for i in orangeCones:
      l = [i[0], mapFile.stringToLandmarkType(i[1])]
      self.cones.append(l)
    duringLine = False
    for i in tk:
      l = [i[0], mapFile.stringToLandmarkType(i[1])]
      self.cones.append(l)
      if(duringLine):
        self.timeKeepingGates[-1].append(l)
        duringLine = False
      else:
        self.timeKeepingGates.append([l])
        duringLine = True
    if(len(self.lanesConnectionLeft) >= 2 and connected):
       self.lanesConnectionLeft.append(self.lanesConnectionLeft[0])
    if(len(self.lanesConnectionRight) >= 2 and connected):
       self.lanesConnectionRight.append(self.lanesConnectionRight[0])

  def drawCones(self):
    self.graphicsView.initFromLoad()
    #  for c in self.cones:
    #     self.graphicsView.addCone(c)
  def writeMapFile(self, path):
    conesUnknown = []
    for i in self.cones:
      good = True
      for j in self.lanesConnectionLeft:
         good = good and (i is not j)
      for j in self.lanesConnectionRight:
         good = good and (i is not j)
      good = good and (i[1] != landmarkType.TIMEKEEPING)
      if(good):
         conesUnknown.append(i)
    timeKeeping = []
    for i in self.timeKeepingGates:
      timeKeeping.append(i[0])
      timeKeeping.append(i[1])

    mapFile.writeYaml(path, conesUnknown, self.lanesConnectionLeft, self.lanesConnectionRight, timeKeeping, [self.startPosition, self.startOrientation], [self.originGeodeticCoordinates, self.originENURotation], self.trajectory_json_data)