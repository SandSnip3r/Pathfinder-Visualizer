import csv
import sys

def adjustLatLongPointsForPathfinder(polylines):
  # Input points are in lat/long. Center them around 0,0 and scale them up to fill a 1000x1000 area.
  minX = float('inf')
  minY = float('inf')
  maxX = float('-inf')
  maxY = float('-inf')
  for polyline in polylines:
    for point in polyline:
      minX = min(minX, point[0])
      minY = min(minY, point[1])
      maxX = max(maxX, point[0])
      maxY = max(maxY, point[1])
  xRange = maxX-minX
  yRange = maxY-minY
  midpoint = [minX + xRange/2, minY + yRange/2]
  desiredRange = 1000
  multiplier = desiredRange/max(xRange,yRange)
  newPolylines = []
  for polyline in polylines:
    newPolyline = []
    for point in polyline:
      newPolyline.append([multiplier * (point[0]-midpoint[0]), multiplier * (point[1]-midpoint[1])])
    if len(newPolyline) > 0:
      newPolylines.append(newPolyline)
  return newPolylines

def printAsPoly(polylines, output_file):
  with open(output_file, 'w') as f:
    # 9 2 0 0
    pointCount = sum(len(line) for line in polylines) + 4
    print(f'{pointCount} 2 0 0', file=f)
    index = 0
    minX = float('inf')
    minY = float('inf')
    maxX = float('-inf')
    maxY = float('-inf')
    for polyline in polylines:
      for point in polyline:
        print(f'{index} {point[0]} {point[1]}', file=f)
        index += 1
        minX = min(minX, point[0])
        minY = min(minY, point[1])
        maxX = max(maxX, point[0])
        maxY = max(maxY, point[1])
    print(f'{index} {minX} {minY}', file=f)
    index += 1
    print(f'{index} {minX} {maxY}', file=f)
    index += 1
    print(f'{index} {maxX} {minY}', file=f)
    index += 1
    print(f'{index} {maxX} {maxY}', file=f)
    index += 1

    connections = []
    # 0 0 1 3
    for polyline in polylines:
      connections.append([[x, x+1, 3] for x in range(len(polyline)-1)])
    connectionCount = 0
    index = 1
    while index < len(polylines):
      connectionCount += len(polylines[index-1])
      for connection in connections[index]:
        connection[0] += connectionCount
        connection[1] += connectionCount
      index += 1
    
    print(f'{sum(len(connectionList) for connectionList in connections)} 1', file=f)
    index = 0
    for connectionList in connections:
      for connection in connectionList:
        print(f'{index} {connection[0]} {connection[1]} {connection[2]}', file=f)
        index += 1

    print('0\n', file=f)

def parse_csv(file_path):
  try:
    polylines = []
    with open(file_path, 'r', newline='') as csvfile:
      # Create a CSV reader
      csv_reader = csv.reader(csvfile)
      polyline = []
      # Read and print each row
      isFirst = True
      for row in csv_reader:
        if isFirst:
          isFirst = False
          continue
        if row[3] == '0':
          if len(polyline) > 0:
            polylines.append(polyline)
          polyline = []
        polyline.append([float(row[0]), float(row[1])])
      if len(polyline) > 0:
        polylines.append(polyline)
      
    return polylines
  except FileNotFoundError:
    print(f"File not found: {file_path}")
  except Exception as e:
    print(f"An error occurred: {e}")

if __name__ == "__main__":
  if len(sys.argv) != 3:
    print(f"Usage: python {sys.argv[0]} input_file.csv output_file.poly")
    sys.exit(1)

  input_file = sys.argv[1]
  output_file = sys.argv[2]

  print(f'Parsing .csv file {input_file}')
  polylines = parse_csv(input_file)
  # output_file
  adjustedPolylines = adjustLatLongPointsForPathfinder(polylines)
  print(f'Writing .poly file {output_file}')
  printAsPoly(adjustedPolylines, output_file)
  

