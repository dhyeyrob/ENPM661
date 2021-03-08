# Importing Required Libraries
import cv2
import numpy as np

# Making a function to check if the new point lies inside the obstacle space or not
def obstacle_check(y,x):
    z = 0

    if (x - 90) ** 2 + (y - 230) ** 2 <= (35 * 35):      # Checking if new point is inside circle obstacle
        z = z + 1

    elif (((x - 246) ** 2) / (60 * 60) + ((y - 155) ** 2) / (30 * 30)) <= 1:  # Checking if new point is inside ellipse
        z = z + 1

    elif (200 <= x <= 230 and 60 <= y <= 70) or (200 <= x <= 230 and 20 <= y <= 30) or (200 <= x <= 210 and 30 <= y <= 60):  # Checking if new point is inside C shaped obstacle
        z = z + 1

    elif ((300-y) - 0.7 * x - 74.4 >= 0) and ((300-y) - 0.7 * x - 90.8 <= 0) and ((300-y) + 1.43 * x - 176.64 >= 0) and ((300-y) + 1.43 * x - 438.64 <= 0): # checking if point is inside rectangle
        z = z + 1

    #  Checking if new point is inside polygon obstacle or not
    elif ((300-y) - x + 265 >= 0) and ((300-y) - x + 216 <= 0) and (x >= 354) and (x <= 381) or ((300-y) + (42 / 43) * x - (16485 / 43) >= 0) and ((300-y) - x + 265 >= 0) and (x <= 354) and ((300-y) - x + 180 <= 0) and ((300-y) + (7 / 29) * x - (6480 / 29) <= 0):
        z = z + 1



    return z

# Main code starts from here
image = np.zeros((300, 400, 3), dtype="uint8")    # Generating a black space of the size 300x400

for i in range(300):
    for j in range(400):
        y = i
        x = j

        # Drawing circle obstacle
        if (x - 90) ** 2 + (y - 230) ** 2 <= (35 * 35):
            image[i:(i + 1), j:(j + 1)] = (125, 0, 125)

        #  Drawing ellipse obstacle
        elif ((x-246)**2/(60*60)) + ((y-155)**2)/(30*30) <= 1:
            image[i:(i + 1), j:(j + 1)] = (125, 0, 125)
        # Drawing C shaped obstacle
        elif (200 <= x <= 230 and 60 <= y <= 70) or (200 <= x <= 230 and 20 <= y <= 30) or (200 <= x <= 210 and 30 <= y <= 60):
            image[i:(i + 1), j:(j + 1)] = (125, 0, 125)

# Drawing rectangle and complex polygon obstacle
for i in range(300):
    for j in range(400):

        y = i
        y = 300 - y
        x = j
        if (y - 0.7 * x - 74.4 >= 0) and (y - 0.7 * x - 90.8 <= 0) and (y + 1.43 * x - 176.64 >= 0) and (y + 1.43 * x - 438.64 <= 0):
            image[i:(i + 1), j :(j + 1) ] = (125, 0, 125)

        if (y - x + 265 >= 0) and (y - x + 216 <= 0) and (x >= 354) and (x <= 381):
            image[i:(i + 1), j:(j + 1)] = (125, 0, 125)

        elif (y + (42 / 43) * x - (16485 / 43) >= 0) and (y - x + 265 >= 0) and (x <= 354) and (
                y - x + 180 <= 0) and (y + (7 / 29) * x - (6480 / 29) <= 0):
            image[i:(i + 1), j:(j + 1)] = (125, 0, 125)



print("searching.... ")
grid = [[0 for row in range(400)] for col in range(300)]     # Creating a grid of 300x400
mark_path = [[0 for row in range(400)] for col in range(300)]
action = [[0 for row in range(400)] for col in range(300)]
action_name = ['>', '<', '^', 'v', 'right lower diag', 'left lower diag','right upper diag', 'left upper diag']

# Creating a move function with 8 possible moves
mov_fn = [[0,1],
          [0,-1],
          [1,0],
          [-1,0],
          [1,1],
          [1,-1],
          [-1,1],
          [-1,-1]]

# Asking the user to enter coordinates of the starting location of the point robot
xs = int(input("Please enter the x coordinate of the START location:"))
ys = int(input("Please enter the y coordinate of the START location:"))

# Asking the user to enter coordinates of the goal location of the point robot
xg = int(input("Please enter the x coordinate of the GOAL location:"))
yg = int(input("Please enter the y coordinate of the GOAL location:"))

ys = 300 - ys
yg = 300 - yg

t = obstacle_check(ys,xs)  # Checking the validity of start location
r = obstacle_check(yg,xg)  # Checking the validity of the goal location
image[yg][xg] = (0,0,255)

# Checking validity of starting point
if t == 0 and ys >= 0 and ys < len(grid) and xs >= 0 and xs < len(grid[0]):

    # Checking validity of goal point
    if r == 0 and yg >= 0 and yg < len(grid) and xg >= 0 and xg < len(grid[0]):
        init = [ys,xs]                           # Initializing starting points
        goal_loc = [yg,xg]                       # Initializing goal points
        list_poi = [[init[0],init[1]]]           # Creating a list of points to store all valid points
        all_poi = [[init[0],init[1]]]            # Creating a list of points to store all valid points
        g = 0
        nodes = [[g, init[0], init[1]]]
        mark_points = []
        mark_points.append([goal_loc[0], goal_loc[1]])  # Creating a list to compute back tracking
        goal_reached = False

        #  Initializing a while loop until goal position is reached
        while goal_reached == False:
            temp_poi = list_poi.pop(0)                  # Popping out points from list of points
            g = g + 1
            for i in range(len(mov_fn)):                # Initializing move function for 8 possible moves
                if goal_reached == False:
                    xn = temp_poi[0] + mov_fn[i][0]
                    yn = temp_poi[1] + mov_fn[i][1]

                    if xn >= 0 and xn < len(grid) and yn >= 0 and yn < len(grid[0]): # Checking validity of new point
                        n = 0
                        for k in range(len(all_poi)):                                # Checking repeatability of new point
                            if all_poi[k][0] == xn and all_poi[k][1] == yn:
                                n = n + 1

                        if n == 0:
                            s = obstacle_check(xn,yn)                                # Checking new point inside obstacle
                            if s == 0:
                                list_poi.append([xn,yn])                             # Appending points in list of points
                                all_poi.append([xn,yn])                              # Appending points in list of points
                                nodes.append([g, xn, yn])
                                action[xn][yn] = i
                                image[xn][yn] = (255, 255, 255)
                                cv2.imshow("Searching", image)                       # Displaying visualization for search
                                cv2.waitKey(1)

                                if xn == goal_loc[0] and yn == goal_loc[1]:          # Comparing point with goal location
                                    print("goal found")
                                    goal_reached =  True
                                    xi = goal_loc[0]
                                    yi = goal_loc[1]
                                    mark_path[xi][yi] = 'G'
                                    while xi != init[0] or yi != init[1]:            # Initializing back tracking
                                        xc = xi - mov_fn[action[xi][yi]][0]          # Starting bak tracking from goal loc
                                        yc = yi - mov_fn[action[xi][yi]][1]
                                        mark_path[xc][yc] = action_name[action[xi][yi]]
                                        mark_points.append([xc, yc])                 # Appending points of optimal path
                                        xi = xc
                                        yi = yc

        cv2.destroyAllWindows()
        tracking_done = False

        # Initializing Back Tracking Visualization
        while tracking_done == False:

            for v in mark_points:
                image[v[0]][v[1]] = (0,0,255)
                cv2.imshow("Optimal Path", image)
                cv2.waitKey(250)
            cv2.destroyAllWindows()
            tracking_done = True

        cv2.imshow("Final Output", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    else: print("invalid goal location")

else:
    print("please enter valid start location")