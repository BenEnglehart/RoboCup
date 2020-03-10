# IMPORTED LIBRARIES
import math

# FUNCTIONS:
def grid_to_pxl_ratio():

    global pxl_size, field_bounds

    x_rat = pxl_size[0]/field_bounds[1]
    y_rat = pxl_size[1]/field_bounds[3]

    return [x_rat, y_rat]

def count_rob(coords_arr):
    # Function to return the number of robots identified by the vision algorithm
    # Inputs: coords_arr - matrix of robot information for check
    # Outputs: rob_num - number of rows identified (symbolizing number of robots)
    global rob_num

    rob_num = len(coords_arr)

def pxl_2_grid(coords_arr):
    # Function used to convert the information gathered in pixels to point on our imaginary grid placed over the field of
    # play
    # Inputs: coords_arr - matrix containing all robot information [id, x, y, angle]
    # Outputs: coords_arr_grd - matrix containing all robot information on grid [id, x, y, angle]

    global pxl_grd_ratio, rob_num

    coords_arr_grd = []        # Initiate empty matrix
    for i in range(rob_num):
        dummy_arr = [0, 0, 0, 0]      # Initiate dummy arr for appending
        dummy_arr[0] = coords_arr[i][0]

        # Modulus code to determine weather to ceil() or floor() to round x coordinate to nearest grid position
        if (coords_arr[i][1] % pxl_grd_ratio[0] > math.floor(pxl_grd_ratio[0] / 2)):
            dummy_arr[1] = math.ceil(coords_arr[i][1]/pxl_grd_ratio[0])
        else:
            dummy_arr[1] = math.floor(coords_arr[i][1] / pxl_grd_ratio[0])

        # Modulus code to determine weather to ceil() or floor() to round y coordinate to nearest grid position
        if coords_arr[i][2] % pxl_grd_ratio[1] > math.floor(pxl_grd_ratio[1] / 2):
            dummy_arr[2] = math.ceil(coords_arr[i][2]/pxl_grd_ratio[1])
        else:
            dummy_arr[2] = math.floor(coords_arr[i][2] / pxl_grd_ratio[1])

        dummy_arr[3] = coords_arr[i][3]

        coords_arr_grd.append(dummy_arr)

    return coords_arr_grd

def grd_2_pxl(coords_arr_grd):

    global pxl_grd_ratio

    rob_num = al_n + op_n

    new_pos = []        # Initiate empty return array
    for i in range(rob_num):
        dummy_arr = [0, 0, 0, 0]

        dummy_arr[0] = coords_arr[i][0]
        dummy_arr[1] = math.floor(coords_arr_grd[i][1] * pxl_grd_ratio[0])
        dummy_arr[2] = math.floor(coords_arr_grd[i][2] * pxl_grd_ratio[1])
        dummy_arr[3] = coords_arr[i][3]

        new_pos.append(dummy_arr)

    return new_pos

def count_al_op(coords_arr_grd):

    global al_n, op_n, rob_num

    al_n = 0; op_n = 0

    for i in range(rob_num):
        if coords_arr_grd[i][0] <= 16:
            al_n = al_n+1
        else:
            op_n = op_n+1

def point_prox(xpoint, ypoint, oIDInfo):

    # Function used to determine the proximity of a given point on the field to a specified opponent
    # Inputs:   x coordinate of relevant point
    #           y coordinate of relevant point
    #           opponent ID information
    # Outputs:  proximity array detailing magnitude and angle of dirrection (respectiely)

    if oIDInfo[1] - xpoint == 0:
        ang = math.atan((oIDInfo[2] - ypoint) / (0.0000001))        # Special case in which the two are at 90 deg from one another
    else:
        ang = math.atan((oIDInfo[2] - ypoint) / (oIDInfo[1] - xpoint))  # General case

    ang = ang * 180 / 3.14159       # Convert angle from rad to degrees
    mag = math.sqrt(pow(oIDInfo[2] - ypoint, 2) + math.pow(oIDInfo[1] - xpoint, 2))     # Determine the magnitudinal distance between the points and the opponent

    # debugging
    # print(str(ang))
    # print(str(mag))

    ans = [mag, ang]        # Array for return
    return ans

def check_LOS_net(aIDInfo, oIDInfo):

    # Function used to determine if there is a clear line of sight between an ally and the net w.r.t. specified opponent
    # Inputs:   ally ID information
    #           opponent ID information
    # Output:   status of opponents interference in path between the specified ally and the net (1 = clear, 0 = unclear)

    global net_location     # Aquire necessary global variables

    dx = net_location[0] - aIDInfo[1]       # Determine the distance in the x direction
    dy = net_location[1] - aIDInfo[2]       # Determine the distance in the y direction

    j = 0       # Counting variable for point divisions
    point_div = []      # Initiate blank array
    for i in range(50):
        point_div.append(j)
        j = j + 1/50

    for i in range(50):
        prox = point_prox(aIDInfo[1] + point_div[i] * dx, aIDInfo[2] + point_div[i] * dy, oIDInfo)        #For each division along path to net, check proximity to the opponent
        if prox[0] <= 1.7:
            return 0        #If path is found to be within 1/2 a specified unit, break the function and return a "FALSE" flag
        # ***********

    return 1        #If no interference is found, will return a "TRUE" flag

def check_LOS_al(aIDInfo1, aIDInfo2, oIDInfo):

    # Function used to determine if there is a clear line of sight between two allys w.r.t. specified opponent
    # Inputs:   ally ID information ball holder
    #           ally ID information ball recipient
    #           opponent ID information
    # Output:   status of opponents interference in path between the specified allys (1 = clear, 0 = unclear)

    dX = aIDInfo2[1] - aIDInfo1[1]
    dY = aIDInfo2[2] - aIDInfo1[2]

    j = 0  # Counting variable for point divisions
    point_div = []  # Initiate blank array
    for i in range(50):
        point_div.append(j)
        j = j + 1 / 50

    for i in range(50):
        prox = point_prox(aIDInfo1[1] + point_div[i] * dX, aIDInfo1[2] + point_div[i] * dY, oIDInfo)
        if prox[0] <= 1.7:
            return 0

    return 1

def bounds_to_arr(field_bounds):

    # Function used to create an array to store a point values for each point within the grid bounds specified by the user
    # Inputs:   boundaries of the grid as specified by the user
    # Outputs:  returns an array (zeros) to store values for each grid point

    arr = []        # Initialize a blank array
    for i in range(int((field_bounds[1]+1)*(field_bounds[3]+1))):
        arr.append(0)       # For the number of grid spaces specified for the plane, create a corresponding array space to store its value

    # DEBUG
    # print(str(len(point_vals)))

    return arr      # Return the array

def field_sweep(oIDInfo):

    # Function used to sweep the imaginary grid placed over the field to determine high/low risk areas w.r.t. specified opponent
    # Inputs:   opponent ID information
    # Outputs:  modifies values within the "point value" array generated by bounds_to_arr function (modified using global call)

    global point_vals, field_bounds     # Aquire necessary global variables

    for i in range(field_bounds[1]+1):      # For each x coordinate in the grid
        for j in range(field_bounds[3]+1):      # For each y coordinate in the grid
            prox = point_prox(i, j, oIDInfo)        # Determine the proximity of the specified opponent to each point in the plane
            if (prox[0] < math.sqrt(2)) and (point_vals[i*(field_bounds[3] + 1)+j] < 1):
                point_vals[i*(field_bounds[3] + 1)+j] = point_vals[i*(field_bounds[3] + 1)+j] + 1       # If the point is within the specified magnitude and has not already been flagged, then flag it

def point_val_check(x, y):

    # Function called to check the appropriate value in the array associated with the point values of the array
    # Inputs:   x coordinate of interest
    #           y coordinate of interest
    # Outputs:  boulien value associated with grid location

    global field_bounds, point_vals     # Aquire necessary global variables

    arr = []        # Initialize a blank array
    # Following block creates an array to reference in order to find location in the point_vals array
    for i in range(field_bounds[1]+1):
        arr.append(i)
    for i in range(field_bounds[3]+1):
        arr.append(i)

    # DEBUG:
    # print(str(arr))

    status = point_vals[arr[x]*(field_bounds[3] + 1) + y]        # Access the appropriate flag in the point_vals array

    # DEBUG:
    # print(str(arr[x]*7 + arr[field_bounds[2] + y]))

    return status       # Return the flag (occupied/unoccupied) respectively (TRUE/FALSE)

def ser_check(aIDInfoC, aIDInfo2, oIDInfo):

    # Function used to check the imediate e,w,n,s coordinates (respectively) serounding the robot to check for a better scoring/passing position
    # Inputs:   controlled/passing robot
    #           receiving robot
    #           opponent of interest for interference
    # Ouptuts:  modifies a 4 element array holding the value/s of the associated serounding options for kicking/passing

    global net_location, point_vals, ser_check_vals

    aIDInfoOpt = [aIDInfoC[1] + 2, aIDInfoC[2], aIDInfoC[1] - 2, aIDInfoC[2], aIDInfoC[1], aIDInfoC[2] + 2, aIDInfoC[1], aIDInfoC[2] - 2]       # Create a reference for points serrounding the robot

    aIDInfoC1 = [1, aIDInfoC[1] + 2, aIDInfoC[2]]       # Point located at coordinate (x+1, y) w.r.t. current robot position
    aIDInfoC2 = [2, aIDInfoC[1] - 2, aIDInfoC[2]]       # Point located at coordinate (x-1, y) w.r.t. current robot position
    aIDInfoC3 = [3, aIDInfoC[1], aIDInfoC[2] + 2]       # Point located at coordinate (x, y+1) w.r.t. current robot position
    aIDInfoC4 = [4, aIDInfoC[1], aIDInfoC[2] - 2]       # Point located at coordinate (x, y-1) w.r.t. current robot position

    # Each of the following blocks below will evaluate one of the respective points above in the order listed. Done by
    # ensuring first that the position is within the bounds of the field limits, then checking if its risk flag
    # (point_vals) was tripped, and lastly the line of sight (net/opponent depending on location) is checked before its
    # value is either incremented (good location) or left unaltered (poor location) for the specified task.

    # Check location east or robot for LOS to net
    i = 0
    if (aIDInfoOpt[i * 2] <= field_bounds[1]-2) and not(point_val_check(aIDInfoOpt[i * 2], aIDInfoOpt[i * 2 + 1])) and check_LOS_net(aIDInfoC1, oIDInfo):
        ser_check_vals[i] = ser_check_vals[i] + 1
        # DEBUG
        # print"1++")

    # Check location west of robot for LOS to net
    i = i + 1
    if (aIDInfoOpt[i * 2] >= 0) and not(point_val_check(aIDInfoOpt[i * 2], aIDInfoOpt[i * 2 + 1])) and check_LOS_net(aIDInfoC2, oIDInfo):
        ser_check_vals[i] = ser_check_vals[i] + 1
        # DEBUG
        # print('\n2++\n')

    # Check location north of robot for LOS to ally
    i = i + 1
    if (aIDInfoOpt[i * 2 + 1] <= field_bounds[3]-2) and not(point_val_check(aIDInfoOpt[i * 2], aIDInfoOpt[i * 2 + 1])) and check_LOS_al(aIDInfoC3, aIDInfo2, oIDInfo):
        ser_check_vals[i] = ser_check_vals[i] + 1
        # DEBUG
        # print('\n3++\n')

    # Check location south of robot for LOS to ally
    i = i + 1
    if (aIDInfoOpt[i * 2 + 1] >= 0) and not(point_val_check(aIDInfoOpt[i * 2], aIDInfoOpt[i * 2 + 1])) and check_LOS_al(aIDInfoC4, aIDInfo2, oIDInfo):
        ser_check_vals[i] = ser_check_vals[i] + 1
        # DEBUG
        # print('\n4++\n')

def op_rec_loc(aIDInfo, oIDInfo):

    # Function determines the top ____ locations from which to recieve a pass
    # Inputs:   ball possessor ID information
    #           opponent ID information of interest
    # Outputs:  modifies specified array of op_loc_values

    global op_loc_point_vals, net_location, point_vals, field_bounds        # Aquire necessary global variables

    # Check LOS for each point on grid between ball possessor and net

    # DEBUG:
    # k = 1

    for i in range(field_bounds[1] + 1):        # For each x  on grid
        for j in range(field_bounds[3] + 1):        # For each y point on grid
            dummy_id = [0, i, j]        # Create a temporary dummy robot id for function compatibility

            #DEBUG:
            # k = k + 1
            # print(str(i)+" and "+str(j)+" gives " + str(point_val_check(i, j)))
            # print(str(k))

            if not(point_val_check(i, j)) and check_LOS_al(aIDInfo, dummy_id, oIDInfo) and check_LOS_net(dummy_id, oIDInfo):        # If position not flagged, and LOS is clear to both the ball possessor and the net
                op_loc_point_vals[i * (field_bounds[3] + 1) + j] = op_loc_point_vals[i * (field_bounds[3] + 1) + j] + 1             # Increase the locations "value"


    # Check proximity of each point to opponents and adjust their values according to a ratio determined by point proximity to an opponent
    for i in range(field_bounds[1] + 1):        # For each x point on grid
        for j in range(field_bounds[3] + 1):        # For each x point on grid
            prox = point_prox(i, j, oIDInfo)        # Check the proximity of the point
            if prox[0] <= 2:        # If within a specified unit
                op_loc_point_vals[i * (field_bounds[3] + 1) + j] = op_loc_point_vals[i * (field_bounds[3] + 1) + j]*0.85        # Reduce the points "value" by 75%
            elif prox[0] < 4:       # If within two specified unit
                op_loc_point_vals[i * (field_bounds[3] + 1) + j] = op_loc_point_vals[i * (field_bounds[3] + 1) + j] * 0.90       # Reduce the points "value" by 90%
            elif prox[0] < 6:       # If within specified unit
                op_loc_point_vals[i * (field_bounds[3] + 1) + j] = op_loc_point_vals[i * (field_bounds[3] + 1) + j] * 0.95

def op_point_val_check(x, y):

    # Function called to check the appropriate value in the array associated with the point values of the array
    # Inputs:   x coordinate of interest
    #           y coordinate of interest
    # Outputs:  boulien value associated with grid location

    global field_bounds, op_loc_point_vals     # Aquire necessary global variables

    arr = []        # Initiate empty array

    # Following block creates a reference array for the function
    for i in range(field_bounds[1]+1):
        arr.append(i)
    for i in range(field_bounds[3]+1):
        arr.append(i)

    # DEBUG:
    # print(str(arr))

    val = op_loc_point_vals[arr[x]*(field_bounds[3] + 1) + y]       # Identify the correct position in the "value" array

    # DEBUG:
    # print(str(arr[x]*7 + arr[field_bounds[2] + y]))

    return val      # Return the appropriate value of the point

# STILL IN DEVELOPEMENT

# Include a parameter to only allow locations ahead of possessor
def top_loc():

    global op_loc_point_vals, op_n, al_n, possessor_info

    # Select the top al_n*3 positions (1/ally on field)
    top_loc_values = []
    for i in range((al_n-1) * 3):
        top_loc_values.append(0)

    # DEBUG:
    # print(str(top_loc_values))

    pos = []
    for i in range((al_n-1) * 6):
        pos.append(0)
    # DEBUG:
    # print(str(pos))

    n = 0
    for i in range(field_bounds[1]+1):
        for j in range(field_bounds[3] + 1):
            if n == 0 and op_point_val_check(i, j) > (op_n - 1) and j >= possessor_info[2]:
                top_loc_values[n] = op_point_val_check(i, j)
                pos[n] = i
                pos[n+1] = j
                n = n+1
            elif n == (al_n-1) * 3:
                # Check point proximity to those already checked or too close to possessor in following block and flag to skip point if flag if tripped
                flag = 1
                prox_possessor = point_prox(i, j, possessor_info)
                for k in range(n):
                    prox = point_prox(i, j, [0, pos[k * 2], pos[k * 2 + 1]])
                    if prox[0] < 4 or prox_possessor[0] < 4 or j < possessor_info[2]:
                        flag = 0

                    # Flag locations too close to possessor***
                    if op_point_val_check(i, j) > top_loc_values[k] and flag:
                        top_loc_values[k] = op_point_val_check(i, j)
                        pos[k*2] = i
                        pos[k*2+1] = j
            elif n != 0:
                # Check point proximity to those already checked or too close to possessor in following block and flag to skip point if flag if tripped
                flag = 1
                prox_possessor = point_prox(i, j, possessor_info)
                for k in range(n):
                    prox = point_prox(i, j, [0, pos[k * 2], pos[k * 2 + 1]])
                    if prox[0] < 4 or prox_possessor[0] < 4 or j < possessor_info[2]:
                        flag = 0


                if op_point_val_check(i, j) > (op_n - 1) and flag:
                    top_loc_values[n] = op_point_val_check(i, j)
                    pos[n*2] = i
                    pos[n*2+1] =j
                    n = n+1

    # DEBUG
    # print(str(top_loc_values))
    return pos

def ser_val_loc_allocation(i, receiver_info, coords_arr_grd):

    global ser_check_vals

    for j in range(rob_num):
        if coords_arr_grd[j][0] == ball_possesor_id:
            if i == 0:
                coords_arr_grd[j][1] = coords_arr_grd[j][1] + 2
                ang = point_prox(coords_arr_grd[j][1], coords_arr_grd[j][2], [0, net_location[0], net_location[1]])
                coords_arr_grd[j][3] = ang[1]
            elif i == 1:
                coords_arr_grd[j][1] = coords_arr_grd[j][1] - 2
                ang = point_prox(coords_arr_grd[j][1], coords_arr_grd[j][2], [0, net_location[0], net_location[1]])
                coords_arr_grd[j][3] = ang[1]
            elif i == 2:
                coords_arr_grd[j][2] = coords_arr_grd[j][2] + 2
                ang = point_prox(coords_arr_grd[j][1], coords_arr_grd[j][2], receiver_info)
                coords_arr_grd[j][3] = ang[1]
            else:
                coords_arr_grd[j][2] = coords_arr_grd[j][2] - 2
                ang = point_prox(coords_arr_grd[j][1], coords_arr_grd[j][2], receiver_info)
                coords_arr_grd[j][3] = ang[1]

def greater_loop_1(coords_arr_grd):

    global ball_possesor_id, rob_num, ser_check_vals     # Aquire necessary global variables

    count_rob(coords_arr_grd)

    for i in range(rob_num):
        # DEBUG:
        # print(str(coords_arr_grd[i][0] == ball_possesor_id))

        #All decisions related to the ball possesor
        if coords_arr_grd[i][0] == ball_possesor_id:

            # Check if a shot can be taken from current location
            shot_stat = 0       # Initialize shot status flag (must reach op_n to allow a shot)
            for j in range(rob_num):

                # DEBUG:
                # print(str(j))
                # print(str(coords_arr_grd[i][0] != coords_arr_grd[j][0]))
                # print(str(not(coords_arr_grd[j][0] <= 16)))

                if coords_arr_grd[i][0] != coords_arr_grd[j][0] and not(coords_arr_grd[j][0] <= 16):
                    shot_stat = shot_stat + check_LOS_net(coords_arr_grd[i], coords_arr_grd[j])
                # DEBUG:
                # print(str(shot_stat))

            if shot_stat == op_n:
                print("SHOOT")
                ang = point_prox(possessor_info[1], possessor_info[2], [0, net_location[0], net_location[1]])
                for j in range(rob_num):
                    if coords_arr_grd[j][0] == ball_possesor_id:
                        coords_arr_grd[j][3] = ang[1]
                return coords_arr_grd
            # *******************************************************************
            # Check if a pass can be made from current location
            pass_stat = 0       # Initialize pass status flag (must reach op_n to allow a pass)
            for j in range(rob_num):
                # Check if allied id and not passer
                if coords_arr_grd[i][0] != coords_arr_grd[j][0] and coords_arr_grd[j][0] <= 16:
                    receiver_info = coords_arr_grd[j]
                    for k in range(rob_num):
                        # Check if opponent is in way of receiver
                        if not(coords_arr_grd[k][0] <= 16):
                            pass_stat = pass_stat + check_LOS_al(coords_arr_grd[i], coords_arr_grd[j], coords_arr_grd[k])

                            # DEBUG:
                            # print(str(pass_stat))

            if pass_stat == op_n:
                print("PASS")
                ang = point_prox(possessor_info[1], possessor_info[2], receiver_info)
                for j in range(rob_num):
                    if coords_arr_grd[j][0] == ball_possesor_id:
                        coords_arr_grd[j][3] = ang[1]
                    elif coords_arr_grd[j][0] == receiver_info[0]:
                        coords_arr_grd[j][3] = ang[1] + 180
                return coords_arr_grd
            # *******************************************************************
            # Check if immediate surroundings offer a better position from which to shoot or pass
            for j in range(rob_num):
                ser_check_vals = [0, 0, 0, 0]
                # Check if not possessor and allied
                if coords_arr_grd[i][0] != coords_arr_grd[j][0] and coords_arr_grd[j][0] <= 16:
                    # For each passable ally, run ser check value (will pick first possible option as of right now)
                    receiver_info = []
                    for k in range(rob_num):
                        if not(coords_arr_grd[k][0] <= 16):
                            ser_check(coords_arr_grd[i], coords_arr_grd[j], coords_arr_grd[k])
                            receiver_info = coords_arr_grd[j]

                        # DEBUG:
                        # print(str(ser_check_vals))


                    for i in range(4):
                        if ser_check_vals[i] == op_n:
                            ser_val_loc_allocation(i, receiver_info, coords_arr_grd)
                            print("SUR_CHECK")
                            return coords_arr_grd


            # ********************************************************************
            # Fail criteria
            return 'LAST RESORT: SHOOT'

def greater_loop_2(coords_arr_grd):

    global possessor_info, ball_possesor_id         # Aquire necessary global variables

    #Following statement finds ball possessor and runs the optimal receiving location function using his information
    for i in range(rob_num):
        if coords_arr_grd[i][0] > 16:
            op_rec_loc(possessor_info, coords_arr_grd[i])

    top_locations = top_loc()       # Finds the top locations based on the optimal locations field sweep

    # Initiate an array in which to allocate a location to a robot
    allocation_arr = []
    for i in range((al_n-1)*3):
        allocation_arr.append(1)


    clossest_point = [0, 0]         # Initiate an array for clossest points to the robot in question for the loop
    for i in range((al_n-1)*3):
        used_flag = -1          # Set a flag to be tripped if a location has been chosen for a robot so as not to double assign locations to more than one robot
        prox_old = [100, 0]         # Initiate comparison point
        for j in range(rob_num):
            if allocation_arr[i] != 0 and coords_arr_grd[j][0] != ball_possesor_id and coords_arr_grd[j][0] <= 16:      #Chack that location has not already been assigned from top locations & not applying function to ball possessor & that robot is ally
                prox_new = point_prox(top_locations[i*2], top_locations[i*2+1], coords_arr_grd[j])      # Compare each point to one another and determine which is clossest
                if prox_new[0] < prox_old[0]:       # If point is closer than previous
                    used_flag = i       # Indicate the used point
                    clossest_point[0] = top_locations[i*2]      # Grab the x coordinate
                    clossest_point[1] = top_locations[i*2+1]        # Grab the y coordinate

                prox_old = prox_new     # Remember previous point for comparison to next

                # DEBUG
                # print(str(clossest_point))

                coords_arr_grd[j][1] = clossest_point[0]        # Indicate new location for robot (xcoord)
                coords_arr_grd[j][2] = clossest_point[1]        # Indicate new location for robot (ycoord)

    allocation_arr[used_flag] = 0       # Trip the allocation_arr at the used_flag location so as not to double assign locations to two robots

    return coords_arr_grd       # Return the updater array with new robot locations

# SPECIFY GAME STRATEGY:

game_strat = "OFF"                              # Specify offensive or defensive strategy (OFF/DEF)

# GENERAL VARIABLES:
pxl_size = (480, 640)                           # Pixel size of camera view
field_bounds = [0, 10, 0, 13]                   # Specify field boundaries
net_location = (5.5, 13)                        # Specify net location on the grid
pxl_grd_ratio = grid_to_pxl_ratio()             # Determine the multiplication factor necessary to move from grid coordinates to pixel coordinates

# DEBUG
# print("\nGrid to pixel ratio [x, y]:")
# print(str(pxl_grd_ratio)+"\n")

# ALLIES AND OPPONENT INFORMATION: (TESTING PURPOSES)

# CASE 1
a1 = [1, 2, 4, 60]
a2 = [2, 9, 6, 82]

o1 = [17, 3, 5, 30]
# o2 = [18, 7, 10, 148]
o3 = [19, 7, 3, 130]

coords_arr = [a1, a2, o1, o3]       # Create a matrix with all robot information
# coords_arr_grd = [a1, a2, o1, o2, o3]

# NON-USER DEFINED VARIABLES:
ser_check_vals = [0, 0, 0, 0]                   # Modifiable array in ser_check values
point_vals = bounds_to_arr(field_bounds)        # Modifiable array in field_sweep function
op_loc_point_vals = bounds_to_arr(field_bounds) # Modifiable array in op_rec_loc function
possession_flag = 1                             # A flag to determine weather our team is in possession of the ball
ball_possesor_id = coords_arr[0][0]             # remember id of the robot with possession of the ball
possessor_info = coords_arr[0]
rob_num = 0                                     # Remember number of robots identified on field
al_n = 0
op_n = 0

# FUNCTION TESTING:
# print("Testing count_rob and count_al_op function:")
count_rob(coords_arr)
count_al_op(coords_arr)
# print(str(rob_num))
# print(str(al_n))
# print(str(op_n)+"\n")

# print("Testing pxl_2_grd function:")
# print("Non adjusted array = \n" + str(coords_arr))
# check9 = pxl_2_grid(coords_arr)
# print("Adjusted array = \n" + str(check9))
# check10 = grd_2_pxl(check9)
# print("Return array is = \n" + str(check10) + "\n")
#
# print("Testing point_prox function:")
# check1 = point_prox(2, 6, o1)
# print(str(check1)+"\n")
#
# print("Testing check_LOS_net function:")
# check2 = check_LOS_net(check9[0], o1)
# print(str(check2)+"\n")
#
# print("Testing check_LOS_al function:")
# check3 = check_LOS_al(a1, a2, o1)
# print(str(check3) + "\n")
#
# print("Testing check_LOS_al function:")
# check4 = check_LOS_al(a1, a2, o2)
# print(str(check4)+"\n")
#
# print("Testing check_LOS_al function:")
# check5 = check_LOS_al(a1, a2, o3)
# print(str(check5)+"\n")
#
# print("Testing field_sweep function:")
# field_sweep(o1)
# field_sweep(o2)
# field_sweep(o3)
# print(str(point_vals)+"\n")
#
# print("Testing point_val_check function:")
# check7 = point_val_check(2, 6)
# print(str(check7)+"\n")
#
# print("Testing ser_check function:")
# ser_check(a1, a2, o1)
# ser_check(a1, a2, o2)
# ser_check(a1, a2, o3)
# print(str(ser_check_vals)+"\n")
#
# print("Testing op_rec_loc function:")
# op_rec_loc(a1, o1)
# op_rec_loc(a1, o2)
# op_rec_loc(a1, o3)
# print(str(op_loc_point_vals)+"\n")
# print(str)
#
# print("Testing top_loc function:")
# check8 = top_loc()
# print(str(check8)+"\n")



# NESTING TESTING:



# DEMO *****************************************************************************************************************
# # CASE 1
# a1 = [1, 2, 4, 60]
# a2 = [2, 9, 6, 82]
#
# o1 = [17, 2, 10, 30]
# o2 = [18, 7, 10, 148]
# o3 = [19, 7, 3, 130]
#
# coords_arr = [a1, a2, o1, o2, o3]       # Create a matrix with all robot information
#
# CASE 2 ***************************************************************************
# a1 = [1, 2, 4, 60]
# a2 = [2, 9, 6, 82]
#
# o1 = [17, 3, 10, 30]
# o2 = [18, 7, 10, 148]
# o3 = [19, 7, 3, 130]
#
# coords_arr = [a1, a2, o1, o2, o3]       # Create a matrix with all robot information
#
# CASE 3 ***************************************************************************
# a1 = [1, 2, 4, 60]
# a2 = [2, 9, 6, 82]
#
# o1 = [17, 3, 10, 30]
# o2 = [18, 7, 10, 148]
# o3 = [19, 7, 6, 130]
#
# coords_arr = [a1, a2, o1, o2, o3]       # Create a matrix with all robot information
#
# CASE 4 ***************************************************************************
# a1 = [1, 2, 4, 60]
# a2 = [2, 9, 6, 82]
#
# o1 = [17, 4, 7, 30]
# o2 = [18, 7, 10, 148]
# o3 = [19, 7, 6, 130]
#
# coords_arr = [a1, a2, o1, o2, o3]       # Create a matrix with all robot information
#
# CASE 5 ***************************************************************************
# a1 = [1, 2, 4, 60]
# a2 = [2, 9, 6, 82]
#
# o1 = [17, 5, 11, 30]
# o2 = [18, 7, 10, 148]
# o3 = [19, 4, 4, 130]
#
# coords_arr = [a1, a2, o1, o2, o3]       # Create a matrix with all robot information
#
# CASE 6 ***************************************************************************
# a1 = [1, 2, 4, 60]
# a2 = [2, 9, 6, 82]
#
# o1 = [17, 5, 11, 30]
# o2 = [18, 7, 10, 148]
# o3 = [19, 4, 6, 130]
#
# coords_arr = [a1, a2, o1, o2, o3]       # Create a matrix with all robot information
#
# CASE 7 ***************************************************************************
# a1 = [1, 2, 4, 60]
# a2 = [2, 9, 6, 82]
#
# o1 = [17, 5, 11, 30]
# o2 = [18, 7, 10, 148]
# o3 = [19, 4, 5, 130]
#
# coords_arr = [a1, a2, o1, o2, o3]       # Create a matrix with all robot information

print(str(coords_arr))
coords_arr = greater_loop_1(coords_arr)
print(str(coords_arr))
print(str(greater_loop_2(coords_arr)))