# IMPORTED LIBRARIES
import math

# FUNCTIONS:
def grid_to_pxl_ratio():

    global pxl_size, field_bounds

    x_rat = pxl_size[0]/field_bounds[1]
    y_rat = pxl_size[1]/field_bounds[3]

    return [x_rat, y_rat]

def pxl_2_grid():

    global coords_arr, pxl_grd_ratio

    rob_num = al_n + op_n

    coords_arr_grd = []        # Initiate empty matrix
    for i in range(rob_num):
        dummy_arr = [0, 0, 0, 0]      # Initiate dummy arr for appending
        dummy_arr[0] = coords_arr[i][0]

        # Modulus code to determine weather to ceil() or floor() to round x coordinate to nearest grid position
        if (coords_arr[i][1] % pxl_grd_ratio[0] > field_bounds[1] / 2):
            dummy_arr[1] = math.ceil(coords_arr[i][1]/pxl_grd_ratio[0])
        else:
            dummy_arr[1] = math.floor(coords_arr[i][1] / pxl_grd_ratio[0])

        # Modulus code to determine weather to ceil() or floor() to round y coordinate to nearest grid position
        if coords_arr[i][2] % pxl_grd_ratio[1] > field_bounds[3] / 2:
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
        if prox[0] <= 1 / 2:
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
        if prox[0] <= 1 / 2:
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

    aIDInfoOpt = [aIDInfoC[1] + 1, aIDInfoC[2], aIDInfoC[1] - 1, aIDInfoC[2], aIDInfoC[1], aIDInfoC[2] + 1, aIDInfoC[1], aIDInfoC[2] - 1]       # Create a reference for points serrounding the robot

    aIDInfoC1 = [1, aIDInfoC[1] + 1, aIDInfoC[2]]       # Point located at coordinate (x+1, y) w.r.t. current robot position
    aIDInfoC2 = [2, aIDInfoC[1] - 1, aIDInfoC[2]]       # Point located at coordinate (x-1, y) w.r.t. current robot position
    aIDInfoC3 = [3, aIDInfoC[1], aIDInfoC[2] + 1]       # Point located at coordinate (x, y+1) w.r.t. current robot position
    aIDInfoC4 = [4, aIDInfoC[1], aIDInfoC[2] - 1]       # Point located at coordinate (x, y-1) w.r.t. current robot position

    # Each of the following blocks below will evaluate one of the respective points above in the order listed. Done by
    # ensuring first that the position is within the bounds of the field limits, then checking if its risk flag
    # (point_vals) was tripped, and lastly the line of sight (net/opponent depending on location) is checked before its
    # value is either incremented (good location) or left unaltered (poor location) for the specified task.

    # Check location east or robot for LOS to net
    i = 0
    if (aIDInfoOpt[i * 2] < 6) and not(point_val_check(aIDInfoOpt[i * 2], aIDInfoOpt[i * 2 + 1])) and check_LOS_net(aIDInfoC1, oIDInfo):
        ser_check_vals[i] = ser_check_vals[i] + 1
        # DEBUG
        # print"1++")

    # Check location west of robot for LOS to net
    i = i + 1
    if (aIDInfoOpt[i * 2] >= 0) and not(point_val_check(aIDInfoOpt[i * 2], aIDInfoOpt[i * 2 + 1])) and check_LOS_net(aIDInfoC2, oIDInfo):
        ser_check_vals[i] = ser_check_vals[i] + 1
        # DEBUG
        # print('\n2++\n')

    # Check location north of robot for LOS to net
    i = i + 1
    if (aIDInfoOpt[i * 2 + 1] < 5) and not(point_val_check(aIDInfoOpt[i * 2], aIDInfoOpt[i * 2 + 1])) and check_LOS_al(aIDInfoC3, aIDInfo2, oIDInfo):
        ser_check_vals[i] = ser_check_vals[i] + 1
        # DEBUG
        # print('\n3++\n')

    # Check location south of robot for LOS to net
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
                op_loc_point_vals[i * (field_bounds[3] + 1) + j] = op_loc_point_vals[i * (field_bounds[3] + 1) + j] + 1     # Increase the locations "value"


    # Check proximity of each point to opponents
    for i in range(field_bounds[1] + 1):        # For each x point on grid
        for j in range(field_bounds[3] + 1):        # For each x point on grid
            prox = point_prox(i, j, oIDInfo)        # Check the proximity of the point
            if prox[0] <= 1:        # If within a specified unit
                op_loc_point_vals[i * (field_bounds[3] + 1) + j] = op_loc_point_vals[i * (field_bounds[3] + 1) + j]*0.75        # Reduce the points "value" by 75%
            elif prox[0] < 2:       # If within two specified unit
                op_loc_point_vals[i * (field_bounds[3] + 1) + j] = op_loc_point_vals[i * (field_bounds[3] + 1) + j] * 0.9       # Reduce the points "value" by 90%

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

# May need to include a parameter to seperate the receiver from the passing robot
def top_loc():

    global op_loc_point_vals, op_n, al_n

    # Select the top ____ positions (1/ally on field)
    top_loc_values = []
    for i in range((al_n-1) * 2):
        top_loc_values.append(0)
    # DEBUG:
    # print(str(top_loc_values))

    pos = []
    for i in range((al_n-1) * 4):
        pos.append(0)
    # DEBUG:
    # print(str(pos))

    n = 0
    for i in range(field_bounds[1]+1):
        for j in range(field_bounds[3] +1):
            if n == 0 and op_point_val_check(i, j) > (op_n - 1):
                top_loc_values[n] = op_point_val_check(i, j)
                pos[n] = i
                pos[n+1] = j
                n = n+1
            elif n == (al_n-1) * 2:
                for k in range(n):
                    prox = point_prox(i, j, [0, pos[k*2], pos[k*2+1]])
                    if op_point_val_check(i, j) > top_loc_values[k] and prox[0] > 2:
                        top_loc_values[k] = op_point_val_check(i, j)
                        pos[k*2] = i
                        pos[k*2+1] = j
            else:
                # Check point proximity to those already checked in following block and flag to skip subsequenf if statement if tripped
                flag = 1
                for k in range(n):
                    prox = point_prox(i, j, [0, pos[k * 2], pos[k * 2 + 1]])
                    if prox[0] < 3:
                        flag = 0

                if op_point_val_check(i, j) > (op_n - 1) and flag:
                    top_loc_values[n] = op_point_val_check(i, j)
                    pos[n*2] = i
                    pos[n*2+1] =j
                    n = n+1

    # DEBUG
    # print(str(top_loc_values))
    return pos

# UNDER DEVELOPEMENT********
#
# def assign_new_pos(aIDInfo):
#
#
# **************************

# SPECIFY GAME STRATEGY:

game_strat = "OFF"                              # Specify offensive or defensive strategy (OFF/DEF)

# GENERAL VARIABLES:
pxl_size = (480, 640)                           # Pixel size of camera view
field_bounds = [0, 10, 0, 13]                   # Specify field boundaries
net_location = (3, 6)                           # Specify net location on the grid
al_n = 2                                        # Number of allies on field
op_n = 3                                        # Number of opponents on field
pxl_grd_ratio = grid_to_pxl_ratio()             # Determine the multiplication factor necessary to move from grid coordinates to pixel coordinates

# DEBUG
print("\nGrid to pixel ratio [x, y]:")
print(str(pxl_grd_ratio)+"\n")

# ALLIES AND OPPONENT INFORMATION: (TESTING PURPOSES)
a1 = [1, 480/2, 640/2, 60]
a2 = [2, 5, 5, 82]

o1 = [1, 2, 5, 30]
o2 = [2, 4, 3, 148]
o3 = [3, 4, 4, 130]

coords_arr = [a1, a2, o1, o2, o3]       # Create a matrix with all robot information

# NON-USER DEFINED VARIABLES:
ser_check_vals = [0, 0, 0, 0]                   # Modifiable array in ser_check values
point_vals = bounds_to_arr(field_bounds)        # Modifiable array in field_sweep function
op_loc_point_vals = bounds_to_arr(field_bounds) # Modifiable array in op_rec_loc function
possession_flag = 0                             # A flag to determine weather our team is in possession of the ball

# FUNCTION TESTING:
print("Testing point_prox function:")
check1 = point_prox(2, 6, o1)
print(str(check1)+"\n")

print("Testing check_LOS_net function:")
check2 = check_LOS_net(a1, o1)
print(str(check2)+"\n")

print("Testing check_LOS_al function:")
check3 = check_LOS_al(a1, a2, o1)
print(str(check3) + "\n")

print("Testing check_LOS_al function:")
check4 = check_LOS_al(a1, a2, o2)
print(str(check4)+"\n")

print("Testing check_LOS_al function:")
check5 = check_LOS_al(a1, a2, o3)
print(str(check5)+"\n")

print("Testing field_sweep function:")
field_sweep(o1)
field_sweep(o2)
field_sweep(o3)
print(str(point_vals)+"\n")

print("Testing point_val_check function:")
check7 = point_val_check(2, 6)
print(str(check7)+"\n")

print("Testing ser_check function:")
# ser_check(a1, a2, o1)
# ser_check(a1, a2, o2)
# ser_check(a1, a2, o3)
print(str(ser_check_vals)+"\n")

print("Testing op_rec_loc function:")
op_rec_loc(a1, o1)
op_rec_loc(a1, o2)
op_rec_loc(a1, o3)
print(str(op_loc_point_vals)+"\n")
# print(str)

print("Testing top_loc function:")
check8 = top_loc()
print(str(check8)+"\n")

print("Testing pxl_2_grd function:")
print("Non adjusted array = \n" + str(coords_arr))
check9 = pxl_2_grid()
print("Adjusted array = \n" + str(check9))
check10 = grd_2_pxl(check9)
print("Return array is = \n" + str(check10) + "\n")


# ***MAKE A FUNCTION TO CONVERT THE GRID TO A __________,__________ PIXEL GRID***