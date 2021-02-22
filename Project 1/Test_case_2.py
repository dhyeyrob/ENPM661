import time
start_time = time.time()                                              # note the start time
m_file = open('Test_case_2.txt', "w+")                                # open text file

#  A function to convert list into string
def lis_2_str(mat):

    string = ''
    list_temp = []
    for i in range(len(mat)):
        for j in range(len(mat)):
            string = string + str(mat[i][j])
    return string

#  A function to solve the given puzzle
def path_finder(start_pos):

    grid = start_pos.copy()
    for k in range(len(grid)):                                         # Converting 10,11,12,13,14,15 to A,B,C,D,E,F
        for m in range(len(grid)):
            if grid[k][m] < 10:
                grid[k][m] = grid[k][m]
            elif grid[k][m] == 10:
                # print("found 10")
                grid[k][m] = 'A'
            elif grid[k][m] == 11:
                grid[k][m] = 'B'
            elif grid[k][m] == 12:
                grid[k][m] = 'C'
            elif grid[k][m] == 13:
                grid[k][m] = 'D'
            elif grid[k][m] == 14:
                grid[k][m] = 'E'
            elif grid[k][m] == 15:
                grid[k][m] = 'F'

    final_pos = [[1, 2, 3, 4],                                       # Defining our final position
                 [5, 6, 7, 8],
                 [9, 'A', 'B', 'C'],
                 ['D', 'E', 'F', 0]]

    final_pos_str = lis_2_str(final_pos)
    start_pos_str = lis_2_str(grid)
    mov_fn = [[0, 1],                                                # Defining our move function
              [0, -1],
              [1, 0],
              [-1, 0]]

    lis_mat = []
    lis_mat.append(grid)                                             # Creating a list to append matrices of all nodes
    lis_mat_str = []
    lis_mat_str.append(start_pos_str)
    goal_pos_found = False
    lis_mat2 = []
    lis_mat2.append(grid)
    while goal_pos_found == False:                                   # while loop will continue until puzzle is solved

        start_mat = lis_mat.pop(0)                                   # popping out the first matrix from the list
        init = []
        for k in range(len(start_mat)):                              # Finding out the location of zero in the matrix
            for l in range(len(start_mat)):
                if start_mat[k][l] == 0:
                    init.append(k)
                    init.append(l)

        for i in range(len(mov_fn)):
            if goal_pos_found == False:
                temp_mat = []

                for d in range(len(start_mat)):
                    s = []
                    for f in start_mat[d]:
                        s.append(f)
                    temp_mat.append(s)
                xn = init[0] + mov_fn[i][0]                             # steps to move down or up
                yn = init[1] + mov_fn[i][1]                             # steps to move right or left

                if xn >= 0 and xn < len(grid) and yn >= 0 and yn < len(grid):  # check validity of new zero location
                    x_old = init[0]
                    y_old = init[1]

                    temp = temp_mat[x_old][y_old]
                    temp_mat[x_old][y_old] = temp_mat[xn][yn]
                    temp_mat[xn][yn] = temp

                    temp_str = lis_2_str(temp_mat)                      # converting the new matrix into string
                    n = 0
                    for m in range(len(lis_mat_str)):
                        if temp_str == lis_mat_str[m]:                  # checking whether new zero location has been
                            n = n + 1                                   # visited earlier or not

                    if n == 0:
                        if temp_str == final_pos_str:                   # If it is unique then compare with final pos
                            solved_puzzle = []
                            solved_puzzle_str = []
                            goal_pos_found = True                       # If it matches with goal then turn flag = True
                            solved_puzzle = temp_mat                    # Append matirx in the list of matrices
                            solved_puzzle_str = temp_str
                            lis_mat.append(temp_mat)                    # Append string in the list of strings
                            lis_mat2.append(temp_mat)                   # Storing the path of matrices in a list

                        else:
                            lis_mat_str.append(temp_str)                # Append string in the list of strings
                            lis_mat.append(temp_mat)                    # Append matirx in the list of matrices
                            lis_mat2.append(temp_mat)

    #  Writing on our text file to display the initia and final state as well as the path followed
    m_file.write(f"The initial matrix was {grid} and the solved puzzle is {solved_puzzle} \n\n")
    m_file.write("The path followed to reach final position is \n")
    for o in range(len(lis_mat2)):
        m_file.write("\n\n")
        for p in range(4):
            m_file.write("%s\n" %lis_mat2[o][p])

    # return solved puzzle and path of matrix
    return solved_puzzle, lis_mat2

solved_mat, path = path_finder([[1, 0, 3, 4],[ 5, 2, 7, 8], [9, 6, 10, 11] , [13, 14, 15, 12]])

print("solved matrix",solved_mat)                             # Displaying solved matrix
print("path taken is:")                                       # Displaying the path from initial to final position
for o in range(len(path)):
    print(" ")
    for p in range(4):
        print(path[o][p])
end_time = time.time()
print("time taken:", start_time - end_time)                  # Displaying time taken