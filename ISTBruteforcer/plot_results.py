import os
import traceback
import numpy as np

import matplotlib
import matplotlib.colors as clrs
from matplotlib import pyplot as plt


def find_latest_timestamped_directory(directory) -> str :
    latest_directory = ""
    latest_timestamp = None
    
    for dir_name in os.listdir(directory):
        if os.path.isdir(os.path.join(directory, dir_name)):
            timestamp_parts = map(int, dir_name.split('_'))
            timestamp = tuple(timestamp_parts)
            
            if latest_timestamp is None or timestamp > latest_timestamp:
                latest_timestamp = timestamp
                latest_directory = dir_name
    
    return latest_directory


def parse_run_info(directory):
    with open(directory + "runInformation.txt") as ri:
        lines = ri.readlines()
        for line in lines:
            try:
                line_value = line.split(":")[1].strip()
            except Exception:
                continue

            if line.startswith("Norm Min Y"):
                norm_min_y = float(line_value)

            if line.startswith("Norm Min X"):
                norm_min_x = float(line_value)

            if line.startswith("Norm Min Z"):
                norm_min_z = float(line_value)

            if line.startswith("Norm Num Y"):
                norm_num_y = int(line_value)

            if line.startswith("Norm Num X"):
                norm_num_x = int(line_value)

            if line.startswith("Norm Num Z"):
                norm_num_z = int(line_value)

            if line.startswith("Granularity Y"):
                gran_y = float(line_value)

            if line.startswith("Granularity X"):
                gran_x = float(line_value)

            if line.startswith("Granularity Z"):
                gran_z = float(line_value)

    return (norm_min_y, norm_min_x, norm_min_z),\
            (norm_num_y, norm_num_x, norm_num_z),\
            (gran_y, gran_x, gran_z)


def parse_normal_stages(directory, norm_counts):
    with open(directory + "normalStagesReached.bin", mode='rb') as file: # b is important -> binary
        fileContent = file.read()

    int_values_array = np.array([i for i in fileContent])
    return np.reshape(int_values_array, (norm_counts[0], norm_counts[1], norm_counts[2]))


def update_image_plot(implot, img, pauseRate : float, colmap : clrs.LinearSegmentedColormap, title=''):
    implot.set_array(img)
    implot.set(cmap=colmap)
    plt.title(title)
    plt.pause(pauseRate)
    return implot


# cdict = {'red':   [[0.0,  0.0, 0.0],
#                    [0.499,  0.0, 0.0],
#                    [0.5,  1.0, 1.0],
#                    [1.0,  1.0, 1.0]],
#          'green': [[0.0,  0.0, 0.0],
#                    [0.832, 0.0, 0.0],
#                    [0.833, 1.0, 1.0],
#                    [1.0,  1.0, 1.0]],
#          'blue':  [[0.0,  0.0, 0.0],
#                    [0.166,  0.0, 1.0],
#                    [0.167,  1.0, 1.0],
#                    [1.0,  1.0, 1.0]]}

num_stages = 4

cdict = {'red':   [[0.0,  0.0, 0.0],
                   [0.374,  0.0, 0.0],
                   [0.375,  0.5, 0.5],
                   [0.624,  0.5, 0.5],
                   [0.625,  1.0, 1.0],
                   [1.0, 1.0, 1.0]],
         'green': [[0.0,  0.0, 0.0],
                   [0.874, 0.0, 0.0],
                   [0.875, 1.0, 1.0],
                   [1.0, 1.0, 1.0]],
         'blue':  [[0.0, 0.0, 0.0],
                   [0.124, 0.0, 1.0],
                   [0.125, 1.0, 1.0],
                   [1.0, 1.0, 1.0]]}

colormap = matplotlib.colors.LinearSegmentedColormap('fractal', segmentdata=cdict)

if __name__ == "__main__":
    # Load run (loop until successful)
    while True:
        path = input("Enter run timestamp (or just press enter for latest run; also 'q' quits): ")

        if path.strip() == "q":
            exit(0)

        if path.strip() == "":
            path = find_latest_timestamped_directory("./output/")

        dir_name = "./output/" + path
        if not dir_name.endswith("/") or dir_name.endswith("\\"):
            dir_name += "/"

        # Try to load the run info and normal stages files
        try:
            (minNY, minNX, minNZ), norm_counts, (gran_y, gran_x, gran_z) = parse_run_info(dir_name)
            normalsArr = parse_normal_stages(dir_name, norm_counts)
            break
        except Exception as e:
            print("Could not load file; try entering path again!")
            print(e)
            traceback.print_exc()
        
    maxNX = minNX + norm_counts[1] * gran_x
    maxNZ = minNZ + norm_counts[2] * gran_z
    pauseRate = 0.01

    # Show sample or play animation (loop until user quits)    
    while True:
        sample = input("Enter a sample for NY (0 indexed), a valid command (type 'help' for a list), or -1 to quit: ")

        if sample == '-1' or sample == 'quit':
            break

        if sample == 'video' or sample.strip() == "":

            plt.figure(1); plt.clf()

            implot = plt.imshow(normalsArr[0,:,:].transpose(), 
                            cmap=colormap, interpolation='nearest', origin='upper', 
                            extent=[minNX - gran_x/2, maxNX + gran_x/2,\
                                maxNZ + gran_z/2, minNZ - gran_z/2], 
                            vmin=0, vmax=num_stages)
            
            plt.xlabel("nX")
            plt.ylabel("nZ")
            
            plt.xticks(np.arange(minNX, maxNX + gran_x, gran_x * 10))
            plt.yticks(np.arange(maxNZ, minNZ - gran_z, -gran_z * 10))

            cbar = plt.colorbar()
            cbar.ax.locator_params(nbins=4)

            for ny in range(norm_counts[0]):
                title_str = 'nY = ' + str(round(ny * gran_y + minNY, 3))
                implot = update_image_plot(implot, normalsArr[ny,:,:].transpose(),\
                                           pauseRate, colormap, title_str)
            continue

        if sample == 'help':
            print("\nThis program plots per-normal data collected from the BitFS IST bruteforcer.\n\nValid commands:\n====================")
            print('0,1,...    - Displays a single nY slice of the data based on the index entered. The input cannot ')
            print('             exceed the number of nY samples - 1.')
            print('video      - Displays all nY samples in a continuous video, with the frame delay based on the speed')
            print('             of the plotter and the pause rate (typing nothing and hitting enter will do the same).')
            print('chpr <f>   - Changes the pause rate to the input (pause rate is stored as a floating-point variable).')
            print('quit       - Quits the application (typing -1 will do the same).')
            print('help       - Prints this menu.')
            print()

            continue

        if sample.startswith('chpr '):
            try:
                if(float(sample[5:].strip()) > 0):
                    pauseRate = float(sample[5:].strip())
                    print("Pause rate changed to:", pauseRate)
                else:
                    raise ValueError("Pause value must be positive.")
            except:
                print("Invalid pause rate entered; could not change pause rate.")

            continue

        if not sample.isdigit():
            print("Sample is not an integer! Please enter an integer sample!\n")
            continue

        sample = int(sample)
        if sample >= norm_counts[0]:
            print("Sample index is too high! Please enter a lower sample index!\n")
            continue

        plt.imshow(normalsArr[sample,:,:].transpose(),\
                cmap=colormap, interpolation='nearest', origin='upper',\
                extent=[minNX - gran_x/2, maxNX + gran_x/2,\
                        maxNZ + gran_z/2, minNZ - gran_z/2], 
                    vmin=0, vmax=num_stages)
        
        plt.xlabel("nX")
        plt.ylabel("nZ")
        plt.title('nY = ' + str(sample * gran_y + minNY))

        plt.xticks(np.arange(minNX, maxNX + gran_x, gran_x * 10))
        plt.yticks(np.arange(maxNZ, minNZ - gran_z, -gran_z * 10))
        
        cbar = plt.colorbar()
        cbar.ax.locator_params(nbins=4)

        plt.show()
