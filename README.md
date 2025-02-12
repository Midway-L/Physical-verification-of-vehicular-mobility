# Master_thesis_physical_verification

Master thesis project aims for fusion sensor data and location verification in VANET, based on the tool chain of SUMO, Veins and Carla. They need to be simulated together to untilize their functionalities.

To start with, there is installation procedure for the tools.

## Environment: Ubuntu 20.04, Python 3.8.10

## 1. Install OMNET++ 6.0.3 (Build from source)

### See the official site for instruction (Not all are followed in this setting up): https://doc.omnetpp.org/omnetpp/InstallGuide.pdf

### Install required pakages:
```python
sudo apt-get install build-essential clang lld gdb bison flex perl python3 python3-pip qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5opengl5-dev libxml2-dev zlib1g-dev doxygen graphviz libwebkit2gtk-4.0-37 xdg-utils

python3 -m pip install --user --upgrade numpy pandas matplotlib scipy seaborn posix_ipc
```
### Enable the optional parallel simulation support:
```python
sudo apt-get install mpi-default-dev
```
Ubuntu 20.04 has all the packages mentioned in the '6.3.2 Graphical Installation' section, if you are not sure, follow the instructions and check it.

### Download source code and extract to your directory: https://omnetpp.org/download/

`tar xvf omnetpp-6.0.3-linux-x86_64.tgz`

### Set environment variables

Enter your omnetpp-6.0.3 folder and run: `source setenv`

The command only valid for your current terminal window, to permanently set environment varible for omenetpp, you should add a line in your `.bashrc` file. Change the path according to your installation.
```
nano ~/.bashrc
[ -f "$HOME/omnetpp-6.0.3/setenv" ] && source "$HOME/omnetpp-6.0.3/setenv"
```
Then `source ~/.bashrc` to make settings work.

### Configure and build

In your top-level ometpp folder, run: `./configure`

When configuration is finished, run: `make -j4`

### Test installation

If 'make' runs successfully, you can enter the example to verify your installation from top-level of the omentpp folder:
```
cd examples/aloha
./aloha
```
If gui windows and dialogs show up, the installation is done. You can use `omnetpp` command in the terminal to start IDE.

## 2. Install SUMO 1.12.0 (Build from source)

### Build instructions for Linux: https://sumo.dlr.de/docs/Installing/Linux_Build.html

### Download source file of SUMO 1.12.0: https://sumo.dlr.de/docs/Downloads.php

Then extrcact the source to your directory.

### Install required packages:
```
sudo apt-get install git cmake python3 g++ libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev python3-dev swig default-jdk maven libeigen3-dev
sudo apt-get install ccache libavformat-dev libswscale-dev libopenscenegraph-dev
sudo apt-get install libgtest-dev gettext tkdiff xvfb flake8 astyle python3-autopep8 python3-gi-cairo gir1.2-gtk-3.0
sudo apt-get install python3-pyproj python3-rtree python3-pandas python3-pulp
python3 -m pip install texttest
```
To install remainings, navigate to the 'tools' folder and run: `python3 -m pip install -r requirements.txt`

### Set SUMO_HOME environment variables

To add SUMO_HOME permanently to your PATH, add the following to your `.bashrc` file.
```
export SUMO_HOME=/path/to/sumo
export PATH=$SUMO_HOME/bin:$PATH
```
Then `source ~/.bashrc` to reset the envrionment.

Check if SUMO_HOME set corrcetly by `echo $SUMO_HOME`

### Build and make

Building SUMO with cmake requires cmake 3.5 or higher.

In the top-level of SUMO folder, run: `cmake -B build .`

When setting is done, run: `cmake --build build -j $(nproc)`

### Test installation

When 'make' is finished, you can use `sumo-gui` command to start sumo-gui. Installation is successfully once you see the gui coming up.

## 3. Install Veins5.2 and Plexe3.1 (Build from source)

### Build instructions (Plexe is Veins' extension, so Veins build process is included in building Plexe. Plexe's instruction is easy to start with and it works): https://plexe.car2x.org/building/

### Download souce code for Veins5.2 and Plexe3.1 (Not using instant Veins or Plexe in this project, for they are only pre-configured virtual machines)

Veins5.2: https://veins.car2x.org/download/

Plexe3.1: https://plexe.car2x.org/download/

Then extract them seperately to your directory.

### Set environment variables

You can run the `source setenv` secript in both Veins and Plexe folder to temporarily set environment varibles.

For permanently setting, still open '.bashrc' and add the following:
```
export PATH="$PATH":"/path/to/veins/bin"
export PATH="$PATH":"/path/to/plexe/bin"
```
### Configure and Build

Once you have the source code, build Veins first.

In case lack some libaries, install the following:
```
sudo apt-get install build-essential gcc g++ bison flex perl python python3 qt5-default libqt5opengl5-dev tcl-dev tk-dev libxml2-dev zlib1g-dev default-jre doxygen graphviz libwebkit2gtk-4.0-dev libopenscenegraph-dev libosgearth-dev openscenegraph-plugin-osgearth libxerces-c-dev libproj-dev libgdal-dev libfox-1.6-dev libavformat-dev libavcodec-dev libswscale-dev python-dev python-configparser cmake r-base
```
In you Veins folder:

`./configure`

`make -j4`

Once Veins is built, continue in Plexe folder:

`./configure --with-veins=/your/veins/path`

`make -j4`

### Verify installation

When building completed, you can go to /examples/platooning in your Plexe folder and type:

`plexe_run -u Cmdenv -c Sinusoidal -r 2`

sumo-gui would start automatically and remember to press 'Play' to launch to simulation.

### Setup R 

As OMNeT++ 6 dropped the support for the OMNeT++ R plugin, data extraction scripts now work using a mix of R and python scripts.

You need to install some R packages:`install.packages(c('ggplot2', 'data.table'))`

Then download a R package for processing OMNeT++ result files without extracting the archive: http://plexe.car2x.org/download/omnetpp_0.7-1.tar.gz

Install the package: `install.packages("/path/to/your/package/omnetpp_0.7-1.tar.gz", repos=NULL)`

## 4. Install Carla simulator 0.9.15 (Package installation)

The simple way to install Carla is downloading its pre-complied packages, avoiding uneccessary troubles. Just follow the offcial guide of Linux: https://carla.readthedocs.io/en/latest/start_quickstart/

### Check dependencies and download

Before downloading, update your pip (20.3 or higher) and get some dependencies:
```
pip3 install --upgrade pip
pip3 install --user pygame numpy
```
Then download the package in 'B.Package installation' and extract.

Additional assets is not mandatory. 

### Install Carla client

You can just use commands to install Carla client: `pip3 install carla`

### Running Carla

In your Carla folder, simply run:`./CarlaUE4.sh`

Carla simulator would start soon.

Carla simulator is quite computation consuming, if your device performance is not good enough for running Carla, try use following commands which may reduce the computation cost. 

See the official documents: https://carla.readthedocs.io/en/latest/adv_rendering_options.

No-rendering mode: Unreal Engine does not render anything. Graphics are not computed. GPU based sensors return empty data.

Off-screen mode: Unreal Engine is working as usual, rendering is computed but there is no display available. GPU based sensors return data.

Low-quality mode: Reduce the GPU workload.

```
./CarlaUE4.sh -RenderOffScreen
./CarlaUE4.sh -quality-level=Low
```
### Test1: Co-simulation with SUMO

Carla has integrated functionality that supports co-simulation with SUMO. To test it, start Carla as a server first.
```
cd /your/carla/folder
./CarlaUE4.sh
```
In another terminal, load example map.
```
cd /your/carla/folder/PythonAPI/util
python3 config.py --map Town04
```
Then start SUMO.
```
cd /your/carla/folder/Co-Simulation/Sumo
python3 run_synchronization.py examples/Town04.sumocfg  --sumo-gui
```
Once press 'Play' in sumo-gui, the co-simulation is beginning.

## 5. Install veins_carla (Build from source)

Veins_carla is a socket developed by Veins team to make Carla be able to do co-simulation with Veins.

### Download the source code
https://github.com/veins/veins_carla

### Install dependencies
```
python3 -m pip install --user grpcio
python3 -m pip install --user grpcio-tools
python3 -m pip install --user conan==1.54.0
```
Then configure `conan`: 

`conan profile new default --detect && conan profile update settings.compiler.libcxx=libstdc++11 default`

### Build 
You must have build Veins as described before firstly, then in the veins_carla folder, run:
```
./configure --with-veins=/your/veins/path
make -j$(nproc)
```
#### Build Error: 'No such file or directory' for file 'print-veins-version'

That is because the Veins source code downloaded does not contain the file since the version is known. You can download the single file here: https://github.com/sommer/veins. Then put the file in your veins folder.

#### Build Error: Invalid setting
This doesn’t mean that such (e.g., compiler) version is not supported by Conan, it is just that it is not present in the actual defaults settings. You can find in your user home folder ~/.conan/settings.yml

### Test2. Running veins_carla
Carla still needs to be launched as the server first. But no need to put any actors in it `./CarlaUE4.sh`

Then in the veins_carla folder:
```
cd veins_carla/examples/veins_carla
./doRun.sh
```
You should see carla-adpter is starting and messages like 'I got 0 actors' pop up.


## Running options

### 1. Run Carla_SUMO co-simulation
```
cd /your/carla/folder
./CarlaUE4.sh --RenderOffScreen
```
In another terminal, load example map.
```
cd /your/carla/folder/PythonAPI/util
python3 config.py --map Town04
```
Then start SUMO and press 'Play' to launch.
```
cd /your/carla/folder/Co-Simulation/Sumo
python3 run_synchronization.py examples/Town04.sumocfg  --sumo-gui
```
### Run veins_carla
```
cd veins_carla/examples/veins_carla
./doRun.sh
```

### 2. Run veins, carla and sumo as the simulation framework

This is the integrated version of all three simulators, you need to replace the file in the right place in the simulator's folder.

1. veins-carla is modified to solve some bugs and we implement our own applicatrion layer to suit the sychronization requirements for experiment, so the whole folder needs to be replaced and recomplied.

2. For carla-sumo co-simulation, sumo source code remains the same. In carla folder, navigate to Co-simulation/Sumo, then replace the example/ folder which contains the sumo net, rou and cfg file. Then put mytest.py in Co-simulation/Sumo. The script could simply launched just is as described in the Test1. No need for rebuild the source code. 

Control flow:

1. In mytest.py, there is a sychronization loop to sync sumo and carla. Generally, the main synchronization process is managed by the SimulationSynchronization class. Key aspects of the synchronization include:
Initialization: Sets up SUMO and CARLA simulation objects and configures CARLA for synchronous mode with a fixed time step.

Main Synchronization Loop: The tick method handles bidirectional synchronization:
	SUMO to CARLA: Spawns new SUMO vehicles in CARLA, updates existing vehicle states, and syncs traffic light states if configured.
	CARLA to SUMO: Spawns new CARLA vehicles in SUMO, updates existing vehicle states, and syncs traffic light states if configured.
Actor ID Mapping: Maintains dictionaries (sumo2carla_ids and carla2sumo_ids) to track corresponding vehicle IDs between the two systems.
Data Conversion: Utilizes the BridgeHelper class to handle data format conversions between SUMO and CARLA, including coordinate systems and vehicle properties.
Time Synchronization: Ensures both simulations progress at the same rate by waiting after each iteration if necessary.
Cleanup: The close method handles resource cleanup, including destroying synchronized actors and resetting CARLA to asynchronous mode.

2. In veins-carla, we mostly focus on carla-adapter.py and CarlaScenarioManager.cc, which are used to retrive carla client, corresponding actors info and veins operation senario. Key aspects of the synchronization include:
gRPC Interface:A gRPC server allowing Veins to communicate with CARLA via remote procedure calls.
Time Synchronization: The ExecuteOneTimeStep method ensures CARLA and Veins progress in lockstep, waiting for CARLA to complete each simulation tick before allowing Veins to proceed. Veins would call the function continuosly, but only return a valid singal once carla-adpter receive the tick from carla world. The tick is controled by carla-sumo co-simulation. 
Actor Management: Provides methods to retrieve vehicle IDs and detailed information about specific vehicles in the CARLA simulation.
Data Translation: Converts CARLA's internal data structures into protocol buffer messages that Veins can interpret, including vehicle positions.
Event-based Updates: Uses CARLA's tick event system to track simulation progress. Veins would send messages once it receives the carla tick signal. 

3. Basically, veins is like an extension of carla-sumo co-simulation in our project, it passively waits for carla world update (tick signal), and this update is synchronized between sumo and carla. Once carla world updates, veins processes one simulation step accordingly.

Note: Firstly you shoud start carla server, load map (town01, 04 or 05), then run carla-sumo co-simulation script. Waiting it to run for some time, after all the vehicles or actors are sprawned in carla, you shall start veins-carla by executing doRun.sh.

## Scripts usage for data process and result analysis

This section is about the usage of data process scripts. All the data are stored in Json format.

To begin with, we have some raw data file. In Carla simulator, we have saved all the data into a 'vehicle_data' folder, which contains gnss sensor, imu sensor, semantic lidar sensor, original vehicle data, sumo speed, 5 files in total for each vehicle with ID included in the file name. And a sumo distance file. In veins carla, we have std vehcile data (only one is enough) to match the timestamp between simulators, together with the .vec file obtained from omnetpp.

Firstly, copy all the data file and scripts to a folder where you want to start work.

1. parser.py: to process omnetpp .vec file, you would get output_node_0 and output_node_1 for each vehicle, with node0 represents the smaller ID in Carla vehicle. The file contains veins simulation time, message send time, position coordinates in veins, rssi value and message receive time.

Check the std_vehicle_id.json, remove the duplicated data in first line and some lines in the end in order to avoid some issues or mismatched data points caused when the simulation is about ending. Don't forget to remove the same number of lines in the end of output_node_0/1.json, making sure the node file and std vehicle file have the same lines (one line for a single json obeject).

2. overall_process.py: after the std vehicle file, run overall_process.py [vehicle_id] to sort out the vehicle data and all the sensor data, you would get processed_data_[id].json file, which contains the matched timestamp, new timestamp, ground_truth x and y coordinates, lidar gnss and imu sensor data in a single json obeject. Terminal output would tell you how many data points are processed and it should be the same as the numbers in std vehicle file. If not, generally there are less data after processing, it is because the duplicated data points in std vehicle file (besides the first line we have already removed). In this case, run match_timestamp.py processed_data_[id].json std_data_[id].json, this script will tell you what points are duplicated in std vehicle file, you can manauly edit the duplicated ones by checking the timestamp in Carla vehicle data file, which is vehicle_[id].json. 

When the preparations are done, check the calculation scripts and plot scripts, replacing the file name accordingly to your vehicle id, for each simulation the id would change. The following scripts are not neccessarily need any arguments if not directly declared.

3. process_sumo_info.py: requires std vehicle file, speed 0, speed 1 and sumo distance file, output sumo_ground.json to show the speed and distance ground truth in sumo.

4. cal_veins_distandce.py: requires output_node_0/1.json file, output veins_distance.json to show the distance ground truth in veins.

5. cal_distance.py: requires processed_data_[id] for both vehicles, output cars_distance.json to show the distance ground truth in Carla.

6. cal_rssi_distance_origin.py: requires output_node_1.json (the following vehicle), output rssi_distance.json to show the rssi distance calculated. We use simple path loss model in veins.

7. improve_rssi_plot.py: requires rssi_distance.json and cars_distance.json, output the plots of methodologies we used to improve the rssi distance accuracy, for rssi could be extremely inaccurate sometimes. We use Polynomial Fitting and multipling Environment Factor to improve the accuracy. The result and error analysis are in the plots and terminal output.

8. cal_rssi_distance_improved.py: requires output_node_1.json, cars_distance.json and fitting_model.pkl, output improved_rssi_distance.json. We decide to use Polynomial Fitting to improve rssi estimation. The script would check if there is already a fitting model. If so, it would just use the model to calculate rssi distance. If not, it would do Polynomial Fitting, save the model, then calculate the rssi distance and outputs error analysis in the terminal.

9. plot.py: requires cars_distance.json, processed_data_[id].json (both vehicles), improved_rssi_distance.json, sumo_ground.json, veins_distance.json. Output is a series of plots. 
   
   Figure 1: shows the comparision of lidar distance measurement for both vehicles, rssi distance and carla ground truth with error analysis.
   
   Figure 2: shows the ground truth amang carla, sumo and veins, to see if the different corrdinate systems would cause errors in distance calculation.
   
   Figure 3: shows the rssi distance and veins ground truth.

   Figure 4: shows the sumo distance and speed to reflect the vehcile movements.

10. lidar_data_filter.py: requires lidar_semantic_[id].json and processed_data_[id].json, output is filtered_lidar_data_[id].json and how many frames are filtered. The filtered data length should be in the same. In overall_process.py, we only use average to processe the lidar data point cloud, which is simple and convinient, but not accurate enough for further analysis. We use processed timestamp to filter the original lidar sensor data for future use. In our case, we only use the lidar of following vehicle.

11. ekf-3D.py and ekf-3D-dynamic: requires processed_data_[id].json (both vehicles), filtered_lidar_data_[id].json (following vehicle) and improved_rssi_distance.json, output is a series of plots and save the filter to my_ekf_model.pkl.
    
    Figure 1: shows the kalman filter predicted distance with the real distance
    
    Figure 2: shows the predicted angle with the real angle

    Figure 3: shows the error in distance and angle estimation

    Figure 4: shows weights of filter input, lidar and rssi in our case. 
    
    The dynamic one includes some improvement on the original ekf, like adaptive noise adjustment.

12. attacker_model.py: requires my_ekf_model.pkl, processed_data_[id].json (both vehicles), filtered_lidar_data_[id].json (following vehicle) and improved_rssi_distance.json, output is a series of plots. 

    All the attacks starts 30s after the simulation and ends in 70s.

    Figure 1: the comparision of constant offset attack claimed distance and angle, filter estimated distance and angle, real distance and angle, and the detection rate. 
    
    In this attack we set a fixed offset (3m) to relative postion X and Y.

    Figure 2: the comparision of random offset attack claimed distance and angle, filter estimated distance and angle, real distance and angle, and the detection rate. 
    
    In this attack we set a random offset to rx and ry every 5s when attack starts, then last for 5s. The offset is randomly chose between (-3,-7), (3,7).

    Figure 3: the comparision of random jump attack claimed distance and angle, filter estimated distance and angle, real distance and angle, and the detection rate. 
    
    In this attack vehicle suddenly jumps to a new location when attack starts then last for 5s. The offset is randomly chose between (-3,-7), (3,7).
    
    Figure 4: the comparision of lane change attack claimed distance and angle, filter estimated distance and angle, real distance and angle, and the detection rate. 
    
    In this attack we add a fixed offset (3.5m as the lane width) of ry to pretend vehicle is in anther lane.

### Scripts used for dynamic test

The above scripts are used for static test, which means we record data then use it as the analysis input. For dynamic test, we only need to run corresponding carla scripts together with veins_carla, everything will be processed automaticly and returns a comprehensive plot containing distance, angle, errors and speed tracking for the vehicles. To do the dynamic test, we need to exchange RSSI value between Carla and veins_carla, so a new version of veins_carla is implemented, adding functions to establish a socket between C and python scripts. The simulation step is same as before, run carla serve and load map first, start carla script, press play button in sumo-gui, then run veins_carla. The physical-challenge response protocl would only start as veins_carla starting transmitting the RSSI value. During the dynamic test, you should see command line output of vehicle speed, distances and so on. Detailed protocol phase and duration would also be printed out if vehicles are in protocol stage.



    






