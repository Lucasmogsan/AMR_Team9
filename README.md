# Sensor-based underwater tracking of marine object
Final project for course 34763 Autonomous Marine Robotics

# Development

## Git (Downloading and contributing)

1. Clone the repository
    ```bash
    git clone https://github.com/Lucasmogsan/AMR_Team9.git
    ```
1. Navigate into the cloned repository
    ```bash
    cd AMR_Team9
    ```
1. Make a new branch
    ```bash
    git branch new_branch
    ```
1. Switch to your new branch
    ```bash
    git checkout new_branch
    ```
1. Make changes to the code and commit them
    ```bash
    git add .
    git commit -m "Your descriptive commit message"
    ```
1. Push your changes to the new branch
    ```bash
    git push origin new_branch
    ```
1. To merge to main go to the [GitHub repository](https://github.com/Lucasmogsan/AMR_Team9.git)


## Conda environment (Local tests)

1. Create your conda virtual environment (only TBD the first time)
    ```bash
    conda create --name amr_project python=3.10
    ```
1. Activate your conda environement
    ```bash
    conda env list
    conda activate amr_project
    ```
1. Install requirements
    ```bash
    cd ~/amr_project
    conda install -c conda-forge jupyterlab=4.0.7 notebook=7.0.6
    pip install -r dev_requirements.txt
    ```

## Docker (How to run docker environment)

Install docker and docker compose

Docker:
https://docs.docker.com/engine/install/

Docker compose:
https://docs.docker.com/compose/install/

Build the image:
```bash
docker compose build
```

Run the container:
```bash
docker compose up dev
```

Connect to the container:
```bash
docker exec -it amr_team9-dev-1 bash
```


## ROS commands

To give permission to python files in the container use the shell script
```bash
./shell_scripts/give_permission.sh
```

Build and source environment
```bash
catkin build
source devel/setup.bash
```

Launch the main (world, robot, ooi etc)
```bash
roslaunch amr_prj run.launch 
```

Use teleop to control (if desired)
```bash
roslaunch uuv_teleop uuv_keyboard_teleop.launch uuv_name:=bluerov2
roslaunch uuv_teleop uuv_keyboard_teleop.launch uuv_name:=ooi
```

