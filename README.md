# Sensor-based underwater tracking of marine object
Final project for course 34763 Autonomous Marine Robotics


# Development

## Getting Started

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


# Conda environment for local tests

1. Create your conda virtual environment (only TBD the first time)
    ```bash
    conda create --name AMR_Team9 python=3.10
    ```
1. Activate your conda environement
    ```bash
    conda env list
    conda activate AMR_Team9
    ```
1. Install requirements
    ```bash
    cd ~/AMR_Team9
    conda install -c conda-forge jupyterlab=4.0.7 notebook=7.0.6
    pip install -r dev_requirements.txt
    ```

# How to run docker environment

1. 

