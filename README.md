# Install

Tested on Linux Ubuntu systems with Docker version 19.03.6 and docker-compose version 1.28.2.

* [docker](http://www.docker.com)

    See also 
    [Post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall/).
    In particular, add your user to the `docker` group and log out and in again, before proceeding.

* [docker-compose](https://docs.docker.com/compose/install/)

    You need to install the latest version 1.28, check it with

        docker-compose --version


# Setup

* Clone this repository

        git clone https://github.com/nicod-man/DIAG_demo


# Run

On host OS, move to `bin` folder of this repository

        cd DIAG_demo/bin


* Start all services

    Start using OS display (it does not work with Nvidia cards)

        ./start.bash


* Test actions

        ./test_actions.bash


* Test plan

        ./test_plan.bash [<plan_name>|stop]

    Example:

        ./test_plan.bash DIAG_printer_2

    The robot goes to the printer and checks the light color, if it is red, the robot tells it to the blue person, then it goes back home.

    See other plans in the `plans` folder.


* Change light status

        ./setlight.bash [red|green]

    Run the plan to see different behaviors according to the light color.

* Stop all services

        ./stop.bash


# Development

For development or to run the last version of code, follow these additional instructions. 

* Download MARRtino apps

        git clone --depth clone https://nicodman@bitbucket.org/nicodman/marrtino_apps.git

* Set `MARRTINO_APPS_HOME`, `MODELS_HOME` environment variables

    Set
     - `MARRTINO_APPS_HOME` to  `marrtino_apps` folder
     - `MODELS_HOME` to `<path_to_models_folder>` folder
    

    In `~/.bashrc`

        export MARRTINO_APPS_HOME=<PATH_TO>/marrtino_apps
        export MODELS_HOME=<PATH_TO/models


* Open a new terminal before running the demo
    
    Check the settings

        echo $MARRTINO_APPS_HOME
        echo $MODELS_HOME

* Start all services in development mode

        ./start.bash dev
        
        
# Quick starter

In `~/.bashrc`, create some alias (i.e.:) both for testing plans and starting simulator:

        alias startdemo='cd <path_to_DIAG_demo>/bin && ./start.bash'
        alias startdemodev='cd <path_to_DIAG_demo>/bin && ./start.bash dev'
        alias stopdemo='cd <path_to_DIAG_demo>/bin && ./stop.bash'
        
        alias testplanN='cd <path_to_DIAG_demo>/bin && ./test_plan.bash DIAG_printer_N'
        
Then source the bashrc file `source $HOME/.bashrc`.
