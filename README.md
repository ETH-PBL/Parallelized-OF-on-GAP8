Parallelized Optical Flow Estimation on GAP8
====
This project provides the code accompanying the paper entitled [Parallelizing Optical Flow Estimation on an Ultra-Low Power RISC-V Cluster for Nano-UAV Navigation](https://arxiv.org/abs/2305.13055). For now the repository contains the most efficient implementation mentioned in the paper (Local Optim. - i.e., avoiding recalculations per point of interest).

Requirements
----

This project requires the [GAP SDK](https://github.com/GreenWaves-Technologies/gap_sdk). The demo can either be run in GVSoC or on a physical GAP8 board, e.g., [GAPuino](https://greenwaves-technologies.com/product/gapuino/) or the [AI-deck](https://store.bitcraze.io/collections/decks/products/ai-deck-1-1) of bitcraze.

Installation
----
In case you have the GAP SDK not set up, we recommend the usage of the provided docker installation.

- Run `docker compose up -d`
- Attach to the running container using `docker exec -it <container_name> /bin/bash`
- In the active terminal execute the following sequence of commands to install the GAP SDK and register for the Autotiler:
``` shell
cd /home/gap_user/ws/gap_sdk/
GAP_SDK # Select the appropriate GAP8 model
make all # Enter the requested information for the Autotiler when prompted
cd /home/gap_user/ws/src/ # This command brings you to the source folder of this project
```

Running the Code
----
The sample project can be executed using the following command

``` bash
make all run platform=gvsoc
```

or alternatively on the board (please follow the [official guide](https://github.com/GreenWaves-Technologies/gap_sdk) the connect to a board)

``` bash
make all run platform=board
```

The optical flow code will be executed on all eight cluster cores by default. For single core execution change the following line in the `main.c` file: 
``` c
uint8_t SINGLE_CORE = 0;
```

Citing this Work
----
If you found our work helpful in your research, we would appreciate if you cite it as follows:

```
@inproceedings{kuhne2022parallelizing,
  title={Parallelizing optical flow estimation on an ultra-low power risc-v cluster for nano-uav navigation},
  author={K{\"u}hne, Jonas and Magno, Michele and Benini, Luca},
  booktitle={2022 IEEE International Symposium on Circuits and Systems (ISCAS)},
  pages={301--305},
  year={2022},
  organization={IEEE}
}
```
