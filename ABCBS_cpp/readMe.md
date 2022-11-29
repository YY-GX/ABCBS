# Document - ABCBS (CPP version)

## I. Introduction
Anytime bounded conflict-based search (ABCBS), based on related papers:
- [Suboptimal Variants of the Conflict-Based SearchAlgorithm for the Multi-Agent Pathfinding Problem](http://www.bgu.ac.il/~felner/2014/cbseShort.pdf) 
- [Anytime Focal Search with Applications](https://www.ijcai.org/Proceedings/2018/0199.pdf)

## II. File structure
```
.
├── main.cpp                      # Starter file
├── ./yaml-cpp                    # external library
├── ./inputs/input.yaml           # Input file
├── ./outputs/output.yaml         # Output file
└── ...                           # Other files

```

## III. Requirement

### 1. C++ version
**Minimum** version: C++ `17`

### External library
Use [cpp-yaml](https://github.com/jbeder/yaml-cpp) to read/write yaml file.

## IV. Arguments

### Argument explanations
- input: Path of input file (absolute path)                    
- output: Path of output file (absolute path)       
- wh: W_H in paper       
- wl: W_L in paper 
- gamma: Decay ratio for the W_H
- is_anytime: Whether to use anytime feature or not. If 'False', solution will be returned once ABCBS finds it   
- anytime_limitation: Time limitation for an anytime algorithm 

### Default values of arguments
```
       anytime_limitation: 30                            
                    gamma: 0.985         
                       wh: 1.1                           
                       wl: 1.1                  
                    input: My absolute path for input.yaml (change to yours)       
                   output: My absolute path for output.yaml (change to yours)                 
               is_anytime: false                                                                                       
```

## V. Steps for running
### 1. Set input file
**Note**: Different from `python` version, every value should be in form of `string`(i.e., add `""` to values).

Example:
```
agents:                         # Information (start, goal and name) of each agent
-   start: "[24, 26]"
    goal: "[2, 16]"
    name: "agent0"
-   start: "[31, 25]"
    goal: "[19, 30]"
    name: "agent1"
map:
    dimensions: "[32, 32]"      # Dimension of the map
    obstacles:                  # Information (position) of obstacles. Note: type is tuple
    - "[14, 15]"
    - "[9, 28]"
    - "[5, 5]"
```

### 2. Change input & output file paths
See Line `18` & Line `61` in `main.cpp`, change the input & output file path to yours.

### 3. Change other parameters
See Line `29` & Line `30` in `ABCBS.h`, change default values there. But default values are recommended.

### 4. Run
Directly compile && run `main.cpp` is ok.

paths of solution will be saved to `output.py`(under your own path).

### 5. Visualize paths
Use `visualize.py` to visualize solution. Command:
```
python3 visualize.py input.yaml output.yaml
```

## VI. Other
Python code structure is based on [multi_agent_path_planning](https://github.com/atb033/multi_agent_path_planning)