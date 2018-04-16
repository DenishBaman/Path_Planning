# A-Star : Feasible path searching algorithm

*Set-up A-Star repository* 

1. Make a directory and download 
```
cd
mkdir ~/a_star
cd ~/a_star/

cd ~/Download && git clone https://github.com/DenishBaman/Path_Planning.git

```

2. Install [CMake](http://www.edparrish.net/common/sfmlcb-linux.html) and [SFML](https://www.sfml-dev.org/tutorials/2.4/start-linux.php) in linux environment. For [Windows or Mac](https://www.sfml-dev.org/tutorials/2.4/compile-with-cmake.php) environment. 


3. Update the **BebasNeue-Regular.otf** file location in AStar_GridMap::draw_grid() in a **src/a_star/AStar_GridMap.cpp** file.

4. Build the project
```
cd ~/Path_Planning
chmod +x build_project.sh

./build_project.sh
 
```

5. Run the main file using command line or any IDE (e.g. visula studio)
