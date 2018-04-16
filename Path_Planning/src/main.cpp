#include <a_star/AStar_GridMap.h>
#include <SFML/Graphics.hpp>

int main(int argc, char** argv)
{
    AStar_GridMap path_grid(25,25,0.5);
    path_grid.create_grid();

    path_grid.start(23,19);
    path_grid.goal(1,1);
    path_grid.generate_shortest_path();

    //path_grid.draw_grid();

    return EXIT_SUCCESS;
}
