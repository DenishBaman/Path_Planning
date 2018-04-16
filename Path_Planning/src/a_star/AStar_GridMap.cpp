#include <a_star/AStar_GridMap.h>

void AStar_GridMap::create_grid()
{
    for (uint x = 0; x < grid_.size(); ++x)
    {
        for (uint y = 0; y < grid_.size(); ++y)
        {
            grid_[x][y].pose_X = x;
            grid_[x][y].pose_Y = y;
        }
    }
}

// Start position
void AStar_GridMap::start(int x, int y)
{
    start_pose_X_ = x; 
    start_pose_Y_ = y;
    grid_[start_pose_X_][start_pose_Y_].g_cost = 0.0;
    grid_[start_pose_X_][start_pose_Y_].h_cost = 0.0;
}

// Goal position
void AStar_GridMap::goal(int x, int y)
{
    goal_pose_X_ = x;
    goal_pose_Y_ = y;
    grid_[goal_pose_X_][goal_pose_Y_].g_cost = 0.0;
    grid_[goal_pose_X_][goal_pose_Y_].h_cost = 0.0;
}

// 
void AStar_GridMap::calc_node_weights(Node& parent_node, Node& successor_node)
{
    int delta_X = std::abs(successor_node.pose_X - parent_node.pose_X);
    int delta_Y = std::abs(successor_node.pose_Y - parent_node.pose_Y);

    if (delta_X > delta_Y)
    {
        successor_node.g_cost = parent_node.g_cost + 2*delta_Y + 
                                                     1*(delta_X - delta_Y);
    }
    else
    {
        successor_node.g_cost = parent_node.g_cost + 2*delta_X + 
                                                     1*(delta_Y - delta_X);
    }
    // Manhattan Distance
    //successor_node.h_cost = std::abs(successor_node.pose_X - goal_pose_X_) + 
      //                       std::abs(successor_node.pose_Y - goal_pose_Y_);

    // Diagonal Distance
    /* successor_node.h_cost = std::max_element(std::abs(successor_node.pose_X - goal_pose_X_), 
                             std::abs(successor_node.pose_Y - goal_pose_Y_)); */

    // Euclidean Disctance
    successor_node.h_cost = 2 * (std::sqrt(std::pow((successor_node.pose_X - goal_pose_X_),2) + 
                             std::pow((successor_node.pose_Y - goal_pose_Y_),2)));

}

inline bool operator == (const Node& lhs, const Node& rhs)
{
    return (lhs.pose_X == rhs.pose_X)&&(lhs.pose_Y == rhs.pose_Y);
}

bool contains(std::vector<Node>& list, Node& node)
{
    auto iter = std::find(std::begin(list), std::end(list), node);
    return (iter != list.end());
}

bool contains(std::set<std::pair<int, int> >& list, Node& node)
{
    return !(list.find(std::make_pair(node.pose_X, node.pose_Y)) == list.end());
}

std::vector<Node> AStar_GridMap::retrace_path(std::vector<Node>& closed_list)
{
    std::vector<Node> path;
    Node* start_node   = &grid_[start_pose_X_][start_pose_Y_];
    
    Node* current_node = &closed_list[0];

    while(current_node != start_node)
    {   
        path.push_back(*current_node);
        current_node = current_node->parent;
    }
    std::reverse(std::begin(path), std::end(path));
    
    return path;
}

bool AStar_GridMap::push_neighbours_to_list(std::set<std::pair<int, int> >& list,
                                            std::vector<Node>& closed_list, 
                                        int lower_idx_x, int lower_idx_y)
{
    Node& node = grid_[lower_idx_x][lower_idx_y];
        
    // check to the neghibour nodes
    for (int i = node.pose_X - 1; i <= node.pose_X + 1; i++)
    {
        for (int j = node.pose_Y - 1; j <= node.pose_Y + 1; j++)
        {
            if (!contains(closed_list, grid_[i][j]))
            {
                if (i == goal_pose_X_ && j == goal_pose_Y_)
                { 
                    return true;
                }

                if (i == node.pose_X && j == node.pose_Y)
                {continue;}
                else
                {
                    auto current_node = grid_[i][j];
                    calc_node_weights(node,current_node);

                    if (!contains(list, current_node) || list.empty())
                    {
                        list.insert(std::make_pair(i,j));
                        grid_[i][j] = current_node;
                    }
                    else
                    {
                        if (grid_[i][j].g_cost + 
                            grid_[i][j].h_cost 
                                >
                            current_node.g_cost + 
                            current_node.h_cost )
                        {
                            grid_[i][j] = current_node;
                        }
                    }
                }
            }
        }
    }
    return false;
}

void AStar_GridMap::generate_shortest_path()
{
    std::set<std::pair<int,int> > open_list;
    std::vector<Node> closed_list;
    open_list.insert(std::make_pair(start_pose_X_, start_pose_Y_));
    bool found_goal = false;

    while (!open_list.empty())
    {
        std::pair<int, int> lowest_idx;

        auto smaller = open_list.begin();
        for (std::set<std::pair<int,int> >::iterator elem  = open_list.begin(); 
                elem != open_list.end(); ++elem)
        {
            int smaller_f_cost = grid_[smaller->first][smaller->second].g_cost + 
                                  grid_[smaller->first][smaller->second].h_cost;
            
            int elem_f_cost = grid_[elem->first][elem->second].g_cost + 
                              grid_[elem->first][elem->second].h_cost ;

            if ( ( smaller_f_cost > 
                   elem_f_cost) || 
                  (smaller_f_cost == elem_f_cost && grid_[smaller->first][smaller->second].h_cost > 
                                                    grid_[elem->first][elem->second].h_cost))
            {
                smaller = elem;
            }
        }

        int lower_idx_x = smaller->first;
        int lower_idx_y = smaller->second;

        open_list.erase(smaller);
        closed_list.push_back(grid_[lower_idx_x][ lower_idx_y]);
        
        found_goal = push_neighbours_to_list(open_list, closed_list, lower_idx_x, lower_idx_y);
        
        if (found_goal)
        { break; }

    }
    closed_list.push_back(grid_[goal_pose_X_][goal_pose_Y_]);
    /*for (auto elem : closed_list)
    {
        std::cout << elem.first << "    " << elem.second << '\n';
    }*/
    std::vector<Node> shortest_path;
    draw_grid(closed_list);
  //  shortest_path =  retrace_path(closed_list);
    //draw_grid(shortest_path);
}

/*void AStar_GridMap::print()
{
    for (uint x = 0; x < grid_.size(); ++x)
    {
        for (uint y = 0; y < grid_[0].size(); ++y)
        {
            std::cout<< "( " << grid_[x][y].pose_X << ",";
            std::cout<< grid_[x][y].pose_Y << " )";

            std::cout << "    ";
        }
        std::cout << '\n';
    }
}
*/
void AStar_GridMap::draw_grid(std::vector<Node>& list)
{
    sf::RenderWindow window(sf::VideoMode(800,600), "Path Planning Search");

    int number_of_blocks_x = grid_size_X_/(2*node_radius);
    int number_of_blocks_y = grid_size_Y_/(2*node_radius);

    int block_width_in_pixel = 800/number_of_blocks_x;
    int block_height_in_pixel = 600/number_of_blocks_y;

    std::vector<sf::VertexArray> horiz_lines(number_of_blocks_x);
    std::vector<sf::VertexArray> vert_lines(number_of_blocks_y);
    std::vector<sf::CircleShape> circles(number_of_blocks_x*number_of_blocks_y);
    std::vector<sf::CircleShape> sol_circles(list.size());

    int height = block_height_in_pixel;
    for (uint i = 0; i < horiz_lines.size(); ++i)
    {
        horiz_lines[i] = sf::VertexArray(sf::Lines, 2);
        (horiz_lines[i])[0].position = sf::Vector2f(0, height);
        (horiz_lines[i])[1].position = sf::Vector2f(800, height);
        (horiz_lines[i])[0].color = sf::Color::Black;
        (horiz_lines[i])[1].color = sf::Color::Black;
        height += block_height_in_pixel;
    }

    int width = block_width_in_pixel;
    for (uint i = 0; i < vert_lines.size(); ++i)
    {
        vert_lines[i] = sf::VertexArray(sf::Lines, 2);
        (vert_lines[i])[0].position = sf::Vector2f(width,0);
        (vert_lines[i])[1].position = sf::Vector2f(width,600);
        (vert_lines[i])[0].color = sf::Color::Black;
        (vert_lines[i])[1].color = sf::Color::Black;
        width += block_width_in_pixel;
    }

    int idx = 0; int circle_radius = 3.5;
    for (uint i = block_width_in_pixel; i < 800; i += block_width_in_pixel)
    {
        for (uint j = block_height_in_pixel; j < 600; j += block_height_in_pixel)
        {
            circles[idx].setRadius(circle_radius);
            circles[idx].setPosition(sf::Vector2f(i - circle_radius, j - circle_radius));
            circles[idx].setFillColor(sf::Color(204,102,0));
            ++idx;
        }
    }

    sf::Font font;
    if (!font.loadFromFile("/home/dkbaman/a_star/Path_Planning/bebas_neue/BebasNeue-Regular.otf"))
    {
        std::cerr << "Can't load Font\n";
        exit(-1);
    }
    std::vector<sf::Text> txts(number_of_blocks_x*number_of_blocks_y);

    idx = 0; int pose_X_idx = 1, pose_Y_idx = 1;
    std::stringstream SS;

    for (int i = 0; i < (800 - block_width_in_pixel); i += block_width_in_pixel)
    {
        for (int j = 0; j < (600 - block_height_in_pixel); j += block_height_in_pixel)
        {
            SS << grid_[pose_X_idx][pose_Y_idx].pose_X << "," <<grid_[pose_X_idx][pose_Y_idx].pose_Y;
            txts[idx].setFont(font);
            txts[idx].setString(SS.str());
            txts[idx].setColor(sf::Color::Black);
            txts[idx].setCharacterSize(8);
            txts[idx].setPosition(i + (block_width_in_pixel/2) ,
                                  j + (block_height_in_pixel/2));
            ++idx; ++pose_Y_idx;
            SS.str("");
        }
        pose_Y_idx = 1;
        ++pose_X_idx;
    }

    idx = 0;

    for (auto elem : list)
    {
        sol_circles[idx].setRadius(5);
        sol_circles[idx].setPosition(sf::Vector2f(  elem.pose_X*block_width_in_pixel  - 5-(block_width_in_pixel/2), 
                                                    elem.pose_Y*block_height_in_pixel - 5-(block_height_in_pixel/2)));
        sol_circles[idx].setFillColor(sf::Color(255,0,0));
        ++idx;
    }


    sf::Event event;
    while(window.isOpen())
    {
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            { window.close(); }
        }

        window.clear(sf::Color::White);
        for (uint i = 0; i < horiz_lines.size(); ++i)
        {
            window.draw(horiz_lines[i]);
        }
        for (uint i = 0; i < vert_lines.size(); ++i)
        {
            window.draw(vert_lines[i]);
        }
        for (uint i = 0; i < circles.size(); ++i)
        {
            window.draw(circles[i]);
        }
        for (uint i = 0; i < sol_circles.size(); ++i)
        {
            window.draw(sol_circles[i]);
        }
        for (uint i = 0; i < txts.size(); ++i)
        {
            window.draw(txts[i]);
        }
        window.display();
    }
}
